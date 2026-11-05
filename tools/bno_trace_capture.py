"""
bno_trace_capture.py: record and compare the ESP32 <-> BNO085 SPI trace (QD 26, G0 v1 section 7).

The BNO085 talks to the ESP32 over SPI + INT + RST. Env imu_trace_20 / imu_trace_100
(src/tests/test_imu_trace.c, library hooks under -DBNO_TRACE) records every SPI transfer,
INT edge and reset in RAM and prints it over the USB serial port (UART0, 921600) after each
scenario. Run the same env on the car (real chip) and on the HIL bench (H7 emulator), then
compare the two recordings.

  python tools/bno_trace_capture.py --port COM4 --board car --out <05_DATA/results/BNO_TRACE/<date>_run01_car_20hz>
  python tools/bno_trace_capture.py --port COM5 --board hil --out <..._run02_hil_20hz>
  python tools/bno_trace_capture.py --compare <car_dir> <hil_dir>
  python tools/bno_trace_capture.py --parse-only <dir>          # re-read raw.txt

Output in --out (refuses a folder that already holds data): raw.txt (every line as received),
records_<phase>.csv, summary.json, summary.txt, meta.yaml.
The comparison is descriptive (G0 v1 section 7 sets no thresholds): init sequence, packet
lengths and channels, report ids, report period, INT -> chip select delay, behaviour when the
host reads late (S5) and after RST (S4).
"""
import argparse
import csv
import json
import os
import statistics
import subprocess
import sys
import time

TYPES = {1: 'INT', 2: 'RD', 3: 'WR', 4: 'RST', 5: 'TMO', 6: 'MARK'}
F_INT_LOW, F_FAIL, F_BYTES, F_DELAYED = 1, 2, 4, 8
HERE = os.path.dirname(os.path.abspath(__file__))
FW_REPO = os.path.dirname(HERE)
INIT_SHOWN = 40


def git_state(repo):
    def git(*a):
        return subprocess.run(['git', '-C', repo] + list(a), capture_output=True, text=True, check=True).stdout
    try:
        changed = [l[3:] for l in git('status', '--porcelain').splitlines() if l[3:] and not l[3:].startswith('.vscode/')]
        return dict(commit=git('rev-parse', '--short', 'HEAD').strip(),
                    branch=git('rev-parse', '--abbrev-ref', 'HEAD').strip(), dirty=bool(changed), modified=changed)
    except (OSError, subprocess.CalledProcessError) as e:
        return dict(commit='UNKNOWN', branch='UNKNOWN', dirty=True, modified=[f'git error: {e}'])


# ---------------------------------------------------------------- parse

def kv(line):
    return dict(p.split('=', 1) for p in line.split() if '=' in p)


def parse(lines):
    """raw lines -> {'ready': {...}, 'done': bool, 'phases': {name: phase}}."""
    out = {'ready': None, 'done': False, 'phases': {}, 'other_lines': 0}
    cur, counts = None, {}
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith('TRACE READY'):
            out['ready'] = kv(line)
        elif line.startswith('TRACE COUNTS'):
            c = kv(line)
            counts[c.pop('phase')] = c
        elif line.startswith('TRACE BEGIN'):
            h = kv(line)
            cur = {'header': h, 'records': [], 'bytes': {}}
            out['phases'][h['phase']] = cur
        elif line.startswith('TRACE END'):
            cur = None
        elif line.startswith('TRACE DONE'):
            out['done'] = True
        elif cur is not None and line.startswith('R,'):
            p = line.split(',')
            cur['records'].append(dict(i=int(p[1]), type=TYPES.get(int(p[2]), p[2]), t_us=int(p[3]), a=int(p[4]),
                                       b=int(p[5]), flags=int(p[6]), d=bytes.fromhex(p[7]) if len(p) > 7 and p[7] else b''))
        elif cur is not None and line.startswith('B,'):
            _, i, n, hx = line.split(',', 3)
            cur['bytes'][int(i)] = bytes.fromhex(hx)
        else:
            out['other_lines'] += 1
    for name, c in counts.items():
        if name in out['phases']:
            out['phases'][name]['counts'] = c
    return out


# ---------------------------------------------------------------- summarize

def stats(xs):
    if not xs:
        return None
    return dict(n=len(xs), mean=round(statistics.fmean(xs), 1), min=min(xs), max=max(xs))


def transfer(r):
    """(direction, channel, seq, shtp_len) of an RD/WR record from its first bytes."""
    d = r['d']
    if len(d) < 4:
        return (r['type'], None, None, r['b'])
    ln = (d[0] | (d[1] << 8)) & 0x7FFF
    return (r['type'], d[2], d[3], ln)


def report_id(r):
    """Sensor report id of a channel-3 input report: 0xFB base timestamp (5 bytes), then the id."""
    d = r['d']
    if len(d) >= 10 and d[2] == 3 and d[4] == 0xFB:
        return d[9]
    return None


def summarize(ph):
    recs = ph['records']
    by = {}
    for r in recs:
        by.setdefault(r['type'], []).append(r)
    rd = [r for r in by.get('RD', []) if not r['flags'] & F_FAIL]
    s = {'header': ph['header'], 'counts': ph.get('counts'),
         'n_by_type': {k: len(v) for k, v in sorted(by.items())},
         'rd_fail': sum(1 for r in by.get('RD', []) if r['flags'] & F_FAIL),
         'wr_fail': sum(1 for r in by.get('WR', []) if r['flags'] & F_FAIL)}
    chans = {}
    for r in rd:
        _, ch, _, ln = transfer(r)
        chans.setdefault(ch, []).append((r['t_us'], ln))
    s['rd_by_channel'] = {}
    for ch, v in sorted(chans.items(), key=lambda kv: (kv[0] is None, kv[0])):
        ts = [t for t, _ in v]
        s['rd_by_channel'][str(ch)] = dict(n=len(v), lengths=sorted({ln for _, ln in v}),
                                          interval_us=stats([b - a for a, b in zip(ts, ts[1:])]))
    ids = {}
    for r in rd:
        rid = report_id(r)
        if rid is not None:
            ids.setdefault(rid, []).append(r['t_us'])
    s['report_ids'] = {f'0x{k:02X}': dict(n=len(v), interval_us=stats([b - a for a, b in zip(v, v[1:])]))
                       for k, v in sorted(ids.items())}
    # INT -> chip select: last INT before each read (reads with no INT since the previous read counted apart)
    delays, no_int, last_int, used = [], 0, None, True
    for r in recs:
        if r['type'] == 'INT':
            last_int, used = r['t_us'], False
        elif r['type'] == 'RD':
            if last_int is not None and not used:
                delays.append(r['t_us'] - last_int)
                used = True
            else:
                no_int += 1
    s['int_to_cs_us'] = stats(delays)
    s['reads_without_new_int'] = no_int
    s['cs_low_us'] = stats([r['a'] for r in rd])
    s['int_low_before_cs_pct'] = round(100.0 * sum(1 for r in rd if r['flags'] & F_INT_LOW) / len(rd), 1) if rd else None
    late = [r for r in by.get('RD', []) if r['flags'] & F_DELAYED]
    s['delayed_reads'] = dict(n=len(late), int_low_pct=round(100.0 * sum(1 for r in late if r['flags'] & F_INT_LOW)
                                                             / len(late), 1) if late else None)
    s['sequence'] = [list(transfer(r)) for r in recs if r['type'] in ('RD', 'WR', 'RST', 'TMO')][:INIT_SHOWN]
    s['marks'] = [(r['t_us'], r['a'], r['b']) for r in by.get('MARK', [])]
    return s


def write_records(path, ph):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['i', 'type', 't_us', 'a', 'b', 'flags', 'first_bytes_hex', 'full_bytes_hex'])
        for r in ph['records']:
            w.writerow([r['i'], r['type'], r['t_us'], r['a'], r['b'], r['flags'], r['d'].hex(),
                        ph['bytes'].get(r['i'], b'').hex()])


def summary_text(parsed, summ):
    L = [f"READY: {parsed['ready']}", f"DONE: {parsed['done']}  (lines not understood: {parsed['other_lines']})"]
    for name, s in summ.items():
        h = s['header']
        L += ['', f"== {name}: {h.get('n')} records, cap {h.get('cap')}, lost {h.get('lost')}",
              f"  counts from imu_get: {s['counts']}", f"  records by type: {s['n_by_type']}  rd_fail={s['rd_fail']} wr_fail={s['wr_fail']}",
              f"  INT -> CS [us]: {s['int_to_cs_us']}   reads without a new INT: {s['reads_without_new_int']}",
              f"  CS low [us]: {s['cs_low_us']}   INT still low before CS: {s['int_low_before_cs_pct']} %",
              f"  delayed reads (S5): {s['delayed_reads']}"]
        for ch, c in s['rd_by_channel'].items():
            L.append(f"  read channel {ch}: n={c['n']} lengths={c['lengths']} interval={c['interval_us']}")
        for rid, c in s['report_ids'].items():
            L.append(f"  report {rid}: n={c['n']} interval={c['interval_us']}")
        L.append(f"  first transfers (dir, channel, seq, len): {s['sequence'][:12]} ...")
        if h.get('lost') not in (None, '0'):
            L.append(f"  NOTE: buffer full, {h.get('lost')} records lost after record {h.get('n')}")
    return '\n'.join(L)


def save(out, lines, args, extra_meta=''):
    with open(os.path.join(out, 'raw.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return analyse(out, args, extra_meta)


def analyse(out, args, extra_meta=''):
    with open(os.path.join(out, 'raw.txt'), encoding='utf-8') as f:
        lines = f.read().splitlines()
    parsed = parse(lines)
    summ = {name: summarize(ph) for name, ph in parsed['phases'].items()}
    for name, ph in parsed['phases'].items():
        write_records(os.path.join(out, f'records_{name}.csv'), ph)
    with open(os.path.join(out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump({'ready': parsed['ready'], 'done': parsed['done'], 'phases': summ}, f, indent=1)
    text = summary_text(parsed, summ)
    with open(os.path.join(out, 'summary.txt'), 'w', encoding='utf-8') as f:
        f.write(text + '\n')
    if extra_meta:
        with open(os.path.join(out, 'meta.yaml'), 'w', encoding='utf-8') as f:
            f.write(extra_meta)
    return parsed, summ, text


# ---------------------------------------------------------------- compare

def compare(dir_a, dir_b):
    A = json.load(open(os.path.join(dir_a, 'summary.json'), encoding='utf-8'))
    B = json.load(open(os.path.join(dir_b, 'summary.json'), encoding='utf-8'))
    L = [f'A = {dir_a}', f'B = {dir_b}', f"READY A {A['ready']}", f"READY B {B['ready']}"]
    for name in sorted(set(A['phases']) | set(B['phases'])):
        a, b = A['phases'].get(name), B['phases'].get(name)
        L.append(f'\n== {name}')
        if a is None or b is None:
            L.append(f"  only in {'B' if a is None else 'A'}")
            continue
        for key in ('n_by_type', 'rd_fail', 'wr_fail', 'counts', 'int_to_cs_us', 'reads_without_new_int',
                    'cs_low_us', 'int_low_before_cs_pct', 'delayed_reads'):
            same = 'same' if a[key] == b[key] else 'DIFF'
            L.append(f'  {key:24s} {same:4s}  A {a[key]}  |  B {b[key]}')
        for k in sorted(set(a['rd_by_channel']) | set(b['rd_by_channel'])):
            ca, cb = a['rd_by_channel'].get(k), b['rd_by_channel'].get(k)
            L.append(f"  channel {k:4s} lengths A {ca and ca['lengths']} | B {cb and cb['lengths']};"
                     f"  interval A {ca and ca['interval_us']} | B {cb and cb['interval_us']}")
        for k in sorted(set(a['report_ids']) | set(b['report_ids'])):
            ra, rb = a['report_ids'].get(k), b['report_ids'].get(k)
            L.append(f'  report {k}: A {ra} | B {rb}')
        sa, sb = a['sequence'], b['sequence']
        # compare direction, channel, length (seq numbers differ by design)
        key = lambda t: (t[0], t[1], t[3])
        first = next((i for i, (x, y) in enumerate(zip(sa, sb)) if key(x) != key(y)), None)
        if first is None and len(sa) == len(sb):
            L.append(f'  sequence (dir, channel, len) of the first {len(sa)} transfers: same')
        else:
            i = first if first is not None else min(len(sa), len(sb))
            L.append(f'  sequence differs at transfer {i}: A {sa[i:i + 3]} | B {sb[i:i + 3]}')
    return '\n'.join(L)


# ---------------------------------------------------------------- capture

def capture(port, timeout):
    try:
        import serial
    except ImportError:
        sys.exit('pyserial is missing: python -m pip install pyserial')
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 921600, 0.5
    s.dtr = False; s.rts = False          # avoid download mode (as imu_capture.py)
    s.open()
    s.reset_input_buffer()
    s.rts = True; time.sleep(0.15); s.rts = False
    lines, t_end, phase = [], time.time() + timeout, None
    while time.time() < t_end:
        line = s.readline().decode(errors='replace').rstrip('\r\n')
        if not line:
            continue
        lines.append(line)
        if line.startswith(('TRACE READY', 'TRACE BEGIN', 'TRACE END', 'TRACE COUNTS')):
            print(line, flush=True)
        if line.startswith('TRACE DONE'):
            print(line)
            break
    s.close()
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port')
    ap.add_argument('--board', choices=['car', 'hil'], help='car = ESP32 on the AGV with the real BNO085; '
                    'hil = standalone ESP32 of the HIL bench reading the H7 emulator')
    ap.add_argument('--out')
    ap.add_argument('--timeout', type=float, default=400.0, help='seconds to wait for TRACE DONE')
    ap.add_argument('--h7', default='', help='H7 commit flashed (board hil)')
    ap.add_argument('--notes', default='')
    ap.add_argument('--parse-only', metavar='DIR')
    ap.add_argument('--compare', nargs=2, metavar=('DIR_A', 'DIR_B'))
    a = ap.parse_args()
    for st in (sys.stdout, sys.stderr):
        if hasattr(st, 'reconfigure'):
            st.reconfigure(encoding='utf-8')
    if a.compare:
        print(compare(*a.compare))
        return 0
    if a.parse_only:
        print(analyse(a.parse_only, a)[2])
        return 0
    if not (a.port and a.board and a.out):
        sys.exit('--port, --board and --out are needed to record')
    if os.path.isdir(a.out) and os.listdir(a.out):
        sys.exit(f'{a.out} already holds data; use a new runNN folder')
    os.makedirs(a.out, exist_ok=True)
    fw = git_state(FW_REPO)
    lines = capture(a.port, a.timeout)
    meta = (f"# BNO085 SPI trace (tools/bno_trace_capture.py, QD 26 / G0 v1 section 7)\n"
            f"date: {time.strftime('%Y-%m-%d %H:%M:%S')}\nboard: {a.board}\nport: {a.port}\n"
            f"esp32_firmware_commit: {fw['commit']}  # branch {fw['branch']}, dirty={fw['dirty']}"
            f" (read from the PC tree at the end of the run: check it is the flashed one)\n"
            f"h7_firmware: \"{a.h7 or ('FILL IN' if a.board == 'hil' else 'n/a')}\"\n"
            f"lines: {len(lines)}\nnotes: \"{a.notes}\"\n")
    parsed, summ, text = save(a.out, lines, a, meta)
    print(text)
    if not parsed['done']:
        print(f'WARNING: no TRACE DONE within {a.timeout} s')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
