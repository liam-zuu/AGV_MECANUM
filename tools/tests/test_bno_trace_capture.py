"""Tests for tools/bno_trace_capture.py on a synthetic trace (no hardware).

Run: python -m pytest tools/tests -q
"""
import json
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import bno_trace_capture as bt  # noqa: E402


def rd(i, t, ch, seq, ln, payload=b'', flags=0, cs=40):
    d = bytes([ln & 0xFF, ln >> 8, ch, seq]) + payload
    d = (d + bytes(12))[:12]
    return f'R,{i},2,{t},{cs},{ln},{flags},{d.hex().upper()}'


def report(i, t, seq, rid, flags):
    # channel 3 input report: 0xFB + 4-byte base delta, then the sensor report id
    return rd(i, t, 3, seq, 23, bytes([0xFB, 0, 0, 0, 0, rid, 0, 0]), flags=flags)


def make_raw(period=50000, n=6, lost=0, s5=False):
    """RST, INT + advertisement read (full bytes kept), one write, then n bursts of
    INT + 3 reports (accel, gyro, game RV) every `period` us."""
    lines = ['ets Jun  8 2016 boot junk', 'TRACE READY v1 period_us=50000 alt_us=10000 boot_s=60 cap=7000']
    recs, i, t = [], 0, 1000
    recs.append(f'R,{i},4,{t},0,0,0,'); i += 1                       # RST
    t += 100000
    recs.append(f'R,{i},1,{t},0,0,0,'); i += 1                       # INT
    recs.append(rd(i, t + 30, 0, 0, 276)); i += 1                    # advertisement
    recs.append(f'B,{i - 1},280,' + '00' * 280)
    recs.append(f'R,{i},3,{t + 5000},20,5,0,0500020001F9'); i += 1   # host write, channel 2
    for k in range(n):
        t += period
        recs.append(f'R,{i},1,{t},0,0,0,'); i += 1
        for j, rid in enumerate((0x01, 0x02, 0x08)):
            fl = (bt.F_INT_LOW if j == 0 else 0) | (bt.F_DELAYED if s5 else 0)
            recs.append(report(i, t + 50 + 200 * j, k * 3 + j, rid, fl))
            i += 1
    phase = 'S5' if s5 else 'S1'
    lines += ['TRACE COUNTS phase=%s valid=1 A=%d G=%d R=%d' % (phase, n, n, n),
              f'TRACE BEGIN phase={phase} cap=7000 n={i} lost={lost} pool_used=282 rec_bytes=24']
    lines += recs
    lines += [f'TRACE END phase={phase}', 'TRACE DONE']
    return lines


def test_parse_and_summary():
    p = bt.parse(make_raw())
    assert p['done'] and p['ready']['period_us'] == '50000'
    s = bt.summarize(p['phases']['S1'])
    assert s['n_by_type'] == {'INT': 7, 'RD': 19, 'RST': 1, 'WR': 1}
    assert set(s['report_ids']) == {'0x01', '0x02', '0x08'}
    assert s['report_ids']['0x02']['interval_us']['mean'] == 50000
    assert s['rd_by_channel']['3']['lengths'] == [23]
    assert s['rd_by_channel']['0']['lengths'] == [276]
    # first read after each INT: 30 us for the advertisement, 50 us for each report burst
    assert s['int_to_cs_us']['min'] == 30 and s['int_to_cs_us']['max'] == 50
    assert s['reads_without_new_int'] == 12          # 2nd and 3rd report of each burst
    assert s['sequence'][0] == ['RST', None, None, 0]
    assert s['sequence'][1] == ['RD', 0, 0, 276]
    assert s['counts'] == {'valid': '1', 'A': '6', 'G': '6', 'R': '6'}


def test_full_bytes_kept():
    p = bt.parse(make_raw())
    assert len(p['phases']['S1']['bytes'][2]) == 280


def test_delayed_reads_s5():
    s = bt.summarize(bt.parse(make_raw(s5=True))['phases']['S5'])
    assert s['delayed_reads']['n'] == 18
    assert s['delayed_reads']['int_low_pct'] == round(100 * 6 / 18, 1)


def test_analyse_writes_files_and_compare(tmp_path):
    a, b = tmp_path / 'car', tmp_path / 'hil'
    for d, raw in ((a, make_raw()), (b, make_raw(period=52000))):
        d.mkdir()
        bt.save(str(d), raw, SimpleNamespace(), 'board: x\n')
        assert (d / 'records_S1.csv').exists() and (d / 'summary.txt').exists() and (d / 'meta.yaml').exists()
        assert json.loads((d / 'summary.json').read_text(encoding='utf-8'))['done']
    text = bt.compare(str(a), str(b))
    assert 'sequence (dir, channel, len) of the first' in text and ': same' in text
    assert "report 0x02: A {'n': 6, 'interval_us': {'n': 5, 'mean': 50000" in text
    assert "B {'n': 6, 'interval_us': {'n': 5, 'mean': 52000" in text


def test_compare_finds_sequence_difference(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    # B: the advertisement (record 2) is 272 bytes instead of 276
    raw_b = [bt_rd_272(l) if l.startswith('R,2,2,') else l for l in make_raw()]
    for d, raw in ((a, make_raw()), (b, raw_b)):
        d.mkdir()
        bt.save(str(d), raw, SimpleNamespace())
    text = bt.compare(str(a), str(b))
    assert 'sequence differs at transfer 1' in text


def bt_rd_272(line):
    p = line.split(',')
    return rd(int(p[1]), int(p[3]), 0, 0, 272)


def test_lost_records_noted(tmp_path):
    d = tmp_path / 'x'
    d.mkdir()
    _, _, text = bt.save(str(d), make_raw(lost=123), SimpleNamespace())
    assert '123 records lost' in text
