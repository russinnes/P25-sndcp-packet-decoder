import sys, argparse
sys.path.insert(0, '.')
import dual_chaser as d

def mk():
    a = argparse.Namespace(rid_prefix=100, rid_range=[], assume_afrrcs_plan=True, follow_opts0=False, home_hz=0, min_dwell=1.0, release_quiet=2.0,
                           routine_quiet=1.0, max_dwell=12.0, ack_grace=1.0, preempt_after=1.5, routine_text_max=8)
    tunes = []
    c = d.Chaser(a, lambda hz: tunes.append(hz) or 5.0, lambda *_: None)
    return c, tunes

def mk_noplan():
    c, tunes = mk(); c.iden = {}; return c, tunes

def g(rid, ch, opts=1): return {"cat": "DATA_GRANT", "name": "x", "rid": rid, "ch": ch, "opts": opts}

# 1 channel plan checks
plan = {0: d.ASSUME_PLAN[0]}
assert d.channel_to_hz(237, plan) == 772043750 and d.channel_to_hz(361, plan) == 773593750 and d.channel_to_hz(161, plan) == 771093750
assert d.channel_to_hz(0x1005, plan) is None            # identifier 1 not announced
# real IDEN_UP captured from a live control channel: BD000325E064092B0CBA
f = d.parse_tsbk(b'\x00\x00' + bytes.fromhex('BD000325E064092B0CBA3C11'))
assert f["cat"] == "IDEN" and f["iden"] == 0 and f["plan"] == {"base_hz": 769081250, "spacing_hz": 12500, "toff_hz": 30000000}, f
assert d.parse_tsbk(b'\x00\x00' + bytes.fromhex('BDA00325E064092B0CBA3C11')) is None   # non-standard mfid ignored
# 2 TSBK parse of a real-shaped SN_DATA_CHN_GNT: opcode 0x14, opts 1, ch_t 420, ch_r 0xFFFF, rid 1006730
rid, ch = 1006730, 420
tsbk = (0x14 << 88) | (0x00 << 80) | (1 << 72) | (ch << 56) | (0xFFFF << 40) | (rid << 16)
raw = bytes([0x14, 1]) + (tsbk >> 16).to_bytes(10, 'big') + b'\x00\x00'
f = d.parse_tsbk(raw); assert f and f["rid"] == rid and f["ch"] == ch and f["opts"] == 1, f
# 2b plan learned from the tower: grant before IDEN_UP is not chased, after it is
c, t = mk_noplan()
c.on_msg(g(1006730, 420), 1.0); assert c.mode == "IDLE" and c.c["grants_no_plan"] == 1 and not t
c.on_msg(d.parse_tsbk(b'\x00\x00' + bytes.fromhex('BD000325E064092B0CBA3C11')), 2.0)
c.on_msg(g(1006730, 420), 3.0); assert c.mode == "CHASE" and t == [774331250], t
# 2c rid filters
a2 = argparse.Namespace(rid_prefix=None, rid_range=[(2000000, 2009999)])
assert d.rid_wanted(2003173, a2) and not d.rid_wanted(1006730, a2)
assert d.rid_wanted(1, argparse.Namespace(rid_prefix=None, rid_range=[]))
# 3 chase, busy-miss, piggyback, same-channel, release
c, t = mk()
c.on_msg(g(1006730, 420), 100.0); assert c.mode == "CHASE" and t == [d.channel_to_hz(420, plan)]
c.on_msg(g(1006730, 420), 100.1); assert c.c["repeat_grants"] == 1
c.on_msg(g(1006773, 300), 100.5); assert c.c["missed_busy"] == 1          # busy, different ch
c.on_msg(g(1006773, 420), 100.6); assert c.c["piggyback"] == 1             # busy, same ch
c.on_hold(1006730, 100.8); assert c.had_ping
c.tick(101.2); assert c.mode == "CHASE"                                     # < routine_quiet since last ping
c.tick(101.9); assert c.mode == "IDLE" and c.c["rel_routine-quiet"] == 1
c.on_msg(g(1006730, 420), 120.0); assert c.c["same_channel"] == 1 and len(t) == 1   # no retune needed
# 4 opts0 skipped, other prefix ignored
c.tick(125); c.on_msg(g(1006730, 300, 0), 130.0); assert c.c["skipped_opts0"] == 1
c.on_msg(g(5901330, 300), 131.0); assert c.mode == "IDLE" and c.c["grants_other_prefix"] == 1
# 5 preempt when nothing has decoded
c, t = mk()
c.on_msg(g(1006730, 420), 0.0); c.on_msg(g(1006773, 300), 0.5); assert c.c["missed_busy"] == 1
c.on_msg(g(1006773, 300), 1.6); assert c.c["preempted"] == 1 and c.rid == 1006773
# 6 text completion releases after grace
c, t = mk(); c.on_msg(g(1006730, 420), 0.0); c.on_hold(1006730, 0.4)
c.on_tms(1006730, "x" * 40, False, 1.0); c.tick(1.5); assert c.mode == "CHASE"; c.tick(2.1); assert c.mode == "IDLE"
# 7 max dwell
c, t = mk(); c.on_msg(g(1006730, 420), 0.0)
for i in range(1, 140): c.on_hold(1006730, i * 0.1); c.tick(i * 0.1)
assert c.mode == "IDLE" and c.c["rel_max-dwell"] == 1
print("all chaser tests passed")