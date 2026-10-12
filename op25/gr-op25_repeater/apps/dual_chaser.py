#!/usr/bin/env python3
"""
dual_chaser.py -- two-SDR "wait and chase" controller for AFRRCS data grants.

    SDR #1  (OP25 instance A): parked on the control channel forever,
            exports every TSBK to UDP 127.0.0.1:51002 (already done by
            p25p1_fdma.cc's tsbk_export).  Never retuned.
    SDR #2  (OP25 instance B): the chaser.  This script retunes it on
            each data grant for a 411xxxx radio, holds while OP25 reports
            decoded activity (UDP 51001 HOLD, 51003 TMS text), then lets
            it sit where it is (or goes --home-hz).

Differences from op25_sdr_only.py (single SDR):
  * The CC never goes dark, so there is no CC mode / cc_settle / release-to-CC.
  * Grants that arrive while busy are COUNTED and logged (BUSY-MISS), not
    silently ignored, so you can see what the single chaser costs you.
  * A grant on the channel we are already on is free (no retune).
  * Measures grant -> tune-done -> first decode, per chase, so the timing
    assumption can be checked on real hardware instead of guessed.
  * --replay lets you run the same state machine over an old grant log
    (the Uniden-monitor format) with a simulated clock.

Tune back-ends (pick one):
  --tune-http URL   OP25 http_server.py POST set_freq   (the fixed ~1-3 ms one,
                    NOT the old 200 ms wait-for-reply version)
  --tune-unix PATH  sdr_feed.py control socket  {"command":"set_freq","freq":hz}\\n
  --dry-run         print only
"""
import argparse
import collections
import heapq
import json
import os
import re
import socket
import statistics
import sys
import threading
import time
import urllib.request
from datetime import datetime

# ----------------------------------------------------------------- TSBK ---
OPCODES = {
    0x3D: ("IDEN", "IDEN_UP"),
    0x34: ("IDEN", "IDEN_UP_VU"),
    0x10: ("DATA_GRANT", "IND_DATA_CH_GRANT"),
    0x11: ("DATA_GRANT", "GRP_DATA_CH_GRANT"),
    0x14: ("DATA_GRANT", "SN_DATA_CHN_GNT"),
    0x15: ("DATA_PAGE", "SN_DATA_PAGE_REQ"),
}
INVALID_CH = {0, 0xFFFF}
# Fallback plan, used ONLY with --assume-afrrcs-plan (the Alberta AFRRCS 700 MHz plan).
# Normally the plan comes from the tower itself (IDEN_UP / IDEN_UP_VU, see Chaser.iden).
ASSUME_PLAN = {0: {"base_hz": 769081250, "spacing_hz": 12500, "toff_hz": 30000000}}


def parse_iden(tsbk, opcode):
    """TIA-102.AABC-B 6.2.9 (IDEN_UP, 0x3D) / 6.2.29 (IDEN_UP_VU, 0x34).
    `tsbk` is the 96-bit value (10 octets + 16 CRC bits zero).  Verified against a live AFRRCS
    capture: BD000325E064092B0CBA -> iden 0, base 769.08125 MHz, spacing 12.5 kHz, tx +30 MHz."""
    v = tsbk >> 16
    iden = (v >> 60) & 0xF
    spacing = (v >> 32) & 0x3FF
    base = v & 0xFFFFFFFF
    if opcode == 0x3D:
        toff = (v >> 42) & 0x1FF                      # 9 bits: sign + value in 250 kHz steps
        sign = 1 if (toff >> 8) & 1 else -1
        toff_hz = sign * (toff & 0xFF) * 250000
    else:                                             # 0x34, VHF/UHF: 14-bit offset in channel-spacing steps
        toff = (v >> 42) & 0x3FFF
        sign = 1 if (toff >> 13) & 1 else -1
        toff_hz = sign * (toff & 0x1FFF) * spacing * 125
    return iden, {"base_hz": base * 5, "spacing_hz": spacing * 125, "toff_hz": toff_hz}


def channel_to_hz(ch, iden_table):
    """16-bit channel id = 4-bit identifier + 12-bit channel number.  Returns the DOWNLINK (tower->radio)
    frequency, or None if that identifier hasn't been announced yet."""
    p = iden_table.get((ch >> 12) & 0xF)
    if not p:
        return None
    return p["base_hz"] + (ch & 0xFFF) * p["spacing_hz"]


def parse_tsbk(data):
    """data = 14-byte UDP export: opcode, last-block flag, 12-byte TSBK."""
    if len(data) < 14:
        return None
    tsbk = int(data[2:14].hex()[:20], 16) << 16
    opcode = (tsbk >> 88) & 0x3F
    if (tsbk >> 80) & 0xFF != 0 or opcode not in OPCODES:     # standard (mfid 0) messages only
        return None
    cat, name = OPCODES[opcode]
    f = {"cat": cat, "name": name, "opcode": opcode}
    if cat == "IDEN":
        f["iden"], f["plan"] = parse_iden(tsbk, opcode)
        return f
    if opcode in (0x10,):
        f["ch"] = (tsbk >> 64) & 0xFFFF
        f["rid"] = (tsbk >> 40) & 0xFFFFFF
        f["opts"] = 1
    elif opcode == 0x11:
        f["opts"] = (tsbk >> 72) & 0xFF
        f["ch"] = (tsbk >> 56) & 0xFFFF
        f["rid"] = (tsbk >> 16) & 0xFFFFFF
    elif opcode == 0x14:
        f["opts"] = (tsbk >> 72) & 0xFF
        f["ch"] = (tsbk >> 56) & 0xFFFF
        f["rid"] = (tsbk >> 16) & 0xFFFFFF
    elif opcode == 0x15:
        f["rid"] = (tsbk >> 16) & 0xFFFFFF
    return f


def rid_wanted(rid, args):
    """--rid-range LO-HI (repeatable) wins; else --rid-prefix N (N*10000 .. N*10000+9999); else everyone."""
    if args.rid_range:
        return any(lo <= rid <= hi for lo, hi in args.rid_range)
    if args.rid_prefix is not None:
        return int(args.rid_prefix) * 10000 <= rid <= int(args.rid_prefix) * 10000 + 9999
    return True


def parse_range(txt):
    lo, hi = txt.split("-")
    return int(lo), int(hi)


# ------------------------------------------------------------- tuners ---
def _post_nowait(url, payload):
    """POST and close WITHOUT waiting for the reply. OP25's http_server queues the command before it
    ever answers, and the commands we send never produce a reply anyway; an unfixed server sits on the
    request for 200 ms, which used to block the controller (measured: 204 ms per tune on the Pi)."""
    import http.client
    from urllib.parse import urlparse
    u = urlparse(url)
    c = http.client.HTTPConnection(u.hostname, u.port or 80, timeout=1.5)
    try:
        c.putrequest("POST", u.path or "/")
        c.putheader("Content-Type", "application/json")
        c.putheader("Content-Length", str(len(payload)))
        c.endheaders(payload)
    finally:
        c.close()


class HttpTuner:
    def __init__(self, url):
        self.url = url

    def __call__(self, hz):
        body = json.dumps([{"command": "set_freq", "arg1": float(hz), "arg2": 0}]).encode()
        t0 = time.time()
        _post_nowait(self.url, body)
        return (time.time() - t0) * 1000.0


def post_cmd(url, command, arg1):
    """Fire one OP25 http_server.py command (set_freq, set_dwell_rid, ...)."""
    _post_nowait(url, json.dumps([{"command": command, "arg1": float(arg1), "arg2": 0}]).encode())


class UnixTuner:
    def __init__(self, path):
        self.path, self.sock = path, None

    def __call__(self, hz):
        t0 = time.time()
        for _ in range(2):
            try:
                if self.sock is None:
                    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    self.sock.settimeout(0.5)
                    self.sock.connect(self.path)
                self.sock.sendall((json.dumps({"command": "set_freq", "freq": float(hz)}) + "\n").encode())
                return (time.time() - t0) * 1000.0
            except OSError:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
        raise OSError("unix tune failed: %s" % self.path)


# ------------------------------------------------------------ chaser ---
class Chaser:
    """All policy lives here; time and tuning are injected so the same
    object runs live, in --replay, and in the unit tests."""

    def __init__(self, args, tune, log=print):
        self.a, self.tune, self.log = args, tune, log
        self.lock = threading.RLock()
        self.mode = "IDLE"
        self.cur_hz = None
        self.rid = self.ch = None
        self.t_grant = self.t_dwell = self.t_tuned = 0.0
        self.last_activity = 0.0
        self.had_ping = False
        self.first_ping = None
        self.release_after = None
        self.c = collections.Counter()
        self.lat_tune, self.lat_first, self.dwells = [], [], []
        self.missed = []
        self.dwell = None          # callable(rid) -> tells rx.py's decoder which RID this dwell is for
        self.iden = dict(ASSUME_PLAN) if getattr(args, "assume_afrrcs_plan", False) else {}
        self.warned_noplan = set()
        self.by_prefix = collections.Counter()      # grants seen, by rid // 10000 (before any filter)

    # -- helpers
    def _ts(self, now):
        return datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3]

    def _start(self, now, rid, ch, hz, why):
        ms = 0.0
        already = (hz == self.cur_hz)
        if hz != self.cur_hz:
            try:
                ms = self.tune(hz)
            except Exception as e:
                self.c["tune_fail"] += 1
                self.log("%s TUNE FAIL rid=%s ch=%s %d Hz: %s" % (self._ts(now), rid, ch, hz, e))
                return
            self.cur_hz = hz
            self.c["retunes"] += 1
            self.lat_tune.append(ms)
        else:
            self.c["same_channel"] += 1
        self._dwell(rid, now)                  # after the retune: the retune is the time-critical part
        self.mode, self.rid, self.ch = "CHASE", rid, ch
        self.t_grant = self.t_dwell = self.last_activity = now
        self.t_tuned = now + ms / 1000.0
        self.had_ping, self.first_ping, self.release_after = False, None, None
        self.c["chases"] += 1
        self.log("%s CHASE rid=%s ch=%s %d Hz %s (%.1fms)" % (
            self._ts(now), rid, ch, hz, why + (" [already there]" if already else ""), ms))

    def _dwell(self, rid, now):
        if self.dwell:
            try:
                self.dwell(rid)
            except Exception as e:
                self.log("%s set_dwell_rid(%s) failed: %s" % (self._ts(now), rid, e))

    def _release(self, now, reason):
        dwell = now - self.t_dwell
        self.dwells.append(dwell)
        self.c["rel_" + reason] += 1
        self.log("%s RELEASE rid=%s ch=%s after %.2fs [%s]%s" % (
            self._ts(now), self.rid, self.ch, dwell, reason,
            "" if self.had_ping else "  (nothing decoded)"))
        self.mode, self.rid, self.ch = "IDLE", None, None
        self._dwell(0, now)
        if self.a.home_hz and self.cur_hz != self.a.home_hz:
            try:
                self.tune(self.a.home_hz)
                self.cur_hz = self.a.home_hz
            except Exception as e:
                self.log("home tune failed: %s" % e)

    # -- inputs
    def on_tsbk(self, data, now=None):
        now = time.time() if now is None else now
        f = parse_tsbk(data)
        if f:
            self.on_msg(f, now)

    def on_msg(self, f, now):
        with self.lock:
            if f["cat"] == "IDEN":
                new = f["plan"]
                if self.iden.get(f["iden"]) != new:
                    self.iden[f["iden"]] = new
                    self.log("%s CHANNEL PLAN iden %d: base %.5f MHz spacing %.2f kHz tx offset %+.1f MHz" % (
                        self._ts(now), f["iden"], new["base_hz"] / 1e6, new["spacing_hz"] / 1e3, new["toff_hz"] / 1e6))
                self.c["iden_seen"] += 1
                return
            if f["cat"] == "DATA_PAGE":
                self.c["pages_seen"] += 1
                if getattr(self.a, "verbose", False):
                    self.log("%s page  %-18s rid=%s" % (self._ts(now), f["name"], f["rid"]))
                return
            self.c["grants_seen"] += 1
            rid, ch = f["rid"], f.get("ch")
            self.by_prefix[rid // 10000] += 1
            v = getattr(self.a, "verbose", False)

            def vlog(why):
                if v:
                    self.log("%s grant %-18s rid=%s ch=%s opts=%s -> %s" % (
                        self._ts(now), f["name"], rid, ch, f.get("opts"), why))
            if ch in INVALID_CH or ch is None:
                return vlog("ignored (invalid channel)")
            if not rid_wanted(rid, self.a):
                self.c["grants_other_prefix"] += 1
                return vlog("skipped (rid not in --rid-range/--rid-prefix)")
            self.c["grants_target"] += 1
            if f.get("opts", 1) == 0 and not self.a.follow_opts0:
                self.c["skipped_opts0"] += 1
                return vlog("skipped (opts=0; use --follow-opts0)")
            vlog("wanted")
            hz = channel_to_hz(ch, self.iden)
            if hz is None:
                self.c["grants_no_plan"] += 1
                if (ch >> 12) not in self.warned_noplan:
                    self.warned_noplan.add(ch >> 12)
                    self.log("%s grant on channel %#06x but identifier %d not announced yet (waiting for IDEN_UP)" % (
                        self._ts(now), ch, (ch >> 12) & 0xF))
                return
            if self.mode == "IDLE":
                self._start(now, rid, ch, hz, "(grant)")
                return
            # busy
            if rid == self.rid:
                self.last_activity = max(self.last_activity, now)
                if ch != self.ch:
                    self.c["same_rid_channel_switch"] += 1
                    self._start(now, rid, ch, hz, "(same rid moved)")
                else:
                    self.c["repeat_grants"] += 1
                return
            if hz == self.cur_hz:
                self.c["piggyback"] += 1       # other radio, same channel: already hearing it
                self.log("%s PIGGYBACK rid=%s ch=%s (already on it)" % (self._ts(now), rid, ch))
                return
            idle_for = now - self.last_activity
            if (not self.had_ping) and (now - self.t_dwell) >= self.a.preempt_after:
                self.c["preempted"] += 1
                self.log("%s PREEMPT rid=%s (nothing decoded for %.2fs)" % (self._ts(now), self.rid, now - self.t_dwell))
                self._release(now, "preempt")
                self._start(now, rid, ch, hz, "(preempt)")
                return
            self.c["missed_busy"] += 1
            self.missed.append((now, rid, ch, self.rid, self.ch))
            self.log("%s BUSY-MISS rid=%s ch=%s  (chasing rid=%s ch=%s, %.2fs in)" % (
                self._ts(now), rid, ch, self.rid, self.ch, now - self.t_dwell))

    def on_hold(self, rid, now=None):
        now = time.time() if now is None else now
        with self.lock:
            if self.mode != "CHASE" or rid != self.rid:
                return
            self.last_activity = now
            if not self.had_ping:
                self.had_ping = True
                self.first_ping = now
                lat = (now - self.t_grant) * 1000.0
                self.lat_first.append(lat)
                self.log("  first decode rid=%s  %.0f ms after grant (%.0f ms after tune done)" % (
                    rid, lat, (now - self.t_tuned) * 1000.0))

    def on_tms(self, rid, text, partial, now=None):
        now = time.time() if now is None else now
        with self.lock:
            if self.mode != "CHASE" or rid != self.rid:
                return
            self.last_activity = now          # any TMS export is decode activity
            if partial or len(text.strip()) < self.a.routine_text_max:
                return
            self.c["texts_seen"] += 1
            self.release_after = now + self.a.ack_grace
            self.log("  TEXT complete rid=%s len=%d -> release in %.1fs" % (rid, len(text.strip()), self.a.ack_grace))

    def tick(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            if self.mode != "CHASE":
                return
            dwell = now - self.t_dwell
            if self.release_after is not None and now >= self.release_after:
                return self._release(now, "text-complete")
            if dwell >= self.a.max_dwell:
                return self._release(now, "max-dwell")
            if dwell >= self.a.min_dwell:
                quiet = now - self.last_activity
                thr = self.a.routine_quiet if self.had_ping else self.a.release_quiet
                if quiet >= thr:
                    return self._release(now, "routine-quiet" if self.had_ping else "quiet")

    def summary(self):
        c = self.c

        def stat(x, unit):
            if not x:
                return "n/a"
            xs = sorted(x)
            return "n=%d med=%.0f%s p90=%.0f%s max=%.0f%s" % (
                len(xs), statistics.median(xs), unit, xs[int(0.9 * (len(xs) - 1))], unit, xs[-1], unit)
        t = c["chases"] + c["missed_busy"] + c["piggyback"]
        out = ["SUMMARY grants_seen=%d target=%d other_rid=%d skipped_opts0=%d no_plan=%d pages=%d iden_msgs=%d" % (
                   c["grants_seen"], c["grants_target"], c["grants_other_prefix"],
                   c["skipped_opts0"], c["grants_no_plan"], c["pages_seen"], c["iden_seen"]),
               "        grants by rid range: " + (", ".join("%dxxxx=%d" % (p, n) for p, n in self.by_prefix.most_common(8)) or "none yet"),
               "        chases=%d (retunes=%d same_channel=%d preempt=%d switched=%d) piggyback=%d "
               "missed_busy=%d tune_fail=%d texts=%d" % (
                   c["chases"], c["retunes"], c["same_channel"], c["preempted"], c["same_rid_channel_switch"],
                   c["piggyback"], c["missed_busy"], c["tune_fail"], c["texts_seen"]),
               "        releases: " + ", ".join("%s=%d" % (k[4:], v) for k, v in sorted(c.items()) if k.startswith("rel_")),
               "        tune latency: " + stat(self.lat_tune, "ms"),
               "        grant->first decode: " + stat(self.lat_first, "ms"),
               "        dwell: " + stat([d * 1000 for d in self.dwells], "ms")]
        if t:
            out.append("        covered %.1f%% of target bursts (chased+piggyback), %.1f%% hit while busy" % (
                100.0 * (c["chases"] + c["piggyback"]) / t, 100.0 * c["missed_busy"] / t))
        return "\n".join(out)


# --------------------------------------------------------------- live ---
def live(args, ch):
    def listener(port, handler, size):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((args.bind, port))
        while True:
            raw, _ = s.recvfrom(size)
            try:
                handler(raw)
            except Exception:
                pass

    threading.Thread(target=listener, args=(51001, lambda r: ch.on_hold(int(json.loads(r.decode())["rid"])), 256), daemon=True).start()

    def tms(r):
        m = json.loads(r.decode())
        # p25p1_fdma's tms_export sends the text under "tms_text" (older builds: "text")
        ch.on_tms(int(m["rid"]), str(m.get("tms_text", m.get("text", ""))), bool(m.get("partial", False)))
    threading.Thread(target=listener, args=(51003, tms, 4096), daemon=True).start()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((args.bind, args.tsbk_port))
    s.settimeout(0.05)
    last_sum = time.time()
    try:
        while True:
            try:
                data, _ = s.recvfrom(64)
                ch.on_tsbk(data)
            except socket.timeout:
                pass
            ch.tick()
            sys.stdout.flush()
            if time.time() - last_sum > args.summary_every:
                print(ch.summary())
                last_sum = time.time()
    except KeyboardInterrupt:
        print("\n" + ch.summary())


# ------------------------------------------------------------- replay ---
GRANT_RE = re.compile(r'^(\d\d):(\d\d):(\d\d\.\d+) (DATA_GRANT)\s+(\S+)\s+target_rid=\[(\d+)[^\]]*\] '
                      r'ch_t=(\d+) ch_r=\d+ opts=(\d+)')


CCLOG_RE = re.compile(r'^\d{4}-\d\d-\d\dT(\d\d):(\d\d):(\d\d\.\d+)[-+]\d\d:\d\d 0x[0-9a-fA-F]{2} \S+ .*hex=([0-9A-Fa-f]{24})')


def replay(args, ch_factory):
    events = []
    for line in open(args.replay, errors="ignore"):
        m = CCLOG_RE.match(line)          # raw control-channel TSBK log (cc_tsbk_logger.py): includes IDEN_UP
        if m:
            h, mi, sec, hx = m.groups()
            f = parse_tsbk(b"\x00\x00" + bytes.fromhex(hx))
            if f:
                events.append((int(h) * 3600 + int(mi) * 60 + float(sec), f))
            continue
        m = GRANT_RE.match(line)
        if m:
            h, mi, s, _, name, rid, c, opts = m.groups()
            events.append((int(h) * 3600 + int(mi) * 60 + float(s),
                           {"cat": "DATA_GRANT", "name": name, "rid": int(rid), "ch": int(c), "opts": int(opts)}))
    if not events:
        sys.exit("no DATA_GRANT / raw TSBK lines found in %s" % args.replay)
    events.sort(key=lambda e: e[0])
    clock = [events[0][0]]
    quiet_log = []
    ch = ch_factory(lambda *_: None, quiet_log.append if not args.verbose else print)
    # swap tune for a latency-modelling no-op
    ch.tune = lambda hz: args.replay_tune_ms
    pings = []      # heap of (t, rid)
    step = 0.05

    def advance(to):
        while clock[0] + step <= to:
            clock[0] += step
            while pings and pings[0][0] <= clock[0]:
                t, rid = heapq.heappop(pings)
                ch.on_hold(rid, t)
            ch.tick(clock[0])
        clock[0] = to
    for t, f in events:
        advance(t)
        before = ch.c["chases"]
        ch.on_msg(f, t)
        if args.replay_ping_ms >= 0 and ch.c["chases"] > before and ch.mode == "CHASE":
            for k in range(3):    # model: decode activity starts at +X ms, then keeps pinging briefly
                heapq.heappush(pings, (t + (args.replay_tune_ms + args.replay_ping_ms) / 1000.0 + 0.3 * k, ch.rid))
    advance(events[-1][0] + 30)
    print("replay %s: %d messages over %.0f min  (model: tune %.0f ms, first decode +%d ms after tune)"
          % (args.replay, len(events), (events[-1][0] - events[0][0]) / 60.0, args.replay_tune_ms, args.replay_ping_ms))
    print(ch.summary())
    return ch


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--tune-http", help="OP25 chaser instance URL, e.g. http://127.0.0.1:8086/")
    g.add_argument("--tune-unix", help="sdr_feed.py control socket path")
    g.add_argument("--dry-run", action="store_true")
    ap.add_argument("--rid-prefix", type=int, default=None, help="only chase RIDs N0000-N9999 (default: everyone)")
    ap.add_argument("--rid-range", type=parse_range, action="append", default=[], metavar="LO-HI",
                    help="only chase RIDs in this inclusive range (repeatable)")
    ap.add_argument("--assume-afrrcs-plan", action="store_true",
                    help="use the built-in AFRRCS 700 MHz channel plan until the tower's IDEN_UP is heard")
    ap.add_argument("--follow-opts0", action="store_true")
    ap.add_argument("--dwell-url", help="rx.py HTTP URL for set_dwell_rid (default: the --tune-http URL; "
                                        "needed with --tune-unix). Tells the decoder which RID each dwell is for.")
    ap.add_argument("--no-dwell-rid", action="store_true")
    ap.add_argument("--home-hz", type=int, default=0, help="retune here when idle (default: stay put)")
    ap.add_argument("--min-dwell", type=float, default=1.0)
    ap.add_argument("--release-quiet", type=float, default=2.0, help="quiet before release if NOTHING decoded")
    ap.add_argument("--routine-quiet", type=float, default=1.0, help="quiet before release once something decoded")
    ap.add_argument("--max-dwell", type=float, default=12.0)
    ap.add_argument("--ack-grace", type=float, default=1.0)
    ap.add_argument("--preempt-after", type=float, default=1.5,
                    help="a new grant for another radio may preempt a chase that has decoded nothing for this long")
    ap.add_argument("--routine-text-max", type=int, default=8)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--tsbk-port", type=int, default=51002)
    ap.add_argument("--summary-every", type=float, default=300)
    ap.add_argument("--replay", help="drive the state machine from an old Uniden-monitor grant log")
    ap.add_argument("--replay-tune-ms", type=float, default=15.0)
    ap.add_argument("--replay-ping-ms", type=int, default=400,
                    help="model: first decode this long after tune done (-1 = never decodes)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    def make(tune, log):
        return Chaser(args, tune, log)

    if args.replay:
        return replay(args, make)
    if args.tune_http:
        tune = HttpTuner(args.tune_http)
    elif args.tune_unix:
        tune = UnixTuner(args.tune_unix)
    elif args.dry_run:
        tune = lambda hz: 0.0
    else:
        ap.error("need --tune-http, --tune-unix, --dry-run or --replay")
    dwell_url = None if args.no_dwell_rid else (args.dwell_url or args.tune_http)
    chaser = make(tune, print)
    if dwell_url:
        chaser.dwell = lambda rid, u=dwell_url: post_cmd(u, "set_dwell_rid", rid)
    print("dual_chaser: TSBK udp/%d  min_dwell=%.1f routine_quiet=%.1f release_quiet=%.1f max=%.0f" % (
        args.tsbk_port, args.min_dwell, args.routine_quiet, args.release_quiet, args.max_dwell))
    live(args, chaser)


if __name__ == "__main__":
    main()
