#!/usr/bin/env python3
"""
multi_chaser.py -- two-SDR "wait and chase" controller for P25 Phase 1 data grants,
with one or MORE control channels watched by the first SDR.

    SDR #1  (OP25 multi_rx.py, one channel per control channel): parked forever,
            exports every TSBK to UDP 127.0.0.1:51002 (already done by
            p25p1_fdma.cc's tsbk_export).  Never retuned.  All the control
            channels must fit inside that one SDR's sample-rate window.
    SDR #2  (OP25 rx.py instance): the chaser.  This script retunes it on
            each data grant for a radio in the target RID range, holds while OP25 reports
            decoded activity (UDP 51001 HOLD, 51003 TMS text), then lets
            it sit where it is (or goes --home-hz).

The grant's channel number is turned into Hz with the IDEN_UP table from the
control channels, so a grant from ANY watched site retunes SDR #2 to the right
frequency.  The sites are assumed to announce the same channel plan (the
TSBK export carries no source tag).

With --config FILE the whole setup (both SDRs, the control-channel list, RID ranges, gains, ppm, timings) comes from one
JSON file: this script writes the multi_rx.py config and builds the rx.py command line itself.  --check-config prints
what it would run and exits.  See multi_chaser.example.json.

Optionally this script also launches and babysits both OP25 processes
(--cc-cmd / --chase-cmd): it starts them, logs their output to size-capped
files, restarts either one if it dies, and stops both on Ctrl-C / SIGTERM.


Tune back-ends (pick one):
  --tune-http URL   OP25 http_server.py POST set_freq   (the fixed ~1-3 ms one,
                    NOT the old 200 ms wait-for-reply version)
  --tune-unix PATH  sdr_feed.py control socket  {"command":"set_freq","freq":hz}\\n
  --dry-run         print only
"""
import argparse
import collections
import copy
import heapq
import json
import os
import re
import shlex
import signal
import socket
import statistics
import subprocess
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
# Fallback plan, used ONLY with --assume-700mhz-plan (a 700 MHz plan: base 769.08125 MHz, 12.5 kHz spacing, +30 MHz transmit offset).
# Normally the plan comes from the tower itself (IDEN_UP / IDEN_UP_VU, see Chaser.iden).
ASSUME_PLAN = {0: {"base_hz": 769081250, "spacing_hz": 12500, "toff_hz": 30000000}}


def parse_iden(tsbk, opcode):
    """TIA-102.AABC-B 6.2.9 (IDEN_UP, 0x3D) / 6.2.29 (IDEN_UP_VU, 0x34).
    `tsbk` is the 96-bit value (10 octets + 16 CRC bits zero).  Verified against a live capture from a 700 MHz
    system: BD000325E064092B0CBA -> iden 0, base 769.08125 MHz, spacing 12.5 kHz, tx +30 MHz."""
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
    request for 200 ms, which used to block the controller"""
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
        self.iden = dict(ASSUME_PLAN) if getattr(args, "assume_700mhz_plan", False) else {}
        self.warned_noplan = set()
        self.by_prefix = collections.Counter()      # grants seen, by rid // 10000 (before any filter)
        self.iden_heard = set()                     # identifiers announced by a tower (IDEN_UP / IDEN_UP_VU)
        self.iden_used = collections.Counter()      # grants seen, by channel identifier (before any filter)

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
                new, i = f["plan"], f["iden"]
                old = self.iden.get(i)
                if i not in self.iden_heard or old != new:
                    note = ("heard from tower" if old is None else
                            "matches the built-in plan" if old == new else
                            ("REPLACES the built-in plan" if i not in self.iden_heard else "CHANGED"))
                    self.log("%s CHANNEL PLAN iden %d: base %.5f MHz spacing %.2f kHz tx offset %+.1f MHz  [%s]" % (
                        self._ts(now), i, new["base_hz"] / 1e6, new["spacing_hz"] / 1e3, new["toff_hz"] / 1e6, note))
                self.iden_heard.add(i)
                self.iden[i] = new
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
            idn = (ch >> 12) & 0xF
            if self.iden_used[idn] == 0:             # first grant on this identifier: show the plan really is being used
                hz0 = channel_to_hz(ch, self.iden)
                self.log("%s first data grant uses iden %d (channel %d -> %s)" % (
                    self._ts(now), idn, ch & 0xFFF, "%.5f MHz" % (hz0 / 1e6) if hz0 else "plan not heard yet"))
            self.iden_used[idn] += 1
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

    def _save_text(self, now, rid, text, partial, port=None):
        """Print every TMS text the decoder reports and append it to --text-log
        (one JSON object per line), whether or not a chase is active."""
        t = text.strip()
        if not t:
            return
        key = (rid, t, bool(partial))
        if getattr(self, "_last_text", None) and self._last_text[0] == key and now - self._last_text[1] < 60:
            return                                   # same exact text re-reported (retries)
        self._last_text = (key, now)
        self.c["texts_logged"] += 1
        ch = self.ch if (self.mode == "CHASE" and rid == self.rid) else None
        self.log("TEXT rid=%s%s%s%s: %s" % (rid, " ch=%s" % ch if ch else "", " port=%s" % port if port else "",
                                            " [partial]" if partial else "", t))
        path = getattr(self.a, "text_log", None)
        if path:
            try:
                with open(path, "a", encoding="utf-8") as f:
                    rec = {"ts": datetime.fromtimestamp(now).astimezone().isoformat(timespec="milliseconds"),
                           "rid": rid, "ch": ch, "port": port, "partial": bool(partial), "text": t}
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            except OSError as e:
                self.log("text-log write failed: %s" % e)

    def on_tms(self, rid, text, partial, now=None, port=None):
        now = time.time() if now is None else now
        with self.lock:
            self._save_text(now, rid, text, partial, port)
            if self.mode != "CHASE" or rid != self.rid:
                return
            self.last_activity = now          # any TMS export is decode activity
            if not self.had_ping:             # a text counts as the first decode (the 51001 ping may not be sent by this build)
                self.had_ping, self.first_ping = True, now
                lat = (now - self.t_grant) * 1000.0
                self.lat_first.append(lat)
                self.log("  first decode (text) rid=%s  %.0f ms after grant (%.0f ms after tune done)" % (
                    rid, lat, (now - self.t_tuned) * 1000.0))
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
        out.append("        channel plans: heard iden %s | grants use %s" % (
            ",".join(str(i) for i in sorted(self.iden_heard)) or "none",
            ", ".join("iden %d (%d)" % (i, n) for i, n in sorted(self.iden_used.items())) or "none yet"))
        if getattr(self, "restarts", None):
            out.append("        child restarts: " + ", ".join("%s=%d" % kv for kv in sorted(self.restarts().items())))
        if t:
            out.append("        covered %.1f%% of target bursts (chased+piggyback), %.1f%% hit while busy" % (
                100.0 * (c["chases"] + c["piggyback"]) / t, 100.0 * c["missed_busy"] / t))
        return "\n".join(out)


# ------------------------------------------------------------- config ---
class ConfigError(Exception):
    pass


DEFAULT_DEVICE = {"type": "rtl", "serial": None, "index": None, "ppm": 0, "gains": "", "buflen": 16384, "args": None}
CFG_DEFAULTS = {
    "control_channels": {
        "device": DEFAULT_DEVICE,
        "frequencies": None,                 # required: list of Hz (or MHz) numbers, or {"name":..., "frequency":...}
        "sample_rate": "auto",               # "auto" (RTL only) or an explicit rate in Hz
        "center_frequency": "auto",
        "usable_bw_pct": 0.9,
        "http_port": 8085,
        "verbosity": 0,
        "demod": {"demod_type": "cqpsk", "filter_type": "rrc", "excess_bw": 0.2, "if_rate": 24000,
                  "symbol_rate": 4800, "gain_mu": 0.04, "costas_alpha": 0.02},
    },
    "chaser": {
        "device": DEFAULT_DEVICE,
        "start_frequency": "auto",           # where the chaser SDR waits until the first grant; "auto" = average of the control channels
        "sample_rate": 1000000,
        "costas_alpha": 0.02,
        "gain_mu": 0.04,
        "http_port": 8086,
        "verbosity": 0,
        "extra_args": "",                    # appended to the rx.py command line as-is
    },
    "policy": {
        "rid_ranges": [],                    # ["LO-HI", ...]; empty = chase everyone
        "follow_opts0": False,
        "min_dwell": 1.0, "release_quiet": 2.0, "routine_quiet": 1.0, "max_dwell": 12.0,
        "ack_grace": 1.0, "preempt_after": 1.5, "routine_text_max": 8,
        "home_frequency": 0,                 # retune here when idle (0 = stay put)
        "summary_every": 300,
    },
    "channel_plan": {"assume": "700mhz"},    # fallback plan until the tower's IDEN_UP is heard: "700mhz" or "none"
    "paths": {"op25_dir": ".", "log_dir": "logs", "text_log": "chaser_texts.jsonl", "python": "python3"},
    "process": {"stagger": 3.0, "child_log_max_mb": 50.0, "activity_ping": True},
}
# Sample rates tried (smallest first) when control_channels.sample_rate is "auto" on an RTL-SDR.
NICE_RATES = [1000000, 1024000, 1200000, 1250000, 1536000, 1600000, 1800000, 2000000, 2048000, 2400000]
EDGE_GUARD_HZ = 25000        # keep each channel this far inside the usable window
DC_GUARD_HZ = 25000          # keep each channel this far from the SDR's centre (DC) spike
DC_SHIFT_HZ = 150000         # move the centre this far when a channel would sit on the spike


def _merge(defaults, user, path=""):
    """User values over defaults; an unknown key is an error (it's a typo), keys starting with _ are comments."""
    if not isinstance(user, dict):
        raise ConfigError("%s must be an object" % (path.rstrip(".") or "config"))
    for k in user:
        if not k.startswith("_") and k not in defaults:
            raise ConfigError("unknown setting '%s%s' (allowed here: %s)" % (path, k, ", ".join(sorted(defaults))))
    out = {}
    for k, dv in defaults.items():
        if k in user:
            out[k] = _merge(dv, user[k], path + k + ".") if isinstance(dv, dict) else user[k]
        else:
            out[k] = copy.deepcopy(dv)
    return out


def load_config(path):
    try:
        with open(path, encoding="utf-8-sig") as f:
            raw = json.load(f)
    except OSError as e:
        raise ConfigError("cannot read %s: %s" % (path, e))
    except ValueError as e:
        raise ConfigError("%s is not valid JSON: %s" % (path, e))
    cfg = _merge(CFG_DEFAULTS, raw)
    if not cfg["control_channels"]["frequencies"]:
        raise ConfigError("control_channels.frequencies is required (list the control channel frequencies)")
    for sect in ("control_channels", "chaser"):
        d = cfg[sect]["device"]
        if d["args"] is None and d["serial"] is None and d["index"] is None:
            raise ConfigError("%s.device needs a 'serial' (preferred) or an 'index' (or raw 'args')" % sect)
        if d["serial"] is not None and len(str(d["serial"])) < 2:
            raise ConfigError("%s.device.serial '%s' is too short: a 1-character value is read as a device index" % (sect, d["serial"]))
    a, b = cfg["control_channels"]["device"], cfg["chaser"]["device"]
    if a["args"] is None and b["args"] is None and (a["serial"], a["index"]) == (b["serial"], b["index"]):
        raise ConfigError("control_channels.device and chaser.device point at the same SDR")
    if cfg["channel_plan"]["assume"] not in ("700mhz", "none"):
        raise ConfigError("channel_plan.assume must be \"700mhz\" or \"none\"")
    return cfg


def _hz(v, what):
    """Accept Hz (771093750) or MHz (771.09375)."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ConfigError("%s: '%s' is not a number" % (what, v))
    return int(round(v * 1e6)) if v < 100000 else int(round(v))


def parse_cc_list(items):
    out, seen = [], set()
    for it in items:
        name = None
        if isinstance(it, dict):
            name, it = it.get("name"), it.get("frequency")
        hz = _hz(it, "control_channels.frequencies")
        if hz in seen:
            raise ConfigError("control channel %d Hz listed twice" % hz)
        seen.add(hz)
        out.append((name or "CC_%d" % (hz // 1000), hz))
    return sorted(out, key=lambda x: x[1])


def parse_rid_ranges(items):
    out = []
    for it in items:
        try:
            lo, hi = (it if isinstance(it, (list, tuple)) else str(it).split("-"))
            lo, hi = int(lo), int(hi)
        except (TypeError, ValueError):
            raise ConfigError("policy.rid_ranges: '%s' is not LO-HI" % (it,))
        if lo > hi:
            raise ConfigError("policy.rid_ranges: %s-%s has LO greater than HI" % (lo, hi))
        out.append((lo, hi))
    return out


def device_args(dev):
    if dev["args"]:
        return dev["args"]
    if dev["type"] != "rtl":
        raise ConfigError("device.type '%s': give the raw osmosdr string in device.args" % dev["type"])
    ident = dev["serial"] if dev["serial"] is not None else dev["index"]
    return "rtl=%s,buflen=%d" % (ident, int(dev["buflen"]))


def plan_window(freqs, rate, pct, center="auto"):
    """Pick the SDR centre frequency so every control channel is inside the usable window and none sits on the
    DC spike.  Returns (centre_hz, half_window_hz).  Raises ConfigError if they don't fit at this rate."""
    lo, hi = min(freqs), max(freqs)
    half = rate * pct / 2.0 - EDGE_GUARD_HZ

    def ok(c):
        return all(abs(f - c) <= half for f in freqs)

    def on_dc(c):
        return any(abs(f - c) < DC_GUARD_HZ for f in freqs)
    if center != "auto":
        c = _hz(center, "control_channels.center_frequency")
        if not ok(c):
            raise ConfigError("control channels don't fit around centre %d Hz at %d Hz sample rate (usable ±%.0f kHz)" % (
                c, rate, half / 1e3))
        return c, half
    c0 = int(round((lo + hi) / 2.0 / 1000.0)) * 1000
    for c in ([c0] if not on_dc(c0) else [c0 + DC_SHIFT_HZ, c0 - DC_SHIFT_HZ]):
        if ok(c) and not on_dc(c):
            return c, half
    raise ConfigError("control channels (%.3f MHz wide) don't fit at %d Hz sample rate (usable ±%.0f kHz)" % (
        (hi - lo) / 1e6, rate, half / 1e3))


def choose_rate(freqs, cc):
    rate = cc["sample_rate"]
    if rate != "auto":
        return int(rate)
    if cc["device"]["type"] != "rtl":
        raise ConfigError("control_channels.sample_rate must be given explicitly (auto is RTL-SDR only)")
    for r in NICE_RATES:
        try:
            plan_window(freqs, r, cc["usable_bw_pct"], cc["center_frequency"])
            return r
        except ConfigError:
            continue
    span = (max(freqs) - min(freqs)) / 1e6
    raise ConfigError("the control channels span %.3f MHz, which is too wide for one RTL-SDR (max %.1f MS/s). "
                      "Use fewer sites, or an SDR with a wider window and set sample_rate explicitly" % (
                          span, NICE_RATES[-1] / 1e6))


def choose_park(freqs, start, rate, pct=0.9):
    """Where the chaser SDR waits.  "auto" = the average of the control channel frequencies, nudged away by whole
    sample-rate steps if that would put a control channel inside the chaser's own window (it would then decode the
    control channel itself and send every TSBK twice).  Returns (hz, note, warning_or_None)."""
    half = rate * pct / 2.0

    def clear(f):
        return all(abs(f - c) > half for c in freqs)
    if start != "auto":
        hz = _hz(start, "chaser.start_frequency")
        return hz, "from the config", (None if clear(hz) else
            "chaser.start_frequency %.5f MHz is inside the chaser's window of a control channel: it would decode that "
            "control channel itself and double every TSBK" % (hz / 1e6))
    mean = int(round(sum(freqs) / len(freqs) / 1000.0)) * 1000
    for delta in (0, 1, -1, 2, -2):
        cand = mean + delta * int(rate)
        if clear(cand):
            return cand, "average of the control channels" + ("" if delta == 0 else " (moved %+d MHz to clear a control channel)" % (delta * rate / 1e6)), None
    return mean, "average of the control channels", "could not find a parking frequency clear of every control channel"


def build_from_config(cfg):
    """Everything the launcher needs, computed from the one config.  Pure function: writes nothing."""
    cc, ch, pol, paths = cfg["control_channels"], cfg["chaser"], cfg["policy"], cfg["paths"]
    sites = parse_cc_list(cc["frequencies"])
    freqs = [hz for _, hz in sites]
    rate = choose_rate(freqs, cc)
    pct = float(cc["usable_bw_pct"])
    centre, half = plan_window(freqs, rate, pct, cc["center_frequency"])
    d = cc["demod"]
    ppm = cc["device"]["ppm"]
    cc_json = {
        "devices": [{"name": "sdr_cc", "args": device_args(cc["device"]), "rate": rate, "frequency": centre, "offset": 0,
                     "ppm": int(ppm) if float(ppm) == int(ppm) else ppm, "gain_mode": False,
                     "gains": cc["device"]["gains"], "tunable": False, "usable_bw_pct": pct}],
        "channels": [dict({"name": n, "device": "sdr_cc", "frequency": hz}, **{
            "demod_type": d["demod_type"], "filter_type": d["filter_type"], "excess_bw": d["excess_bw"],
            "if_rate": d["if_rate"], "symbol_rate": d["symbol_rate"], "enable_analog": "off",
            "destination": "0.0.0.0:0", "gain_mu": d["gain_mu"], "costas_alpha": d["costas_alpha"]}) for n, hz in sites],
        "terminal": {"module": "terminal.py", "terminal_type": "http:0.0.0.0:%d" % cc["http_port"]},
    }
    gen_dir = os.path.join(paths["log_dir"], "generated")
    cc_json_path = os.path.abspath(os.path.join(gen_dir, "cc_multi_rx.json"))
    cc_cmd = shlex.join([paths["python"], "./multi_rx.py", "-c", cc_json_path, "-v", str(cc["verbosity"])])
    cd = ch["device"]
    park, park_note, park_warn = choose_park(freqs, ch["start_frequency"], int(ch["sample_rate"]))
    chase = [paths["python"], "./rx.py", "--args", device_args(cd)]
    if cd["gains"]:
        chase += ["-N", cd["gains"]]
    chase += ["-C", "%g" % ch["costas_alpha"], "-G", "%g" % ch["gain_mu"], "-S", str(int(ch["sample_rate"])),
              "-f", str(park),
              "-l", "http:0.0.0.0:%d" % ch["http_port"], "-v", str(ch["verbosity"]), "-q", "%g" % cd["ppm"]]
    chase_cmd = shlex.join(chase) + ((" " + ch["extra_args"]) if ch["extra_args"] else "")
    return {"sites": sites, "rate": rate, "centre": centre, "half_window": half, "cc_json": cc_json,
            "cc_json_path": cc_json_path, "cc_cmd": cc_cmd, "chase_cmd": chase_cmd,
            "rid_ranges": parse_rid_ranges(pol["rid_ranges"]), "park": park, "park_note": park_note,
            "warnings": [w for w in (park_warn,) if w]}


def describe(built, cfg):
    sites = built["sites"]
    lines = ["control channels (%d) on one SDR: %s" % (len(sites), ", ".join("%s %.5f MHz" % (n, hz / 1e6) for n, hz in sites)),
             "  sample rate %.3f MS/s, centre %.5f MHz, usable window ±%.0f kHz, widest channel ±%.0f kHz from centre" % (
                 built["rate"] / 1e6, built["centre"] / 1e6, built["half_window"] / 1e3,
                 max(abs(hz - built["centre"]) for _, hz in sites) / 1e3),
             "  generated multi_rx config: %s" % built["cc_json_path"],
             "  cc    : %s" % built["cc_cmd"],
             "  chase : %s" % built["chase_cmd"],
             "  park  : chaser SDR waits at %.5f MHz (%s)" % (built["park"] / 1e6, built["park_note"]),
             "  RIDs  : %s" % (", ".join("%d-%d" % r for r in built["rid_ranges"]) or "everyone"),
             "  plan  : %s until the tower announces its own" % ("built-in 700 MHz plan" if cfg["channel_plan"]["assume"] == "700mhz" else "no fallback")]
    lines += ["  WARNING: " + w for w in built["warnings"]]
    return "\n".join(lines)


def apply_config(args, ap, cfg, built):
    """Config fills in whatever the command line left at its default; an explicit command-line flag always wins."""
    def setdef(name, value):
        if getattr(args, name) == ap.get_default(name):
            setattr(args, name, value)
    pol, paths, proc = cfg["policy"], cfg["paths"], cfg["process"]
    setdef("cc_cmd", built["cc_cmd"])
    setdef("chase_cmd", built["chase_cmd"])
    setdef("cwd", paths["op25_dir"])
    setdef("log_dir", paths["log_dir"])
    setdef("text_log", paths["text_log"])
    for k in ("min_dwell", "release_quiet", "routine_quiet", "max_dwell", "ack_grace", "preempt_after",
              "routine_text_max", "summary_every", "follow_opts0"):
        setdef(k, pol[k])
    setdef("home_hz", _hz(pol["home_frequency"], "policy.home_frequency") if pol["home_frequency"] else 0)
    setdef("stagger", proc["stagger"])
    setdef("child_log_max_mb", proc["child_log_max_mb"])
    if not args.rid_range and args.rid_prefix is None:
        args.rid_range = built["rid_ranges"]
    if cfg["channel_plan"]["assume"] == "700mhz":
        args.assume_700mhz_plan = True
    if not proc["activity_ping"]:
        args.no_activity_ping = True


def write_generated(built):
    os.makedirs(os.path.dirname(built["cc_json_path"]), exist_ok=True)
    with open(built["cc_json_path"], "w") as f:
        json.dump(built["cc_json"], f, indent=2)
        f.write("\n")


# --------------------------------------------------------- supervisor ---
def http_from_cmd(cmd):
    """Pull the control URL out of an rx.py command line (-l http:HOST:PORT) so --tune-http
    doesn't have to be given twice.  0.0.0.0 becomes 127.0.0.1.  None if there isn't one."""
    m = re.search(r'(?:^|\s)(?:-l|--terminal)\s+http:([^:\s]*):(\d+)', cmd)
    if not m:
        return None
    host = m.group(1)
    if host in ("", "0.0.0.0"):
        host = "127.0.0.1"
    return "http://%s:%s/" % (host, m.group(2))


class Child:
    """One OP25 process: own session (so the whole group can be stopped), stdout+stderr drained by a
    thread into a size-capped log file (the pipe is always drained, so the child can never block on it)."""

    def __init__(self, name, cmd, cwd, logpath, env, max_bytes, log):
        self.name, self.cmd, self.cwd, self.logpath = name, cmd, cwd, logpath
        self.env, self.max_bytes, self.log = env, max_bytes, log
        self.proc = None
        self.t_start = 0.0
        self.restarts = 0
        self.next_try = 0.0
        self.backoff = 2.0

    def start(self):
        self.proc = subprocess.Popen(shlex.split(self.cmd), cwd=self.cwd, env=self.env,
                                     stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        self.t_start = time.time()
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True).start()
        self.log("%s started pid=%d: %s" % (self.name, self.proc.pid, self.cmd))

    def _pump(self, proc):
        f = size = None
        try:
            f = open(self.logpath, "ab")
            size = f.tell()
            for line in iter(proc.stdout.readline, b""):
                f.write(line)
                size += len(line)
                if size >= self.max_bytes:                       # keep one backup, then start over
                    f.close()
                    os.replace(self.logpath, self.logpath + ".1")
                    f = open(self.logpath, "ab")
                    size = 0
                f.flush()
        except (OSError, ValueError):
            for _ in iter(proc.stdout.readline, b""):          # logging failed: still drain
                pass
        finally:
            if f:
                f.close()

    def tail(self, n=3):
        try:
            with open(self.logpath, "rb") as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 2000))
                lines = f.read().decode("utf-8", "replace").splitlines()
        except OSError:
            return []
        return [l for l in lines if l.strip()][-n:]

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self, grace=5.0):
        if not self.alive():
            return
        for sig, wait in ((signal.SIGTERM, grace), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            t_end = time.time() + wait
            while time.time() < t_end:
                if self.proc.poll() is not None:
                    return
                time.sleep(0.05)


class Supervisor:
    """Starts the children in order (control-channel SDR first), restarts any that die, with backoff."""

    def __init__(self, children, log, on_restart=None, stagger=3.0):
        self.children, self.log, self.on_restart, self.stagger = children, log, on_restart, stagger
        self._last_check = 0.0

    def start(self):
        for i, c in enumerate(self.children):
            c.start()
            if i < len(self.children) - 1:
                time.sleep(self.stagger)       # let the first SDR claim its USB device before the next one starts

    def poll(self, now=None):
        """Cheap; call from the main loop.  Checks once a second."""
        now = time.time() if now is None else now
        if now - self._last_check < 1.0:
            return
        self._last_check = now
        for c in self.children:
            if c.alive() or now < c.next_try:
                continue
            code = c.proc.poll() if c.proc else None
            up = now - c.t_start
            c.backoff = 2.0 if up > 30.0 else min(c.backoff * 2.0, 60.0)
            c.next_try = now + c.backoff
            c.restarts += 1
            self.log("%s exited (code %s) after %.0fs -- restart #%d" % (c.name, code, up, c.restarts))
            for ln in c.tail(3):                       # the reason is usually the last line of its log
                self.log("  %s.log: %s" % (c.name, ln))
            try:
                c.start()
            except OSError as e:
                self.log("%s restart failed: %s" % (c.name, e))
                continue
            if self.on_restart:
                self.on_restart(c.name)

    def stop(self):
        for c in reversed(self.children):
            c.stop()


# --------------------------------------------------------------- live ---
def live(args, ch, periodic=None):
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
        ch.on_tms(int(m["rid"]), str(m.get("tms_text", m.get("text", ""))), bool(m.get("partial", False)),
                  port=m.get("port"))
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
            if periodic:
                periodic()
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


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--tune-http", help="OP25 chaser instance URL, e.g. http://127.0.0.1:8086/")
    g.add_argument("--tune-unix", help="sdr_feed.py control socket path")
    g.add_argument("--dry-run", action="store_true")
    ap.add_argument("--rid-prefix", type=int, default=None, help="only chase RIDs N0000-N9999 (default: everyone)")
    ap.add_argument("--rid-range", type=parse_range, action="append", default=[], metavar="LO-HI",
                    help="only chase RIDs in this inclusive range (repeatable)")
    ap.add_argument("--assume-700mhz-plan", action="store_true",
                    help="use the built-in 700 MHz channel plan until the tower's IDEN_UP is heard")
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
    ap.add_argument("--text-log", default="chaser_texts.jsonl", help="append every decoded TMS text here as JSON lines (empty string disables)")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--tsbk-port", type=int, default=51002)
    ap.add_argument("--summary-every", type=float, default=300)
    ap.add_argument("--replay", help="Depreciated")
    ap.add_argument("--replay-tune-ms", type=float, default=15.0)
    ap.add_argument("--replay-ping-ms", type=int, default=400,
                    help="model: first decode this long after tune done (-1 = never decodes)")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--config", help="one JSON file for the whole setup (see multi_chaser.example.json); command-line flags override it")
    ap.add_argument("--check-config", action="store_true", help="validate --config, print what would run, and exit")
    g2 = ap.add_argument_group("process supervision (optional: without these, start multi_rx.py and rx.py yourself)")
    g2.add_argument("--cc-cmd", help="command line for the control-channel SDR, e.g. 'python3 ./multi_rx.py -c cc.json -v 0'")
    g2.add_argument("--chase-cmd", help="command line for the chaser SDR, e.g. 'python3 ./rx.py --args rtl=1 ... -l http:0.0.0.0:8086'. "
                                        "--tune-http is taken from its -l http:HOST:PORT unless given.")
    g2.add_argument("--cwd", default=".", help="directory the commands run in (OP25's apps/ directory)")
    g2.add_argument("--log-dir", default="logs", help="child output goes to LOG_DIR/cc.log and chase.log")
    g2.add_argument("--child-log-max-mb", type=float, default=50.0, help="rotate a child log at this size (one .1 backup kept)")
    g2.add_argument("--child-env", action="append", default=[], metavar="KEY=VAL", help="extra environment for both children (repeatable)")
    g2.add_argument("--no-activity-ping", action="store_true",
                    help="don't set P25_ACTIVITY_PING=1 for the chaser SDR (then only decoded texts count as decode activity)")
    g2.add_argument("--stagger", type=float, default=3.0, help="seconds between starting the two SDR processes")
    return ap


def main():
    ap = build_parser()
    args = ap.parse_args()
    if args.config:
        try:
            cfg = load_config(args.config)
            built = build_from_config(cfg)
        except ConfigError as e:
            ap.error("config: %s" % e)
        print(describe(built, cfg))
        if args.check_config:
            print("\ngenerated multi_rx config:\n" + json.dumps(built["cc_json"], indent=2))
            return
        apply_config(args, ap, cfg, built)
        write_generated(built)
    elif args.check_config:
        ap.error("--check-config needs --config")

    def make(tune, log):
        return Chaser(args, tune, log)

    if args.replay:
        return replay(args, make)
    if args.chase_cmd and not (args.tune_http or args.tune_unix or args.dry_run):
        args.tune_http = http_from_cmd(args.chase_cmd)
        if not args.tune_http:
            ap.error("--chase-cmd has no '-l http:HOST:PORT'; give --tune-http")
        print("multi_chaser: tuning via %s (from --chase-cmd)" % args.tune_http)
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
    print("multi_chaser: TSBK udp/%d  min_dwell=%.1f routine_quiet=%.1f release_quiet=%.1f max=%.0f" % (
        args.tsbk_port, args.min_dwell, args.routine_quiet, args.release_quiet, args.max_dwell))
    sup = None
    if args.cc_cmd or args.chase_cmd:
        os.makedirs(args.log_dir, exist_ok=True)
        env = dict(os.environ)
        for kv in args.child_env:
            k, _, v = kv.partition("=")
            env[k] = v
        cap = int(args.child_log_max_mb * 1024 * 1024)
        kids = []
        if args.cc_cmd:
            kids.append(Child("cc", args.cc_cmd, args.cwd, os.path.join(args.log_dir, "cc.log"), env, cap, print))
        if args.chase_cmd:
            env_chase = dict(env)
            if not args.no_activity_ping:      # the decoder only sends its 51001 "decoded a PDU" ping when asked
                env_chase.setdefault("P25_ACTIVITY_PING", "1")
            kids.append(Child("chase", args.chase_cmd, args.cwd, os.path.join(args.log_dir, "chase.log"), env_chase, cap, print))

        def on_restart(name):
            if name != "chase":
                return
            with chaser.lock:                      # the retuned SDR came back at its start-up frequency
                chaser.cur_hz = None
                if chaser.mode == "CHASE":
                    chaser._release(time.time(), "child-restart")
        sup = Supervisor(kids, print, on_restart, args.stagger)
        chaser.restarts = lambda: {c.name: c.restarts for c in kids}

        def _term(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, _term)
        sup.start()
    try:
        live(args, chaser, sup.poll if sup else None)
    finally:
        if sup:
            sup.stop()


if __name__ == "__main__":
    main()
