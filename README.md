# OP25 data-channel text message decoder (P25 Phase 1 trunked data)

Recover **text-message / data traffic** from a P25 Phase 1 trunked system with cheap SDRs and a patched
[OP25](https://github.com/boatbod/op25). Developed against P25 Phase 1.

Two ways to run it, from the same decoder:

| | **A — Wideband "hear everything"** | **B — Budget "control channel + chaser"** |
|---|---|---|
| Hardware | 1 × Airspy Mini (or any wideband SDR) | 2 × RTL-SDR |
| Idea | one SDR covers *every* data channel at once; one decoder per channel | SDR 1 sits on the control channel; SDR 2 is retuned to each data grant |
| Process | `multi_rx.py` | two `rx.py` + `dual_chaser.py` |
| Catches the **first** transmission | yes | **usually** (see [the timing problem](#the-timing-problem)) |
| Needs a channel plan | no | learned from the control channel |
| Tuning effort | one device, many channels | **two** devices, each tuned individually |
| CPU | higher (one decoder per channel) | low (RPi-friendly) |

> **Read this first.** Data is unforgiving. Voice has redundancy and repetition, and a dropped
> voice frame is a click. A data packet is a few hundred bytes with no forward repetition, and one lost
> block loses the message. **If the receiver is not tuned precisely, it will not decode.** Both
> configurations below spend most of their instructions on tuning, and that is deliberate. Do not skip
> [Tuning](#tuning-is-the-whole-game).

## Contents

1. [How it works](#how-it-works)
2. [The timing problem](#the-timing-problem)
3. [Tuning is the whole game](#tuning-is-the-whole-game)
4. [Approach A — wideband multi-channel](#approach-a--wideband-multi-channel-airspy--multi_rxpy)
5. [Approach B — two RTL-SDRs and a chaser](#approach-b--two-rtl-sdrs-and-a-chaser)
6. [Reading the logs](#reading-the-logs)
7. [Outputs and integration](#outputs-and-integration)
8. [Building](#building)
9. [Troubleshooting](#troubleshooting)
10. [Status, limits, legal](#status-limits-legal)

For *what changed in OP25 and why*, see **[CHANGES.md](CHANGES.md)**.

---

## How it works

A trunked P25 system has a **control channel** (CC) that broadcasts, continuously, who is talking
where. Voice and data traffic run on separate **working channels** that are assigned per call.

```
  CC (continuous)        data channel (bursty, ~seconds)
  ┌──────────────┐       ┌──────────────────────────────────────────────┐
  │ SN_DATA grant│──────▶│ header PDU │ data block │ … │ data block │ ACK │
  │ "radio X →   │       └──────────────────────────────────────────────┘
  │  channel N"  │         │             │  rate-3/4 trellis, CRC-9 each
  └──────────────┘         ▼             ▼
                      12-byte header   16 data bytes per block
                      (CRC-16, RID,    ──▶ SNDCP ──▶ IPv4 ──▶ UDP :4007
                       #blocks, …)         ──▶ UTF-16LE text  (+ packet CRC-32)
```

1. The CC announces a **data grant** (`SN_DATA_CHN_GNT`, or individual/group data grants) naming a
   radio (RID) and a channel. The channel *number* is turned into a frequency using the tower's own
   `IDEN_UP` broadcast (base frequency, spacing, offset), so no hand-built channel table is needed.
2. A short time later (tens of milliseconds) the tower sends a **PDU** on that channel: a header,
   then *confirmed-data* blocks, each independently coded (rate-3/4 trellis) and individually
   CRC-9-protected, then the radio sends an ACK.
3. If the blocks carry a Motorola TMS message as **SNDCP → IPv4 → UDP port 4007**, with the text in
   UTF-16LE, and a 32-bit CRC over the whole packet.
4. If the radio missed a block, the tower may send a **selective retry** carrying only the missing
   blocks. The decoder merges a retry into a partly-heard first copy.
5. Any reassembled IPv4/UDP datagram whose payload contains readable text,
   whether UTF-16LE or plain ASCII, is printed as a `TMS_TEXT` line and exported on UDP 51003 with
   its real destination port in the port field. That is how text from other vendors' radios
   and data gateways shows up alongside the TMS pages. Filter on `port` in whatever consumes the feed.
   Ports with a known meaning are handled specially: **4005** (ARS, the automatic registration service)
   and **64414** (OTAR/key-management traffic and keepalive beacons, never exported as text).
   The `LEDGER-FMT` line lists the other destination ports seen and how many datagrams each carried,
   which is the quickest way to find what else is on your system.

Stock OP25 can follow voice but cannot decode steps 2–3. This project adds that
([details](CHANGES.md#3-rate-34-trellis-decoding-of-confirmed-data-blocks)), then works hard to avoid
throwing away partial information ([CHANGES §4–5](CHANGES.md#4-not-throwing-data-away-failed-blocks-and-failed-headers)).

### The timing problem

The PDU begins up to roughly **500ms** after the grant. A single SDR that must *retune and re-lock*
(about 190–400 ms in our measurements) will therefore **may miss the first copy**. What a chaser can still
catch is **almost all**, a repeat, or the later per-radio copies (a message to several radios is sent once
per radio, a few seconds apart).
Approach A has no such problem: every channel already has a receiver on it before the grant.
Approach B exists because it is a fraction of the cost and is good enough where "any copy" is the
goal. Choose with that in mind. You can estimate how well a chaser would do on **your** tower before
buying a second dongle by recording the control channel and replaying it
(`dual_chaser.py --replay`, see [Approach B](#approach-b--two-rtl-sdrs-and-a-chaser)).

---

## Tuning is the whole game

This applies to **every** device, and to **each** dongle separately: two RTL-SDRs of the same model
have different frequency error, different temperature drift and different gain curves. A setting
that is right for one is wrong for its twin.

### Why data is stricter than voice

* Voice (IMBE) conceals errors and repeats constantly. A bad frame is a glitch.
* Data blocks are protected by a trellis code and a CRC, and a message needs *all* of its blocks
  (or a retry for the ones it lacks). A marginal link doesn't degrade gracefully; messages just vanish.
* Data channels are **bursty** (a few percent duty cycle). You cannot judge them by watching them for
  a moment: the constellation is empty between bursts. Tune on a *continuous* signal (the control
  channel) and verify on the data channels with counters and measurements.

### What "dialled in" means

1. **Frequency.** Correct the dongle's `ppm` so the control channel is centred. A residual offset of
   a few hundred Hz is tracked by the demodulator's carrier loop, but a large or *wandering* offset
   uses up margin you need elsewhere. If one channel sits well away from the rest, suspect that
   transmitter (or interference) before blaming your dongle.
2. **Gain, neither too high nor too low.** Too low: noise-limited, weak channels fall below the
   decoder's floor. Too high: front-end overload and intermodulation from strong nearby transmitters
   create junk that looks like (and masks) signal. The right setting is the highest gain that stays
   clean, not the highest gain.
3. **A clean constellation on a real signal.** On the control channel (CQPSK) you should see **four tight,
   well-separated clusters**. Smeared or ring-shaped means too much gain, overload, or a frequency
   problem.
4. **Counters that agree.** On data channels, a healthy run shows `pdu_passed` ≈ `pdu_attempted`,
   `pkt_crc_fail=0`, and no `ACK-NODL` lines for radios you know were messaged.

### Procedure — RTL-SDR (approach B, and any single-channel `rx.py` run)

Do this once **per dongle**, on the dongle's final antenna and cabling, at its operating temperature
(let it warm up first, typically 15–20 minutes; RTL oscillators drift while warming).

1. Give each dongle a serial so you never mix them up: `rtl_eeprom -d 0 -s 00000001`, then
   `-d 1 -s 00000002`, and use `rtl=<serial>` in all commands.
2. Park it on the control channel with `rx.py` and open the web UI (`-l http:0.0.0.0:8085`).
   Enable the **constellation** and **datascope/bitstream** plots.
3. **ppm:** with gain mid-range, change `-q <ppm>` until the carrier frequency error shown for the CC
   is as close to 0 as you can get; re-check after another 10 minutes. Write the final value down
   *against that dongle's serial*.
4. **Gain:** start low and step up (`-g`, e.g. 20 → 25 → 30 → 35 …). At each step watch the
   constellation for at least 30 s. Tightening clusters = good. The point where they stop improving,
   or start to smear, is your ceiling. Pick the value just below the ceiling, not at it.
5. **Check the other end.** Switch to the *weakest* data channel you care about (the one from the
   farthest tower) and confirm it still decodes at that gain; then confirm the *strongest* nearby one
   doesn't overload. If the two need different gains, you have a dynamic-range problem. Note the
   trade-off or consider a band-pass filter or attenuator.
6. Repeat for the second dongle. Do not copy settings across dongles.
7. Re-run (3) after a few days and after a season change; record drift.

---

## Approach A — wideband multi-channel (Airspy + `multi_rx.py`)

One Airspy Mini at 6 Msps covers ~5 MHz, enough for 15+ channels of a typical site. `multi_rx.py` runs
one decoder per channel in a single process. You can also add channels from multiple towers
assuming you can hear them clearly. 

### Hardware

* Airspy Mini (R820T2) or similar, and a decent antenna with a clear view of the sites you want.
  On strong signals an external LNA or the bias-T can make overload worse; start without them.
* A machine with enough CPU for one decoder per channel. **An RPi will not handle more than about 4 channels.**

### Install

See [Building](#building). Then confirm the Airspy is visible:

```bash
airspy_info
SoapySDRUtil --find      # or: python3 -c "import osmosdr"
```

### Configure (`config.json`)

The device `frequency` must be the **mathematical centre of the span** covering your widest data channels — not
the control channel, and ideally not sitting on any individual working channel (DC spike at centre).
You are listening to an entire ~3–5 MHz band at once so you catch every grant without retuning.
(See example-multirx.json in ./apps/ folder)

Illustrative example (replace frequencies, ppm, and gains with your own):

```json
{
  "devices": [{
    "name": "airspy",
    "args": "airspy=0,sensitivity=0,linearity=0,bias=0",
    "rate": 6000000,
    "frequency": 772643750,
    "offset": 0,
    "ppm": 2,
    "gain_mode": false,
    "gains": "LNA:10,MIX:12,IF:8",
    "tunable": false,
    "usable_bw_pct": 0.9
  }],
  "channels": [
    {
      "name": "SITE_A_f1",
      "device": "airspy",
      "frequency": 772031250,
      "demod_type": "cqpsk",
      "filter_type": "rrc",
      "excess_bw": 0.2,
      "if_rate": 24000,
      "symbol_rate": 4800,
      "enable_analog": "off",
      "destination": "0.0.0.0:0",
      "gain_mu": 0.04,
      "costas_alpha": 0.02
    },
    {
      "name": "SITE_A_f2",
      "device": "airspy",
      "frequency": 773013750,
      "demod_type": "cqpsk",
      "filter_type": "rrc",
      "excess_bw": 0.2,
      "if_rate": 24000,
      "symbol_rate": 4800,
      "enable_analog": "off",
      "destination": "0.0.0.0:0",
      "gain_mu": 0.04,
      "costas_alpha": 0.02
    }
  ],
  "terminal": {
    "module": "terminal.py",
    "terminal_type": "http:0.0.0.0:8085"
  }
}
```

Add one object to `channels` per frequency. Key points:

| Field | Meaning / how to choose |
|---|---|
| `rate` | 6,000,000 for Airspy Mini. Total span ≈ `rate × usable_bw_pct` |
| `frequency` (device) | centre of the span. Put it in the middle of your channels, and **not on one of them** (DC spike at centre) |
| `ppm` | per-dongle frequency correction, from the tuning procedure above |
| `gain_mode: false` + `gains` | manual per-stage gain. **Never use automatic gain** for data; it breathes with the bursts. Stages: `LNA` 0–14, `MIX` 0–15, `IF` 0–15 |
| `usable_bw_pct` | the outer edges of the band are filtered poorly; 0.9 leaves the usable middle 90% |
| `gain_mu`, `costas_alpha` | clock-recovery gain and Costas-loop bandwidth. Defaults above work; leave alone until everything else is right |
| `demod_type` | `cqpsk` for most data channels; `fsk4` for a C4FM-only system |

One object per channel; the same layout scales to 15+ channels across the 5 MHz span.

### Run

```bash
cd op25/gr-op25_repeater/apps
python3 multi_rx.py -c config.json 2>&1 | tee run.log
```

Decoded texts appear as `TMS_TEXT …` lines and are sent on UDP 51003 (see [Outputs](#outputs-and-integration)).
`--sndcp-debug 1` prints every PDU diagnostic (very verbose, good for tuning verification).

---

## Approach B — two RTL-SDRs and a chaser

```
 SDR 1 ─▶ rx.py (parked on the control channel, never retuned) ─ TSBK ─UDP 51002─┐
                                                                                ▼
 SDR 2 ◀─ rx.py (the chaser, HTTP :8086) ◀── set_freq ───────────── dual_chaser.py
          └─ decode activity ─ UDP 51001 / TMS 51003 ──────────────────────────▶
```

`dual_chaser.py` (in `op25/gr-op25_repeater/apps/`) learns the tower's channel plan (`IDEN_UP`),
follows data grants for the radios you care about, holds while the chaser decodes, and releases.
Standard library only. Use `--replay` to estimate coverage before buying a second dongle.

### Step 0 — decide whether it will work on *your* tower

Record the CC for an hour or two (SDR 1 only, TSBKs only), then replay:

```bash
python3 dual_chaser.py --replay cc.log --rid-range LO-HI
```

It reports how many target bursts one chaser would have covered. On a busy tower **you must filter by
RID range**. The replay *models* retune time; remember that, because of [the timing problem](#the-timing-problem),
the first copy is not catchable — only repeats and later copies.

### Step 1 — tune each dongle individually

Follow the [RTL procedure](#procedure--rtl-sdr-approach-b-and-any-single-channel-rxpy-run) **for both
dongles, separately**, before connecting them to the chaser. Prove each one decodes cleanly on its own
(SDR 1 on the CC; SDR 2 on a busy data channel) first. Only then add the chaser. A chaser on top of
an untuned dongle will look like a chaser problem and it won't be.

### Step 2 — run

This requires 3 separate terminals/shells. I recommend running them in a screen session to detach. 
(Combining into one launcher is on my list)

Prepare a trunking TSV (e.g. `trunk-cc.tsv`) that parks SDR 1 on your site's control channel.
This also points to `blank.wlist` to maintain the CC. 
Use the **ppm and gain values you measured for each dongle** — do not copy settings across dongles.

```bash
# SDR 1: control channel, never retuned. Exports TSBKs to UDP 51002 (built into the decoder).
# Be sure YOUR site control-channel frequency is in trunk-cc.tsv — that holds it on the CC.
python3 rx.py --args "rtl=<your_rtl_sn_1>,buflen=16384" -f <CC_HZ> -S 1000000 -o 150000 \
    -q <ppm1> -g <gain1> -D cqpsk -T trunk-cc.tsv -l http:0.0.0.0:8085

# SDR 2: the chaser. Starts on any data channel. Activity pings + TMS text.
P25_ACTIVITY_PING=1 python3 rx.py --args "rtl=<your_rtl_sn_2>,buflen=16384" -f <DATA_HZ> \
    -S 1000000 -o 150000 -q <ppm2> -g <gain2> -D cqpsk -l http:0.0.0.0:8086

# controller
python3 dual_chaser.py --tune-http http://127.0.0.1:8086/ --rid-range <LO>-<HI>
```

Keep any other flags you normally pass to `rx.py` for a known-good single-SDR run.
Start both decoders, wait for the controller to print `CHANNEL PLAN` lines (the plan arrives every few
seconds), then it begins chasing.

### What to expect
* Not all agencies use packet data / TMS etc. The traffic may simply not exist. 
* Texts to a radio are sent once **per radio**, a few seconds apart. Retune timing is the gap.
* The controller logs every chase with `grant → tune-done → first decode` timing and counts grants it
  could not take because it was busy (`BUSY-MISS`).

---

## Reading the logs

Mostly quiet by default; only recovered text reaches stderr unless started with `P25_SNDCP_DEBUG=1` env variable.

| Line | Meaning |
|---|---|
| `TMS_TEXT [ch] rid=N …` | A recovered message. `partial` = incomplete preview; `SALVAGE_TMS_TEXT` = recovered from a damaged header, RID unverified |
| `STATS nid_ok= nid_fail= pdu_attempted= pdu_passed= pkt_ok= pkt_crc_fail= pkt_incomplete= …` | Periodic per-channel counters. `nid_fail` is a *trend*, not a miss count (~3% is a normal baseline) |
| `ACK-NODL rid=N …` | The radio acknowledged something we never decoded: **we missed a downlink**. The most useful miss detector |
| `LEDGER …` | Reconciles every frame to a named outcome; `unaccounted=` should be 0 |
| `LEN MISMATCH` | IP and UDP length fields disagreed (noise in a length byte); the UDP one is used |

`d34_*` counters in `STATS` are process-wide, so on a multi-channel run they repeat on every channel.

---

## Outputs and integration

All local (`127.0.0.1`), fire-and-forget UDP:

| Port | Producer | Content |
|---|---|---|
| 51003 | decoder | JSON per recovered text: `{"ts","rid","port","partial","tms_text"}` (+`"salvage":true` when applicable) |
| 51002 | decoder | every CRC-good TSBK, 14 raw bytes (opcode, last-block flag, 12 octets) |
| 51001 | decoder (`P25_ACTIVITY_PING=1`) | `{"rid":N,"sec":1}` per header-valid PDU |

`dual_chaser.py --text-log chaser_texts.jsonl` appends every decoded text as JSON lines.

---

## Building

Requires **GNU Radio 3.10**, `gr-osmosdr` or SoapySDR with the Airspy/RTL drivers, `cmake`, a C++17
compiler, and the usual dependencies of [boatbod/op25](https://github.com/boatbod/op25).

```bash
git clone https://github.com/russinnes/P25-sndcp-packet-decoder.git
cd P25-sndcp-packet-decoder
./install.sh
```

After install, the apps live under:

```text
op25/gr-op25_repeater/apps/
```

For distro-specific notes (packaged vs locally-built GNU Radio), see [README-gr3.10](README-gr3.10).
For RTL-SDR blacklisting / driver tips, see [README-update-rtlsdr.md](README-update-rtlsdr.md).

---

## Troubleshooting

| Symptom | Likely cause | Try |
|---|---|---|
| Nothing decodes | wrong `demod_type`, large ppm error, or gain far off | Tune on the CC first ([procedure](#procedure--rtl-sdr-approach-b-and-any-single-channel-rxpy-run)); confirm constellation clusters are tight |
| Works on CC, not on data channels | CC is strong; data channel is weaker or sits in a different part of the passband | Re-check gain on the *weakest* data channel; lower LNA if strong neighbours overload; confirm ppm still holds after warm-up |
| Lots of junk bursts / worse with *more* gain | overload / intermodulation from a strong nearby signal | Lower LNA gain; add an attenuator or band-pass filter |
| One channel always off by a few hundred Hz | transmitter offset, not you | No action needed if it still decodes |
| Chaser logs a decode but no text | the "decode" was an ACK, not the page | Expected — see [timing](#the-timing-problem) |
| Constellation empty on a data channel | no burst right now | Not a fault; data is bursty |
| `ACK-NODL` lines for radios you know were messaged | missed downlink (tuning, timing, or busy chaser) | Check ppm/gain; for Approach B review `BUSY-MISS` and retune latency |

---

## Status, limits, legal

**Tested on:** P25 Phase 1 CQPSK data channels; Airspy Mini on an x86 Linux box;
dual chaser built for and tested with 2× RTL-SDR on a Raspberry Pi (aarch64).
Phase 2 (TDMA) is out of scope.

**RPi users:** The chaser path is the practical choice on low-power CPUs — only two decoders run at
once. Airspy/`multi_rx.py` is more complete but needs substantially more CPU.

**Privacy and law.** Rules on monitoring radio traffic differ by country and by service, and data
PDUs can contain unique information. You are responsible for operating this lawfully.
Do not redistribute decoded content.

**License.** OP25 is GPL-3.0; so are the modified files here and any derivative. This is released
under the same terms. Credit to Max H. Parke (KA1RBI), Graham J. Norbury and the OP25 contributors;
to the authors of the Wireshark P25 dissector (the rate-1/2 deinterleave is adapted from it); and to
`kchmck/p25.rs` and TIA-102.BAAA-A for the trellis and CRC definitions.
