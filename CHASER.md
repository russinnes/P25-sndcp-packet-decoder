# multi_chaser.py — two-SDR "wait and chase" guide

`multi_chaser.py` is Approach B from the [README](README.md): one RTL-SDR watches the control channel(s),
a second RTL-SDR is retuned to whichever data channel is about to carry a text for a radio you care about.
It is a single program that configures, launches and supervises both OP25 processes.

Read the README's [tuning section](README.md#tuning-is-the-whole-game) first. Data does not decode on a
badly tuned dongle, and each dongle must be tuned on its own.

## Contents
1. [How it works](#how-it-works)
2. [Hardware and the one-SDR window](#hardware-and-the-one-sdr-window)
3. [Quick start](#quick-start)
4. [Configuration reference](#configuration-reference)
5. [How a chase works](#how-a-chase-works)
6. [Output](#output)
7. [Reading the console](#reading-the-console)
8. [Estimating coverage before you buy a second dongle](#estimating-coverage-before-you-buy-a-second-dongle)
9. [Manual mode](#manual-mode)
10. [Troubleshooting](#troubleshooting)

---

## How it works

```
 SDR 1 ─▶ multi_rx.py  (one decoder per control channel, never retuned)
              │  every CRC-good TSBK  →  UDP 127.0.0.1:51002
              ▼
          multi_chaser.py ──── HTTP set_freq ────▶ SDR 2 ─▶ rx.py (chaser, :8086)
              ▲                                              │
              └──── decode activity (51001) / TMS text (51003) ◀┘
```

1. SDR 1 listens to every control channel you list, all at once, through one wideband capture.
2. Every TSBK is exported on UDP 51002. `multi_chaser.py` reads `IDEN_UP` messages to learn each tower's
   channel plan, and `SN_DATA_CHN_GNT` messages to learn that *radio X is about to get data on channel N*.
3. If radio X is in your RID range and the chaser is free, SDR 2 is retuned to that channel's frequency.
4. The decoder on SDR 2 recovers the text. The controller holds until the text is complete (or the channel
   goes quiet), saves the text, and goes back to waiting.

Because the chaser tunes an absolute frequency worked out from the tower's own `IDEN_UP`, a grant from
**any** watched site retunes SDR 2 correctly. The sites are assumed to announce the same channel plan
(the TSBK export carries no source tag); this is logged so you can check it (see
[Reading the console](#reading-the-console)).

Python standard library only.

## Hardware and the one-SDR window

* 2 × RTL-SDR (any rtl-sdr dongles), with serials set (`rtl_eeprom -d 0 -s 00000001`, `-d 1 -s 00000002`).
* An RTL-SDR runs to about 2.4 MS/s. **All control channels must fit inside SDR 1's usable window**:
  `sample_rate × usable_bw_pct`, less a 25 kHz guard at each edge, and every channel at least 25 kHz away
  from the centre (the DC spike). Two sites a few hundred kHz apart fit easily; a set that spans more than
  about 2 MHz will not fit one RTL-SDR.
* With `sample_rate: "auto"` (RTL only) the program picks the smallest standard rate that fits, centres on
  the channels, and moves the centre 150 kHz if a channel would land on the DC spike. `--check-config`
  tells you what it chose and warns when something does not fit.
* The chaser SDR runs at 1 MS/s. It only ever needs one channel at a time.
* Works on a Raspberry Pi. Two decoders (plus one per extra control channel) is a light load.

## Quick start

```bash
cd op25/gr-op25_repeater/apps
cp multi_chaser.example.json multi_chaser.json
$EDITOR multi_chaser.json
python3 multi_chaser.py --config multi_chaser.json --check-config    # prints the plan, runs nothing
python3 multi_chaser.py --config multi_chaser.json                   # run
```


Minimum edits to the example: both device serials (or indexes), `ppm` and `gains` per dongle (from your own
tuning), the control channel frequencies, and your `rid_ranges`.


Tip: tune each dongle on its own first. Put your known-good `ppm`/`gains` into the config and nowhere else.

## Configuration reference

Keys starting with `_` are comments and are ignored. An unknown key is an error (usually a typo).
Frequencies may be given in Hz (`770006250`) or MHz (`770.00625`).

### `control_channels` (SDR 1)

| Key | Default | Meaning |
|---|---|---|
| `device` | — | See [device](#device). Required |
| `frequencies` | — | **Required.** List of control channel frequencies, either numbers or `{"name": "SITE_A", "frequency": 770.00625}` |
| `sample_rate` | `"auto"` | `"auto"` (RTL only) or a rate in Hz |
| `center_frequency` | `"auto"` | `"auto"` or a frequency. Leave on auto unless you know better |
| `usable_bw_pct` | `0.9` | Fraction of the sample rate treated as usable |
| `http_port` | `8085` | multi_rx web dashboard |
| `verbosity` | `0` | OP25 `-v` level for this process |
| `demod` | cqpsk defaults | `demod_type`, `filter_type`, `excess_bw`, `if_rate`, `symbol_rate`, `gain_mu`, `costas_alpha` |

### `chaser` (SDR 2)

| Key | Default | Meaning |
|---|---|---|
| `device` | — | See [device](#device). Required |
| `start_frequency` | `"auto"` | Where SDR 2 waits before the first grant. `"auto"` is the average of the control channels |
| `sample_rate` | `1000000` | Chaser sample rate |
| `costas_alpha`, `gain_mu` | `0.02`, `0.04` | Demodulator loop settings |
| `http_port` | `8086` | Chaser web UI, also used for retuning |
| `verbosity` | `0` | OP25 `-v` level |
| `extra_args` | `""` | Appended to the `rx.py` command line as-is (e.g. `"-w"`) |

### `device`

Used by both SDRs.

| Key | Default | Meaning |
|---|---|---|
| `type` | `"rtl"` | |
| `serial` | — | Preferred. At least 2 characters (a single character is read as an index) |
| `index` | — | Used if no serial |
| `ppm` | `0` | Frequency correction for **this dongle** |
| `gains` | `""` | e.g. `"LNA:30"` |
| `buflen` | `16384` | |
| `args` | — | Raw osmosdr args string, overrides the above |

The two devices must not point at the same SDR; the check fails if they do.

### `policy`

| Key | Default | Meaning |
|---|---|---|
| `rid_ranges` | `[]` | `["LO-HI", ...]`. Only chase these radios. **Empty chases everyone**, which on a busy tower is far more than one chaser can follow |
| `follow_opts0` | `false` | Also chase grants with options = 0 (normally skipped) |
| `min_dwell` | `1.0` s | Minimum time on a channel |
| `release_quiet` | `2.0` s | Release after this long with nothing decoded |
| `routine_quiet` | `1.0` s | Release after this long once something has decoded |
| `max_dwell` | `12.0` s | Hard cap on one chase |
| `ack_grace` | `1.0` s | Wait for the radio's ACK after the text completes |
| `preempt_after` | `1.5` s | A grant for another radio may take over a chase that has decoded nothing for this long |
| `routine_text_max` | `8` | Texts up to this length are treated as routine radio traffic |
| `home_frequency` | `0` | Retune here when idle (0 = stay where it is) |
| `summary_every` | `300` s | Print the counter summary this often |

### `channel_plan`

`assume`: `"700mhz"` (default) uses a built-in 700 MHz plan until the tower announces its own;
`"none"` makes the chaser wait for `IDEN_UP`. The tower's own plan always replaces the built-in one.

### `paths`

| Key | Default | Meaning |
|---|---|---|
| `op25_dir` | `.` | The OP25 `apps/` directory (where `rx.py` and `multi_rx.py` live) |
| `log_dir` | `logs` | Child process logs (size-capped, see `process`) |
| `text_log` | `chaser_texts.jsonl` | Decoded texts |
| `python` | `python3` | |

### `process`

| Key | Default | Meaning |
|---|---|---|
| `stagger` | `3.0` s | Delay between starting SDR 1 and SDR 2 |
| `child_log_max_mb` | `50` | Size cap for each child log |
| `activity_ping` | `true` | Sets `P25_ACTIVITY_PING=1` for the chaser so the controller sees decode activity. Leave on |

Any command-line flag overrides the same setting in the config file.

## How a chase works

States: **IDLE** and **CHASE**.

* A data grant for a wanted radio while IDLE: tune SDR 2 to the granted channel and enter CHASE.
* If SDR 2 is already on that channel, no retune is needed.
* While chasing, the decoder's activity messages keep the chase alive. A chase ends on the first of:
  * **text-complete**: the message is complete (plus `ack_grace`);
  * **routine-quiet**: `routine_quiet` after the last decode;
  * **quiet**: `release_quiet` with nothing decoded at all;
  * **max-dwell**.
* A grant for a different radio while chasing is counted as **BUSY-MISS** and skipped, unless:
  * it is on the same channel (**PIGGYBACK**: the chaser is already there), or
  * the current chase has decoded nothing for `preempt_after` (**preempted**).
* The same text is suppressed for 60 s so repeats are not saved twice.


## Output

Every decoded text is appended to `text_log` as one JSON object per line:

```json
{"ts": 1700000000.0, "rid": 1000000, "ch": 4321, "port": 4007, "partial": false, "text": "..."}
```

`partial` is true for an incomplete preview. `port` is the UDP destination port (4007 is TMS). The
decoder also publishes on UDP 51003 as described in the README's
[Outputs](README.md#outputs-and-integration), so other tools can consume texts live.

**These files contain message content.** Treat them as sensitive.

**Message length.** System texts are typically capped by the sender at a 256-byte UDP payload, which is 125
UTF-16 characters. A text cut at that length was truncated at the source, not by the decoder.

## Reading the console

| Line | Meaning |
|---|---|
| `control channels (N) on one SDR: …` | What `--check-config` / startup decided: rate, centre, window, command lines |
| `channel plans: heard iden 0,1,2 \| grants use iden 0 (N)` | Plans heard from the towers, and the one grants actually use. Notes on each: `[matches the built-in plan]`, `[heard from tower]`, `[REPLACES the built-in plan]`, `[CHANGED]` |
| `first data grant uses iden N (channel C -> X MHz)` | Confirms a channel was converted to a real frequency |
| `first decode (text) rid=… ms after grant` | Grant-to-decode latency for that chase |
| `BUSY-MISS` | A wanted grant arrived while the chaser was busy elsewhere |
| `PIGGYBACK` | Another radio on the channel already being chased |
| `(nothing decoded)` | The chase decoded no data. Common for radio-originated traffic |
| periodic summary | Counters: grants seen, chases, decodes, `missed_busy`, releases by reason, restarts |

If a child process exits, the last lines of its log are printed, then it is restarted with backoff.
If SDR 2 restarts, any chase in progress is released.

**Healthy:** decodes ≈ chases that were for downlink text, `missed_busy` near 0, no repeated restarts,
and `CHANGED` never appearing in plan notes. If the two sites ever report different plans, the
single-plan assumption is wrong for your system; open an issue.

## Estimating coverage before you buy a second dongle

You can replay a recorded grant log through the same state machine with a simulated clock:

```bash
python3 multi_chaser.py --replay grants.log --rid-range LO-HI --replay-tune-ms 300
```

It reports how many target bursts one chaser would have covered. On a busy tower **filter by RID range**.
The replay models the retune time, so treat the result as an estimate.

## Manual mode

You do not need the config file. `--cc-cmd` and `--chase-cmd` take full command lines for the two
processes, and `--tune-http http://127.0.0.1:8086/` points at the chaser's web server:

```bash
python3 multi_chaser.py --cwd . \
  --cc-cmd    "python3 multi_rx.py -c my_multi_rx.json" \
  --chase-cmd "python3 rx.py --args rtl=<SN2>,buflen=16384 -N LNA:25 -S 1000000 -f <HZ> -q <ppm> -l http:0.0.0.0:8086 -v 0 -w" \
  --rid-range LO-HI --assume-700mhz-plan
```

Omit `--cc-cmd`/`--chase-cmd` to run only the controller and start the two decoders yourself. `--dry-run`
prints the tunes without sending them; `--verbose` logs every grant. Run `--help` for everything.

## Troubleshooting

| Symptom | Likely cause | Try |
|---|---|---|
| Config error on start | typo in a key, or both devices identical | The message names the key. Keys starting with `_` are comments |
| `WARNING` about channels not fitting | control channels span too much for one RTL | Split across towers/dongles, or use [Approach A](README.md#approach-a--wideband-multi-channel-airspy--multi_rxpy) |
| Control process exits immediately | wrong working directory (config or `rx.py` not found), or the dongle is already in use | Check `paths.op25_dir`; look for a stray `multi_rx.py`/`rx.py` from an earlier run; read the tail printed on exit |
| Chases but "(nothing decoded)" | activity ping off, or the traffic was a radio's own transmission | Keep `process.activity_ping` true; this is normal for uplink-only chases |
| Many `BUSY-MISS` | RID range too wide, or genuinely simultaneous traffic | Narrow `rid_ranges`; this is the limit of a single chaser |
| Decodes on the control channel, nothing from the chaser | chaser dongle untuned | Park SDR 2 on a busy data channel and tune it individually |
| Texts cut at ~125 characters | sender truncation | Not a decoder fault |
| Nothing is chased | RID outside `rid_ranges`, grants have options 0, or the plan is unknown | Run with `--verbose`; try `follow_opts0`; keep `channel_plan.assume` at `700mhz` if your tower uses it |
| Wrong dongle used | serial not set on the dongle | `rtl_eeprom -d N -s 0000000N`, then use `serial` |

See the README's [troubleshooting](README.md#troubleshooting) for tuning-related problems.
