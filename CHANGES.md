# CHANGES — what differs from stock OP25, and why

## 1. The problem these changes solve

Stock OP25 is built to follow **voice**. Its P25 Phase 1 data-PDU path is minimal: it decodes
rate-1/2 trellis blocks (unconfirmed data and control PDUs), and it stops at the first block that fails.
PDU texts (Motorola TMS, others) travel as **confirmed data** — rate-3/4 trellis blocks, each with
its own CRC-9, wrapped in SNDCP / IPv4 / UDP (port 4007) and carrying UTF-16LE text. Stock OP25
cannot decode those blocks, so the texts were invisible. Making them visible, and then making the
decode *reliable* (a lost block means a lost page, with no voice-style redundancy to cover it), is the
whole of this work.

| Area | Stock OP25 | This fork |
|---|---|---|
| Confirmed data (rate 3/4) | not decoded | Viterbi decoder + per-block CRC-9 |
| A block that fails | whole PDU dropped | block kept/flagged; PDU continues |
| Header CRC fails | PDU dropped | text still recovered from the data blocks, with safeguards |
| PDU → message | none | block → packet (CRC-32) → IPv4/UDP → text, with retry merging |
| Message export | none | UDP JSON on 51003 (+ TSBK on 51002, activity ping on 51001) |
| Several channels in one process | shared global state | per-instance state; one decoder per channel |
| Health visibility | `nid` counters only in debug | `STATS`, `LEDGER`, NID ring, `ACK-NODL` lines |

## 2. File-by-file summary

| File | Size vs upstream | Nature of change |
|---|---|---|
| `lib/p25p1_fdma.cc` | 798 → 2909 lines (~2,160 added, ~50 removed/changed) | the substance — see §3–§9 |
| `lib/p25p1_fdma.h` | 136 → 163 lines | `RidReasm` struct moved in as a member; `set_dwell_rid()`; `#include`s |
| `lib/rx_sync.cc` | one hunk, ~17 lines | deliver a PDU body when the sync window expires — §10 |
| `lib/p25_frame_assembler_impl.cc` | 173 → 193 lines | guard `control()` against exceptions; route `set_dwell_rid` — §11 |
| `apps/multi_rx.py` | +25 lines | pass `gain_mu`/`costas_alpha` per device; `set_dwell_rid` plumbing — §12 |
| `apps/http_server.py` | +57 lines | fire-and-forget commands no longer wait for a reply — §13 |
| `apps/rx.py` | **to be diffed against the pinned commit** | see §14 |
| `apps/p25_demodulator.py`, `apps/trunking.py`, `lib/rx_sync.h`, `lib/p25_frame_assembler_impl.h` | **verify** | these were byte-identical between our two machines; run the diff script to see what, if anything, differs from upstream |

New (not in OP25): `dual_chaser.py`, `gain_sweep.py`, `nidshare.sh`, example JSON configs.

## 3. Rate-3/4 trellis decoding of confirmed data blocks

**What.** `block_deinterleave_34()` (new). Same 196-bit interleave table as stock's rate-1/2
`block_deinterleave()`, but the 196 coded bits are viterbi-decoded as a 3/4-rate trellis (49 tribits →
18 octets: 2 octets of serial number + CRC-9, then 16 data octets). Tables are TIA-102.BAAA-A
Table 7-2 (cross-checked with the tribit state machine in `kchmck/p25.rs`).

**Acceptance rule.** The Viterbi path cost gives a quality number. A block is accepted when
`cost <= 8`, **or** when `cost > 8` but its CRC-9 passes (`d34_rescued`). A radio accepts a block on its
CRC, not on our trellis metric, so a block that fails the cost gate but passes CRC-9 is a block the
radio genuinely received. A block with `cost <= 8` but a bad CRC-9 is *kept and counted*
(`d34_crc_bad`) rather than dropped, because the packet-level CRC-32 (§5) is the final arbiter.

**CRC-9.** `blk_crc9_ok()`: generator `x^9+x^6+x^4+x^3+1` over the 7-bit serial number plus 16 data
octets, result inverted. Verified against 1680/1680 real blocks. It is a *real* check — random noise
passes it 1 time in 512 — which is why later salvage logic (§4) uses it as a gate.

**Why it matters.** Without this there is no text at all.

## 4. Not throwing data away: failed blocks and failed headers

Stock `process_blocks()` returns `-1` on the first rate-1/2 block that fails, and the caller drops the
PDU. `process_PDU` also drops any PDU whose 12-byte header fails CRC-16. For voice-era use that is fine.
For dispatch pages it loses messages in a way that is easy to measure: a header can fade while the
data blocks after it are intact (each block is independently coded).

Changes in `process_blocks()`:

1. **Block failure is not fatal.** A failed rate-1/2 block is logged and kept (zeros), and the loop
   continues. Later stages decide whether the PDU is usable.
2. **Header CRC fail, live session.** If a data session for a known RID is active and recent on this
   channel (within `REASM_WINDOW_SEC`), the burst's blocks are decoded against that cached RID instead
   of being discarded. Because each channel now has its own decoder with its own reassembly state (§7),
   the cached RID can only have come from traffic actually heard on that channel.
3. **Header CRC fail, no session ("headless" bursts).** Decode the data blocks anyway and gate
   acceptance on the *data* looking real: Viterbi cost, a serial number in range (observed FSNs run
   0–37; 3 of 36,372 accepted blocks exceeded 127, so the check is `< 128`), and, once reassembled,
   an IPv4/UDP shape. The RID is attributed from the failed header only as a clearly-marked guess, or 0.
   A page with no RID is still better than a silent drop.
4. **All-zero header with a text-sized burst** (seen: 18 blocks, the second radio's copy of a
   dispatch). Earlier this was skipped as a "ghost burst". Now it is tried, but only blocks whose own
   CRC-9 passes are accepted and only if the datagram's IPv4 header checksum is valid (`ip_hdr_cksum_ok`).
   Noise cannot pass both.
5. **Ghost-burst guard.** Large bursts whose decoded header is all zero except byte 0 (e.g.
   `76 c0 00 3c 00 …`, `nblk=20`) skip the Viterbi work entirely; they are noise, not a lost page.
6. **HDRFAIL → ACK linking.** A header-failed PDU has no RID. The tower ACKs the receiving radio
   ~0.5 s later, and the ACK carries the RID. The failed PDU (and any text salvaged from it) is
   remembered per channel and linked to the next ACK within `HDRFAIL_ACK_WIN` = 1.2 s, printed as
   "probable" with the time delta so the reader can judge. 1–2 block failures are treated as damaged ACKs
   and never linked.

Salvaged content carries a `salvage` tag in logs and is **not** exported on UDP 51003 unless
`P25_EXPORT_SALVAGE=1`, because its RID may be a guess.

## 5. From blocks to a message

### 5.1 Packet-level assembler (`PktAsm`, `g_pkt`)
Keyed by `(LLID, N(S), FSNF)` — the packet identity in TIA-102.BAAA-A 6.3–6.4 and TSB102.BAAD — and
**shared across channels under a mutex**, because a retry can arrive on a different data channel from
the first copy. Blocks are placed by serial number (`blk[128][16]`, `have[128]`; 7-bit serial, so 128
slots), so a missing block leaves a hole rather than shifting everything after it. A selective retry
(FMF=0) carries only the blocks the radio missed and fills those holes. The trailing 32-bit packet CRC
(poly `0x04C11DB7`, init 0, output inverted; verified on ~1450 real packets, 100% of complete first
copies matched) validates the whole message once every block is present. `PKT_WINDOW_SEC` = 30 s.

It is deliberately *additive*: it counts complete-and-good (`pkt_ok`), complete-but-bad
(`pkt_crc_fail`), never-completed (`pkt_incomplete`) and retry-without-original (`retry_no_original`)
packets, and emits a message through the text path **only** when a retry completes or repairs it, the
one case the older path could not handle.


* **Window** `REASM_WINDOW_SEC` raised 2.0 → 4.0 s so RF fades and repeater turnaround between bursts
  of one session do not split it. Trade-off: two genuinely separate bursts to one RID are more likely
  to be glued; the symptom is a wrong IP version nibble or nonsense ports in `REASM` output.
* **New-datagram detection.** If the buffer holds an unfinished datagram and the next burst's first
  block starts with an SNDCP header (`51 00`) + IPv4 header, that is a *new* datagram (usually a
  retransmission). It is closed as unfinished and a fresh one started. Previously it was appended,
  producing bogus `00 02 51 00` ("Q") replies.
* **Duplicate rejection.** Only an exact duplicate of the last accepted block (an air retry) is
  rejected. The old "must be next serial" test never matched a real continuation in the field.
* **IP length cross-check.** The IP header's total length and the UDP header's own length must agree.
  A single bit error in just the two IP length bytes used to make the reader run past the datagram
  into the start of the next one (seen: one message's tail glued onto the next message's prefix). On
  disagreement the UDP-derived length is used and `LEN MISMATCH` is logged.
* **Plausibility cap.** `MAX_PLAUSIBLE_IPLEN` = 1500. A UDP length field landing on `0xF5F5` filler
  decodes to 62,965, which made the reassembler hold open for minutes waiting for a 63 KB message.
* **Length lied, bytes fine.** When the length fields are untrustworthy but the RID is still verified
  (header passed CRC), the buffer is scanned for text directly, starting after the IP+UDP header
  (not from byte 0, or the IP version byte `0x45` followed by ToS `0x00` decodes as a spurious
  leading `E`). Exported as `partial`.
* **Per-instance state** — see §7.

## 6. Reading the text

`extract_text()` (shared by every path): try **UTF-16LE** first (printable low byte followed by `0x00`);
fall back to a plain **ASCII** scan when UTF-16LE finds nothing, or finds far fewer characters than
ASCII does. The second condition exists because a plain-ASCII message can contain one accidental
`xx 00` pair at an even offset, which used to yield a one-character result and block the fallback. A
lone printable byte in a binary buffer is noise, so a minimum run length is required.

**Other ports.** Text is not restricted to 4007. For any destination port other than 4007, 4005 or
64414, a datagram whose payload yields at least 4 extractable characters is printed as `TMS_TEXT` and
exported on UDP 51003 with its true `port` (also the `partial` preview path). This is how non-Motorola
text, such as BK Technologies / RELM radios, is captured. The trade-off is that it is a *shape* test,
not a protocol test, so an unrelated service whose payload happens to contain a run of printable bytes
can appear in the feed. Consumers should filter on `port`. The `LEDGER` line counts such datagrams
(`port_other`, `other_port_no_text`) and `LEDGER-FMT` lists up to 8 distinct "other" destination
ports with counts; datagrams on a port that produced no text are counted but not printed.

Port handling: **4007** is Motorola TMS, **4005** ARS, **64414** OTAR/KMM key management
and keepalive "beacons" (never text; they carry the sender's own RID at a fixed offset, which gives a
free consistency check). Datagrams of 3–4 bytes on 4005/4007 (`00 01 3f`, `00 02 70 00`) are binary
control packets, not text; previously `0x70` printed as the letter "p".

`partial` previews: while a message is provably incomplete but a valid IP+UDP header and some payload
exist, the text so far is printed and exported flagged `partial:true`, and refreshed as more bursts
arrive.

## 7. Per-channel state (needed for `multi_rx.py`)

Upstream keeps reassembly state in a single file-scope global. With one decoder (`rx.py`, retuned
SDR) that is harmless. In `multi_rx.py` there is one `p25p1_fdma` per fixed-frequency channel in the
**same process**, so two radios landing PDU bursts on different channels within the window would
interleave into one buffer and corrupt both. `RidReasm` is now a member of `p25p1_fdma`
(`p25p1_fdma.h`), and per-channel diagnostics are arrays indexed by `msgq_id & 15` (so up to 16
channels per process).

**Known wart.** The three `g_stat_d34_*` counters are *process-wide*, not per-channel, so on a
multi-channel run every channel's `STATS` line shows the same `d34_*` values. Read them as a total.

## 8. Outputs added

| Output | Where | Format |
|---|---|---|
| Decoded TMS text | UDP `127.0.0.1:51003` | JSON `{"ts","rid","port","partial",["salvage"],"tms_text"}`; only for real content, never the empty 4-byte poll or beacons |
| Every CRC-good TSBK | UDP `127.0.0.1:51002` | 14 raw bytes: opcode, last-block flag, 12 TSBK octets. No NAC (it was constant) |
| Decode-activity ping | UDP `127.0.0.1:51001` | `{"rid":N,"sec":1}` per header-valid PDU; **off** unless `P25_ACTIVITY_PING=1`. Used by the chaser to know when to hold/release |
| Last PDU burst | `/tmp/pdu_last.bin` | truncated on each write |

Also: failed TSBKs are now logged with their raw bytes, and `bytes[7:10]` is annotated as the plausible target RID.

## 9. Diagnostics

All are log lines on stderr from the decoder. Quiet by default (§ env vars): only recovered text
reaches stderr unless `P25_SNDCP_DEBUG=1`.

* **`STATS`** (periodic, from `load_nid`): `nid_ok`, `nid_fail`, `pdu_attempted/passed`, `reasm_*`,
  `pkt_*`, `d34_*`, `ack_nodl`, `duid_*`. 
* **NID ring** (`g_nid_ring`): the last 64 frame-detection events per channel (NID ok/fail, DUID,
  NAC, length, whether the body completed).
* **`ACK-NODL`**: the tower only acknowledges an uplink the radio actually sent. If an ACK arrives for
  an RID with no downlink decoded on that channel for `NODL_AGE_SEC` = 4 s, a downlink was on the air
  and we missed it (or the radio sent an unsolicited uplink). 
* **`LEDGER`** (v8; on by default, `P25_LEDGER=0` disables): classifies every PDU frame handed over,
  every IP datagram built, and every text printed so "frames detected" reconciles to named outcomes;
  the `unaccounted=` field should be zero.
* **Frame bookkeeping**: a frame with a good NID whose body never completed before the next NID is
  counted as `abandoned`.

## 10. `rx_sync.cc` — deliver the PDU body on sync expiry

A P25 PDU carries no length; its body is handed to the framer only when the **next** sync arrives. If
the tower goes quiet after a PDU there is no next sync, the expiry fires, and `sync_reset()` threw the
whole PDU away. The patch delivers what has been received so far (up to 3,600 symbols ≈ 35 blocks;
the rest is noise) via `p25fdma.load_body()` before resetting. Only on `RX_TYPE_P25P1`, only if at
least 100 symbols were received. The log line is gated behind `P25_SNDCP_DEBUG`.

## 11. `p25_frame_assembler_impl.cc` — control() cannot take the receiver down

`control()` parses JSON from the HTTP terminal. `json::parse()` and `j["field"].get<T>()` throw on
malformed input; uncaught, that unwinds out of a GNU Radio message callback and was observed to
**segfault** (a backtrace resolving into `nlohmann::json::detail::parse_error::create`). The whole
body is now in `try/catch`; a bad command logs and returns `{}`. A `set_dwell_rid` command was added.

## 12. `multi_rx.py`

* Per-device `gain_mu` / `costas_alpha` demodulator parameters are passed through (they were being
  ignored), so the clock-recovery loop gain and Costas-loop bandwidth can be tuned from the JSON.
* `set_dwell_rid` plumbing for an external controller. Present but unused in normal operation: with
  fixed channels there is no retune event to key it on.
* `--sndcp-debug N` sets `P25_SNDCP_DEBUG` for the decoder (added by a small patch script,
  `tools/apply_sndcp_flag.sh`).

## 13. `http_server.py` — the 200 ms that wasn't buying anything

`post_req()` waited up to `t_expiry` for a reply on every request. Of all commands `rx.py` handles,
only `update` and `get_terminal_config` ever produce a reply; `set_freq`, `set_dwell_rid`, `capture`,
`set_freq_corr`… are fire-and-forget and *cannot* answer. Every tune call therefore burned ~200–205 ms
of dead time, subtracted directly from the post-retune settling window before the next burst. Now it only waits for
commands in `RESPONSE_CMDS`. A second fix coerces non-numeric `arg1/arg2` to `0.0` before they hit the
float-only message slots.

## 14. `rx.py` and the chaser support

`rx.py` is substantially larger now! This handles the dual-chaser control now. 

## 16. Environment variables (decoder)

| Variable | Default | Effect |
|---|---|---|
| `P25_SNDCP_DEBUG` | `0` | `1` prints every SNDCP/PDU diagnostic; otherwise only recovered text lines |
| `P25_ACTIVITY_PING` | off | `1` sends the 51001 decode-activity ping (needed by `dual_chaser.py`) |
| `P25_EXPORT_SALVAGE` | `0` | `1` also exports salvaged (unverified-RID) texts on 51003 |
| `P25_LEDGER` | on | `0` disables the `LEDGER` lines |


