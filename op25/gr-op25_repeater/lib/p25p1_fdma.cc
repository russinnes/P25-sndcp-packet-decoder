/* 
 * Copyright 2010, 2011, 2012, 2013, 2014 Max H. Parke KA1RBI 
 * Copyright 2017-2025 Graham J. Norbury
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 * This version is heavily modified for PDU decoding
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "p25p1_fdma.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <array>
#include <map>
#include <mutex>
#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "bch.h"
#include "op25_msg_types.h"
#include "op25_imbe_frame.h"
#include "p25_frame.h"
#include "p25_framer.h"
#include "rs.h"

/* v7: stderr verbosity. Default (P25_SNDCP_DEBUG unset/0): only recovered
 * text lines (TMS_TEXT*, SALVAGE_TMS_TEXT*) reach stderr. P25_SNDCP_DEBUG=1
 * (set by multi_rx.py --sndcp-debug 1) restores every diagnostic. */
static inline int sndcp_debug_on() {
    static int v = -1;
    if (v < 0) { const char* e = getenv("P25_SNDCP_DEBUG"); v = (e && atoi(e) > 0) ? 1 : 0; }
    return v;
}
#define DBGP(...) do { if (sndcp_debug_on()) fprintf(stderr, __VA_ARGS__); } while (0)
#define DBGC(c)   do { if (sndcp_debug_on()) fputc((c), stderr); } while (0)

#include <map>


namespace gr {
    namespace op25_repeater {



        /* Was 2.0s. Bumped to give longer RF fades / repeater-turnaround
         * gaps between confirmed-data bursts a chance to still be treated
         * as one continuous session rather than getting flushed and
         * re-started cold. Trade-off: a longer window also means two
         * genuinely separate bursts to the same RID are more likely to
         * get concatenated as if continuous -- watch REASM output for
         * structurally-wrong packets (bad IP version nibble, nonsense
         * ports) as the sign this has gone too far. */
        static const double REASM_WINDOW_SEC = 4.0;

        /* RidReasm is now declared in p25p1_fdma.h and lives as a member
         * (p25p1_fdma::reasm) instead of a file-scope global -- see the
         * comment on its declaration there. The free functions below
         * that used to touch a shared `g_reasm` directly now take the
         * caller's own RidReasm& instance explicitly. */

        static void reasm_reset(RidReasm& r)
        {
            r.rid = 0;
            r.last_ts = 0;
            r.last_ser = -1;
            r.buf.clear();
        }

        /* Classic hexdump -C style: offset, 16 hex bytes (gapped at 8),
         * ASCII alongside. Replaces the old single unbroken hex/ascii
         * line, which was also silently truncated at 64 bytes -- for a
         * 256-byte TMS message that meant most of it never printed
         * even in the "assembled" view, only scattered across the raw
         * per-block CONFIRMED lines. This covers the full length. */
        static void hexdump(const uint8_t* p, int n, const char* ts,
                            int msgq_id, uint32_t rid)
        {
            for (int off = 0; off < n; off += 16) {
                DBGP("%s [%d] HEX rid=%u %04x: ", ts, msgq_id, rid, off);
                for (int i = 0; i < 16; i++) {
                    if (off + i < n)
                        DBGP("%02x ", p[off + i]);
                    else
                        DBGP("   ");
                    if (i == 7)
                        DBGC(' ');
                }
                DBGP(" |");
                for (int i = 0; i < 16 && off + i < n; i++) {
                    uint8_t c = p[off + i];
                    DBGC((c >= 32 && c < 127) ? c : '.');
                }
                DBGP("|\n");
            }
        }

        /* Diagnostics counters, indexed by msgq_id & 15 (one p25p1_fdma per
         * fixed-frequency channel in multi_rx.py). Printed periodically as a
         * "STATS" line from load_nid(). Static file-scope so the .h is
         * unchanged. */
        static uint64_t g_stat_discard[16]   = {0};   /* unfinished reassembly abandoned */
        static uint64_t g_stat_supersede[16] = {0};   /* short burst replaced by a new datagram start */
        static uint64_t g_stat_nid_ok[16]    = {0};
        static uint64_t g_stat_nid_fail[16]  = {0};
        static double   g_stat_last_ts[16]   = {0};
        static uint64_t g_stat_pkt_ok[16]        = {0};   /* complete first copy, packet CRC32 ok */
        static uint64_t g_stat_pkt_fail[16]      = {0};   /* complete but packet CRC32 failed */
        static uint64_t g_stat_pkt_merged[16]    = {0};   /* completed/repaired by merging a retry */
        static uint64_t g_stat_pkt_incomplete[16]= {0};   /* first copy never completed (blocks missing) */
        static uint64_t g_stat_retry_noorig[16]  = {0};   /* retry seen, first transmission never seen */
        static uint64_t g_stat_d34_rescued    = 0;   /* Viterbi cost>8 but block CRC-9 passed: accepted */
        static uint64_t g_stat_d34_crc_bad    = 0;   /* cost<=8 but block CRC-9 failed (kept, as before) */
        static uint64_t g_stat_d34_crc_ok     = 0;   /* block CRC-9 passed (any cost) */

        /* ---- v5 diagnostics: "where did the downlink go?" ----------------
         * A per-channel ring of the last NID_RING frame-detection events
         * (NID ok / NID fail, DUID, NAC, frame length, and whether the frame
         * body was ever completed and processed), plus the last time a
         * downlink data PDU was decoded for each RID. When the tower ACKs an
         * uplink from a radio that has had no downlink decoded for a few
         * seconds ("ACK-NODL"), the ring is dumped so a missed transmission
         * leaves a trace: NID failure, a frame of another type, or a frame
         * that was detected but never completed. */
        struct NidEv { double ts; uint8_t ok; uint8_t duid; uint16_t nac; uint16_t fr_len; uint8_t processed; };
        static const int NID_RING = 64;
        static NidEv    g_nid_ring[16][NID_RING];
        static uint32_t g_nid_cnt[16] = {0};
        static uint64_t g_stat_duid[16][16] = {{0}};
        static uint64_t g_stat_nodl[16] = {0};
        static std::map<uint32_t, double> g_last_dl[16];
        /* how many body chunks / symbols load_body() was given for the frame
         * currently in progress, so a frame that is detected (NID ok) but never
         * completed can be reported with what it actually received. */
        static uint32_t g_body_calls[16] = {0};
        static uint64_t g_body_syms[16]  = {0};
        static uint64_t g_stat_abandoned[16] = {0};
        /* ---- : header-failed PDUs -------------------------------------
         * When a PDU's 12-byte header block fails CRC the burst has no RID, so
         * neither the log nor the TMS export can say which radio it was for.
         * The tower ACKs the radio ~0.5 s later and that ACK carries the RID,
         * so remember the failed PDU (and any text salvaged from its data
         * blocks) and link it to the next ACK on the same channel. */
        struct HdrFail  { double ts; uint32_t nblk; int linked; };
        struct SalvText { double ts; int valid; int verified; char text[600]; };
        static HdrFail  g_hdrfail[16];
        static SalvText g_salv_tmp[16];          /* filled by reasm_print_ip(salvage) */
        static SalvText g_salv[16];              /* stamped copy used for ACK linking */
        static int      g_salv_clean_next[16];   /* caller: every accepted block passed CRC-9 */
        static uint64_t g_stat_hdrfail[16]     = {0};
        static uint64_t g_stat_hdrfail_big[16] = {0};
        static uint64_t g_stat_attrib[16]      = {0};

        /* Pure counters (no behaviour change): classifies every PDU frame the
         * framer hands over, every IP datagram built from them, and every text printed,
         * so "frames detected" can be reconciled against named outcomes. */
        struct Ledger {
            uint64_t pdu_in, rc_fail, hf_small, hf_mid, hf_big;
            uint64_t resp[4];                 /* ACK, NACK, SELECTIVE_RETRY, reserved */
            uint64_t data_hdr, other_fmt;
            uint64_t ip_ok, ip_salv, ip_tiny, ip_nonudp, ip_short;
            uint64_t port[6];                 /* 4007>4, 4007<=4, 4005, 64414, other, (unused) */
            uint64_t text, text_salv, other_notext;
            uint32_t fs[32][64];              /* header fmt x sap histogram */
            uint64_t oport[8]; uint32_t oport_n; uint16_t oport_v[8]; /* distinct "other" dports */
        };
        static Ledger g_led[16];
        static bool ledger_enabled() {
            static int v = -1;
            if (v < 0) { const char* e = getenv("P25_LEDGER"); v = (e && atoi(e) == 0 && e[0] == '0') ? 0 : 1; }
            return v == 1;
        }
        static void led_other_port(Ledger& L, uint16_t dp) {
            for (uint32_t i = 0; i < L.oport_n; i++) if (L.oport_v[i] == dp) { L.oport[i]++; return; }
            if (L.oport_n < 8) { L.oport_v[L.oport_n] = dp; L.oport[L.oport_n++] = 1; }
        }
        static void ledger_print(int ci, const char* ts) {
            if (!ledger_enabled()) return;
            const Ledger& L = g_led[ci];
            uint64_t acct = L.rc_fail + L.hf_small + L.hf_mid + L.hf_big + L.resp[0] + L.resp[1] +
                            L.resp[2] + L.resp[3] + L.data_hdr + L.other_fmt;
            fprintf(stderr, "%s [%d] LEDGER pdu_in=%llu | rc_fail=%llu hdrfail(1-2blk)=%llu hdrfail(3-11blk)=%llu hdrfail(>=12blk,text-sized)=%llu | "
                    "resp ack=%llu nack=%llu retry=%llu other=%llu | data_hdr_ok=%llu other_fmt=%llu | unaccounted=%lld\n",
                    ts, ci, (unsigned long long)L.pdu_in, (unsigned long long)L.rc_fail,
                    (unsigned long long)L.hf_small, (unsigned long long)L.hf_mid, (unsigned long long)L.hf_big,
                    (unsigned long long)L.resp[0], (unsigned long long)L.resp[1], (unsigned long long)L.resp[2],
                    (unsigned long long)L.resp[3], (unsigned long long)L.data_hdr, (unsigned long long)L.other_fmt,
                    (long long)L.pdu_in - (long long)acct);
            fprintf(stderr, "%s [%d] LEDGER-IP frames_duid_pdu=%llu frames_abandoned=%llu ip_ok=%llu ip_salvage=%llu ip_tiny=%llu "
                    "ip_non_udp=%llu ip_short=%llu | port4007_text=%llu port4007_ctrl=%llu port4005=%llu port64414_beacon=%llu "
                    "port_other=%llu | texts=%llu texts_salvage=%llu other_port_no_text=%llu\n",
                    ts, ci, (unsigned long long)g_stat_duid[ci][0xc], (unsigned long long)g_stat_abandoned[ci],
                    (unsigned long long)L.ip_ok, (unsigned long long)L.ip_salv, (unsigned long long)L.ip_tiny,
                    (unsigned long long)L.ip_nonudp, (unsigned long long)L.ip_short,
                    (unsigned long long)L.port[0], (unsigned long long)L.port[1], (unsigned long long)L.port[2],
                    (unsigned long long)L.port[3], (unsigned long long)L.port[4],
                    (unsigned long long)L.text, (unsigned long long)L.text_salv, (unsigned long long)L.other_notext);
            char buf[900]; int n = 0; buf[0] = 0;
            for (int f = 0; f < 32; f++)
                for (int p = 0; p < 64; p++)
                    if (L.fs[f][p] && n < (int)sizeof(buf) - 40)
                        n += snprintf(buf + n, sizeof(buf) - n, " %02x:%d=%u", f, p, L.fs[f][p]);
            char pb[200]; int m = 0; pb[0] = 0;
            for (uint32_t i = 0; i < L.oport_n; i++)
                m += snprintf(pb + m, sizeof(pb) - m, " %u=%llu", L.oport_v[i], (unsigned long long)L.oport[i]);
            fprintf(stderr, "%s [%d] LEDGER-FMT fmt:sap=count (hdr-ok PDUs):%s | other_ports:%s\n", ts, ci, buf, pb);
        }
        static const double HDRFAIL_ACK_WIN = 1.2;   /* ACK must follow the failed PDU within this */
        static const uint32_t HDRFAIL_BIG_BLKS = 12; /* "text-sized": ~190+ bytes of payload */
        static bool export_salvage_enabled()
        {
            static int v = -1;
            if (v < 0) { const char* e = getenv("P25_EXPORT_SALVAGE"); v = (e && atoi(e) == 1) ? 1 : 0; }
            return v == 1;
        }
        /* IPv4 header checksum: ones-complement sum over the header must be 0xffff. */
        static bool ip_hdr_cksum_ok(const uint8_t* p, int ihl)
        {
            uint32_t sum = 0;
            for (int i = 0; i + 1 < ihl; i += 2) sum += ((uint32_t)p[i] << 8) | p[i + 1];
            while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
            return sum == 0xffff;
        }
        static const double NODL_AGE_SEC = 4.0;     /* no downlink for this long before the ACK */
        static const double NODL_LOOKBACK = 3.0;    /* how far back the ring dump looks */

        /* CRC-9 of a confirmed-data block (TIA-102.BAAA-A 6.4.2): 7-bit serial
         * number + 16 data octets, G9 = x^9+x^6+x^4+x^3+1, result inverted.
         * Verified against 1680/1680 real blocks. blk = 18 octets as decoded:
         * octet0 = serial<<1 | crc bit8, octet1 = crc bits 7..0. */
        static bool blk_crc9_ok(const uint8_t* blk)
        {
            uint32_t r = 0;
            auto feed = [&](int b) {
                uint32_t top = (r >> 8) & 1;
                r = ((r << 1) & 0x1ff) | (uint32_t)b;
                if (top) r ^= 0x059;
            };
            for (int i = 7; i >= 1; i--) feed((blk[0] >> i) & 1);
            for (int o = 2; o < 18; o++)
                for (int i = 7; i >= 0; i--) feed((blk[o] >> i) & 1);
            for (int i = 0; i < 9; i++) feed(0);
            r ^= 0x1ff;
            uint32_t rx = ((uint32_t)(blk[0] & 1) << 8) | blk[1];
            return r == rx;
        }

        /* Same local-integration pattern as tsbk_export (below), different
         * key/port so a consumer can tell the streams apart: real,
         * populated TMS text as JSON over UDP to 127.0.0.1:51003. Only
         * called for genuine content (same gate as the "*** TMS/TEXT
         * PAYLOAD ***" alert) -- the routine empty 4-byte poll and the
         * beacon never reach this, so this stays a high-signal feed
         * rather than getting flooded with placeholder traffic. */
        static void tms_export(const char* ts, uint32_t rid, int port, const char* text, bool partial = false, bool salvage = false)
        {
            int fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd < 0) return;
            struct sockaddr_in a;
            memset(&a, 0, sizeof(a));
            a.sin_family = AF_INET;
            a.sin_port = htons(51003);
            inet_aton("127.0.0.1", &a.sin_addr);
            char buf[600];
            int n = snprintf(buf, sizeof(buf),
                "{\"ts\":\"%s\",\"rid\":%u,\"port\":%d,\"partial\":%s,%s\"tms_text\":\"",
                ts, rid, port, partial ? "true" : "false",
                salvage ? "\"salvage\":true," : "");
            for (const char* p = text; *p && n < (int)sizeof(buf) - 4; p++) {
                if (*p == '"' || *p == '\\')
                    buf[n++] = '\\';
                buf[n++] = *p;
            }
            n += snprintf(buf + n, sizeof(buf) - n, "\"}");
            sendto(fd, buf, n, 0, (struct sockaddr*)&a, sizeof(a));
            close(fd);
        }

        /* optional "decode activity" ping for an external chaser (dual_chaser.py).
         * OFF unless P25_ACTIVITY_PING=1 (set by rx.py/multi_rx.py when asked); when on, every
         * header-valid PDU (data or response) sends {"rid":N,"sec":1} to 127.0.0.1:51001 -- the same
         * shape the old op25_hold_notify used, but local-only and not limited to one RID range.
         * Fire-and-forget UDP on a persistent socket; never blocks the decoder. */
        static void activity_ping(uint32_t rid)
        {
            static int on = -1;
            static int fd = -1;
            if (on < 0) { const char* e = getenv("P25_ACTIVITY_PING"); on = (e && atoi(e) > 0) ? 1 : 0; }
            if (!on) return;
            if (fd < 0) {
                fd = socket(AF_INET, SOCK_DGRAM, 0);
                if (fd < 0) { on = 0; return; }
            }
            struct sockaddr_in a;
            memset(&a, 0, sizeof(a));
            a.sin_family = AF_INET;
            a.sin_port = htons(51001);
            inet_aton("127.0.0.1", &a.sin_addr);
            char buf[64];
            int n = snprintf(buf, sizeof(buf), "{\"rid\":%u,\"sec\":1}", rid);
            sendto(fd, buf, n, MSG_DONTWAIT, (struct sockaddr*)&a, sizeof(a));
        }

        /* Shared by reasm_print_ip() and the implausible-iplen salvage
         * path in reasm_dump(): try UTF-16LE first (low byte, then a
         * 0x00 high byte) -- matches full free-text messages -- and
         * fall back to a plain single-byte ASCII scan only when that
         * comes back completely empty (matches short canned/status
         * messages with no null padding, e.g. "HOURS"). Always null-
         * terminates text_buf and returns the char count written;
         * *ascii_fallback reports which path was used. */
        static int extract_tms_text(const uint8_t* buf, int len, char* text_buf,
                                     size_t text_buf_size, bool* ascii_fallback)
        {
            *ascii_fallback = false;
            int tlen = 0;
            for (int i = 0; i + 1 < len; i += 2) {
                if (buf[i+1] == 0 && buf[i] >= 32 && buf[i] < 127) {
                    if ((size_t)tlen < text_buf_size - 1)
                        text_buf[tlen++] = (char)buf[i];
                }
            }
            /* Always scan plain ASCII too. Choose it when UTF-16LE found
             * nothing, OR when it found far fewer chars than the ASCII
             * scan (a plain-ASCII message can contain one accidental
             * "xx 00" pair at an even offset -- e.g. a trailing NUL
             * after a ':' -- that made the UTF-16 pass return 1 char and
             * block the fallback). For genuine UTF-16LE text the two
             * counts are nearly equal (the 0x00 bytes are skipped by the
             * ASCII scan), so those messages stay on the UTF-16 path.
             * A lone stray printable byte in an otherwise-binary buffer
             * is noise, so still require a real run (MIN_ASCII_FALLBACK_LEN). */
            {
                static const int MIN_ASCII_FALLBACK_LEN = 4;
                char ascii_buf[600];
                int ascii_tlen = 0;
                for (int i = 0; i < len; i++) {
                    if (buf[i] >= 32 && buf[i] < 127) {
                        if ((size_t)ascii_tlen < sizeof(ascii_buf) - 1 &&
                            (size_t)ascii_tlen < text_buf_size - 1)
                            ascii_buf[ascii_tlen++] = (char)buf[i];
                    }
                }
                if (ascii_tlen >= MIN_ASCII_FALLBACK_LEN &&
                    (tlen == 0 || ascii_tlen > 2 * tlen + 4)) {
                    memcpy(text_buf, ascii_buf, ascii_tlen);
                    tlen = ascii_tlen;
                    *ascii_fallback = true;
                }
            }
            text_buf[tlen] = '\0';
            return tlen;
        }

        /* Best-effort partial decode -- fires when reassembly is
         * provably incomplete (REASM wait) but there's already a
         * valid IP+UDP header and some real payload bytes sitting in
         * the buffer. REASM/TMS_TEXT proper only ever print once a
         * datagram is 100% complete (tot==have); that's correct for
         * the "is this exact number of bytes right" checks, but it
         * means a message that's genuinely 13 of 16 blocks in --
         * readable, useful, just missing its tail to a real Viterbi
         * failure on the last couple blocks -- was previously
         * invisible anywhere except the raw CONFIRMED blk hex. This
         * prints (and exports) whatever text IS there so far, clearly
         * labeled partial, and naturally gets called again with more
         * text each time a new burst adds to a still-incomplete
         * session -- a live, growing preview rather than all-or-
         * nothing. Deliberately a separate function from
         * reasm_print_ip rather than retrofitting it with a partial
         * mode, so the already-working complete-datagram path is
         * never at risk of being disturbed by this. */
        static void reasm_print_ip_partial(int msgq_id, const char* ts,
                                           const uint8_t* p, int avail,
                                           int declared_iplen, uint32_t rid)
        {
            if (avail < 20)
                return;
            uint8_t ihl = (p[0] & 0x0f) * 4;
            if (avail < ihl + 8)
                return;
            uint8_t proto = p[9];
            if (proto != 17)
                return;

            const uint8_t* u = p + ihl;
            uint16_t dport = (u[2]<<8)|u[3];
            int pay = avail - ihl - 8;
            if (pay <= 4)
                return;   /* nothing beyond the routine fixed-size replies yet */

            char text_buf[600];
            bool ascii_fb = false;
            int tlen = extract_tms_text(u + 8, pay, text_buf, sizeof(text_buf), &ascii_fb);
            if (tlen < 4)
                return;   /* not enough decoded so far to be worth printing */

            fprintf(stderr,
                    "%s [%d] *** TMS_TEXT_PARTIAL rid=%u have=%d/%d text=%s ***\n",
                    ts, msgq_id, rid, avail, declared_iplen, text_buf);

            /* 64414 is OTAR/KMM key-management traffic, never text. */
            if (dport != 64414)
                tms_export(ts, rid, dport, text_buf, true);   /* partial=true */
        }

        static void reasm_print_ip(int msgq_id, const char* ts,
                                   const uint8_t* p, int iplen, uint32_t rid,
                                   bool salvage = false)
        {
            /* salvage=true means this datagram was recovered from a burst
             * whose 12-byte header failed CRC -- rid may be an unverified
             * guess (or 0), and the blocks that built this buffer were
             * accepted on Viterbi cost + shape alone, not a session we
             * were already tracking. Tag every line so this never reads
             * as an ordinary, fully-verified decode, and never hand it to
             * tms_export() -- that's for verified content only. */
            const char* tag = salvage ? "SALVAGE_" : "";
            Ledger& LD = g_led[msgq_id & 15];
            if (iplen < 20) {
                LD.ip_tiny++;
                return;
            }
            if (salvage) LD.ip_salv++; else LD.ip_ok++;

            uint8_t ihl = (p[0] & 0x0f) * 4;
            uint16_t tot = (p[2] << 8) | p[3];
            uint8_t proto = p[9];
            /* v6: the reassembled buffer ends on a whole 16-byte block, so it can run
             * past the datagram (f5 filler / pad). Trim to the IP total length so the
             * UDP payload and any text are exactly what the sender put on the air. */
            const int have_raw = iplen;
            const bool ipck_ok = (ihl >= 20 && (int)ihl <= iplen) ? ip_hdr_cksum_ok(p, ihl) : false;
            if (tot >= (uint16_t)(ihl + 8) && (int)tot < iplen)
                iplen = tot;
            uint32_t src = (p[12]<<24)|(p[13]<<16)|(p[14]<<8)|p[15];
            uint32_t dst = (p[16]<<24)|(p[17]<<16)|(p[18]<<8)|p[19];

            DBGP("%s [%d] %sIP rid=%u ver=%u ihl=%u tot=%u have=%d proto=0x%02x "
                    "%u.%u.%u.%u -> %u.%u.%u.%u\n",
                    ts, msgq_id, tag, rid,
                    p[0] >> 4, ihl, tot, iplen, proto,
                    (src>>24)&0xff, (src>>16)&0xff, (src>>8)&0xff, src&0xff,
                    (dst>>24)&0xff, (dst>>16)&0xff, (dst>>8)&0xff, dst&0xff);

            if (salvage)
                DBGP("%s [%d] SALVAGE_IPCK rid=%u ipck=%s raw_len=%d tot=%u\n",
                        ts, msgq_id, rid, ipck_ok ? "ok" : "BAD", have_raw, tot);

            if (tot > iplen) LD.ip_short++;
            if (tot > iplen)
                DBGP("%s [%d] %sIP SHORT rid=%u need=%u have=%d missing=%d\n",
                        ts, msgq_id, tag, rid, tot, iplen, tot - iplen);

            if (proto != 17 || iplen < ihl + 8) {
                LD.ip_nonudp++;
                return;
            }

            const uint8_t* u = p + ihl;
            uint16_t sport = (u[0]<<8)|u[1];
            uint16_t dport = (u[2]<<8)|u[3];
            uint16_t ulen  = (u[4]<<8)|u[5];
            int pay = iplen - ihl - 8;
            if (pay < 0) pay = 0;
            if (ulen >= 8 && (int)ulen - 8 < pay)
                pay = (int)ulen - 8;     /* v6: never report bytes beyond the UDP length */
            if (!salvage) {
                if (dport == 4007) LD.port[pay > 4 ? 0 : 1]++;
                else if (dport == 4005) LD.port[2]++;
                else if (dport == 64414) LD.port[3]++;
                else { LD.port[4]++; led_other_port(LD, dport); }
            }

            DBGP("%s [%d] %sUDP rid=%u %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u "
                    "udp_len=%u payload=%d\n",
                    ts, msgq_id, tag, rid,
                    (src>>24)&0xff, (src>>16)&0xff, (src>>8)&0xff, src&0xff, sport,
                    (dst>>24)&0xff, (dst>>16)&0xff, (dst>>8)&0xff, dst&0xff, dport,
                    ulen, pay);

            if (ulen > 8 && (ulen - 8) > pay)
                DBGP("%s [%d] %sUDP SHORT rid=%u udp_payload=%u have=%d\n",
                        ts, msgq_id, tag, rid, ulen - 8, pay);

            /* Port 4007 is Motorola TMS (
             * Motorola LRRP/ARS/TMS case-study docs). ! 4-byte
             * "mailbox empty" reply. Anything bigger is a real, populated
             * text and worth a loud, greppable line of its own rather
             * than blending into routine UDP output. */
            if (dport == 4007 && pay > 4)
                DBGP("%s [%d] *** %sTMS PAYLOAD rid=%u len=%d (port 4007, normally 4) ***\n",
                        ts, msgq_id, tag, rid, pay);
            /* Mirror for port 4005 (ARS) -- always 3-byte fixed in
             * every capture so far, so this is precautionary rather
             * than fixing an observed miss. Costs nothing to cover it
             * in case  content ever lands there too. */
            if (dport == 4005 && pay > 3)
                DBGP("%s [%d] *** %sARS PAYLOAD rid=%u len=%d (port 4005, normally 3) ***\n",
                        ts, msgq_id, tag, rid, pay);

            /* Port 64414 keepalive: always carries the sender's own RID
             * at a fixed offset in the payload. Skip
             * the full hexdump for this known-shape traffic; one
             * compact line is enough, and it stays out of the way of
             * anything actually worth reading by eye. */
            if (dport == 64414 && pay >= 21) {
                /* Beacon/keepalive shape -- routine self-identifying
                 * traffic, never a real message. In salvage mode this is
                 * exactly the "not worth the noise" case: no rid to
                 * cross-check it against anyway (that's the whole reason
                 * we're here), so skip the line entirely rather than
                 * print a MISMATCH that means nothing. */
                if (salvage)
                    return;
                uint32_t self_id = ((uint32_t)u[8+18] << 16) |
                                    ((uint32_t)u[8+19] << 8)  |
                                     (uint32_t)u[8+20];
                DBGP("%s [%d] BEACON rid=%u self_id=%u %s\n",
                        ts, msgq_id, rid, self_id,
                        (self_id == rid) ? "ok" : "*** MISMATCH ***");
                return;
            }

            DBGP("%s [%d] %sHEXDUMP rid=%u len=%d:\n", ts, msgq_id, tag, rid, pay);
            hexdump(u + 8, pay, ts, msgq_id, rid);

            /* Space-separated, fixed fields first so this is easy to
             * awk/parse downstream: timestamp, msgq, a stable tag, then
             * rid=, then the free-text message to end of line. Grep for
             * "TMS_TEXT" instead of the old "utf16:" label. Builds a
             * plain buffer alongside the printed version so it can also
             * go out over tms_export() for genuine content.
             *
             * Two different TMS payload encodings show up in practice.
             * Full free-text messages come across as UTF-16LE -- each
             * printable char followed by a 0x00 high byte. Rather than
             * guess which encoding a given payload uses, try UTF-16LE
             * first (unchanged behavior for every message that already
             * decodes correctly), and only fall back to a plain-ASCII
             * byte scan when that pass comes back completely empty. */
            char text_buf[600];
            bool ascii_fallback = false;
            int tms_tlen = extract_tms_text(u + 8, pay, text_buf, sizeof(text_buf), &ascii_fallback);
            if (tms_tlen > 0) { if (salvage) LD.text_salv++; else LD.text++; }
            else if (dport != 4007 && dport != 4005) LD.other_notext++;
            if ((dport == 4007 || dport == 4005) && pay <= 4) {
                /* 3-4 byte datagrams from the gateway are binary control
                 * packets, not text. 00 01 3f on 4005 and 00 02 70 00 on 4007
                 */
                const uint8_t* pl = u + 8;
                const char* kind = "SHORT_CTRL";
                if (dport == 4005 && pay == 3 && pl[0] == 0x00 && pl[1] == 0x01) kind = "ARS_QUERY";
                else if (dport == 4007 && pay == 4 && pl[0] == 0x00 && pl[1] == 0x02) kind = "TMS_CTRL";
                DBGP("%s [%d] %sSHORT_CTRL rid=%u port=%u kind=%s bytes=",
                        ts, msgq_id, tag, rid, (unsigned)dport, kind);
                for (int i = 0; i < pay; i++)
                    DBGP("%s%02x", i ? " " : "", pl[i]);
                if (kind[0] == 'T') DBGP(" op=0x%02x", pl[2]);
                DBGP("\n");
            } else if (sndcp_debug_on() || tms_tlen > 0)
            fprintf(stderr, "%s [%d] %sTMS_TEXT rid=%u text=%s%s\n",
                    ts, msgq_id, tag, rid, text_buf,
                    ascii_fallback ? " (ascii fallback)" : "");

            /* remember a salvaged text so the next ACK on this channel can name the radio. */
            if (salvage && dport == 4007 && pay > 4) {
                int sci = msgq_id & 15;
                g_salv_tmp[sci].valid = 1;
                g_salv_tmp[sci].verified = (g_salv_clean_next[sci] && ipck_ok && have_raw >= (int)tot) ? 1 : 0;
                snprintf(g_salv_tmp[sci].text, sizeof(g_salv_tmp[sci].text), "%s", text_buf);
            }

            /* Mirrors the ARS PAYLOAD gate above -- 4007 is the normal
             * TMS path, 4005 covered defensively in case content ever
             * lands there too. Salvage content never reaches tms_export()
             * -- that path is for content we actually trust (a verified
             * header, or a live session we were already tracking), and
             * rid may be an unverified guess or 0 here. */
            if (!salvage && ((dport == 4007 && pay > 4) || (dport == 4005 && pay > 3) ||
                             (dport != 4007 && dport != 4005 && dport != 64414 && tms_tlen >= 4)))
                tms_export(ts, rid, dport, text_buf);
        }

        static void reasm_dump(RidReasm& r, int msgq_id, const char* ts)
        {
            if (r.buf.empty())
                return;
            for (;;) {
                uint8_t* p = r.buf.data();
                size_t n = r.buf.size();
                int off = -1;
                for (size_t i = 0; i + 20 <= n; i++) {
                    if (p[i] == 0x45) { off = (int)i; break; }
                }
                if (off < 0) {
                    if (n >= 8)
                        DBGP("%s [%d] REASM raw rid=%u leftover=%zu\n",
                                ts, msgq_id, r.rid, n);
                    break;
                }
                int tot = (p[off+2] << 8) | p[off+3];
                if (tot < 20)
                    break;

                /* Cross-check the IP header's declared total length (tot)
                 * against the UDP header's own length field (udp_len) --
                 * two independently-decoded fields that must agree if
                 * both decoded cleanly. A single bit error landing in
                 * just the 2 tot bytes was otherwise silently accepted
                 * as-is, and since same-RID retries within
                 * REASM_WINDOW_SEC get concatenated into one buffer, a
                 * too-large tot reads straight past the real end of this
                 * datagram and into the start of whatever's next --
                 * observed in the field as one message's real tail glued
                 * directly onto the next message's start (e.g. a genuine ending
                 * immediately followed by the fixed prefix that starts
                 * every real message). When they disagree, prefer the
                 * udp_len-derived length: not guaranteed correct either,
                 * but it's the one Motorola's stack actually needs right
                 * to deliver the page at all, and the mismatch is now
                 * logged loudly (grep "LEN MISMATCH") so these are
                 * visible instead of silently corrupting output. */
                /* Real datagrams on this system have never exceeded ~200
                 * bytes (TMS/SNDCP text, BEACON keepalives, etc.). Blocks
                 * are fixed-size (16 payload bytes each) and any unused
                 * tail of an already-CONFIRMED block is padded with 0xF5
                 * filler -- visible throughout these logs as trailing
                 * "f5 f5 f5 ..." runs -- not zeroed and not flagged as
                 * "not yet real data". If the UDP length field (u[4],u[5])
                 * happens to land on that filler, 0xF5F5 decodes as
                 * udp_len=62965, giving derived=ihl+62965=62985 -- an
                 * exact, reproducible value (confirmed hitting this same
                 * 62985 on two independent receivers/RIDs). Uncapped, the
                 * "prefer derived length" logic below then latches onto
                 * that phantom 63KB datagram and holds the reassembly
                 * open (REASM wait / repeated HOLD REQUEST renewals) for
                 * minutes waiting for a message that was never coming,
                 * instead of the far shorter real one already in hand. */
                static const int MAX_PLAUSIBLE_IPLEN = 1500;
                int iplen = tot;
                uint8_t ihl = (p[off] & 0x0f) * 4;
                uint8_t proto = p[off+9];
                if (proto == 17 && (size_t)(off + ihl + 8) <= n) {
                    const uint8_t* u = p + off + ihl;
                    uint16_t udp_len = (u[4] << 8) | u[5];
                    int derived = ihl + udp_len;
                    if (udp_len >= 8 && derived != tot) {
                        if (derived <= MAX_PLAUSIBLE_IPLEN) {
                            DBGP("%s [%d] REASM LEN MISMATCH rid=%u tot=%d ihl+udp_len=%d "
                                    "(ihl=%u udp_len=%u) -- using derived length\n",
                                    ts, msgq_id, r.rid, tot, derived, ihl, udp_len);
                            iplen = derived;
                        } else {
                            DBGP("%s [%d] REASM LEN MISMATCH rid=%u tot=%d ihl+udp_len=%d "
                                    "(ihl=%u udp_len=%u) -- derived length implausible "
                                    "(likely unfilled 0xF5 padding), keeping tot=%d instead\n",
                                    ts, msgq_id, r.rid, tot, derived, ihl, udp_len, tot);
                        }
                    }
                }
                if (iplen > MAX_PLAUSIBLE_IPLEN) {
                    DBGP("%s [%d] REASM rid=%u implausible iplen=%d (>%d) -- treating as "
                            "corrupt header, discarding session instead of waiting\n",
                            ts, msgq_id, r.rid, iplen, MAX_PLAUSIBLE_IPLEN);

                    /* Try
                     * a direct text scan on the raw buffer, skipping the
                     * IP/UDP length fields entirely rather than trusting
                     * them to find the payload -- same UTF-16LE/ASCII
                     * extraction used everywhere else. */
                    bool have_udp = proto == 17 && (size_t)(off + ihl + 8) <= n;
                    const uint8_t* udp_payload = have_udp ? p + off + ihl + 8 : nullptr;
                    int udp_payload_avail = have_udp ? (int)(n - (off + ihl + 8)) : 0;

                    if (r.buf.size() >= 20) {
                        char text_buf[600];
                        bool ascii_fallback = false;
                        int tlen;

                        /* Prefer scanning from right after the IP+UDP
                         * header -- ihl/proto are reliable (separate
                         * bytes from the length field that lied), and
                         * this avoids the IP header's OWN leading bytes
                         * decoding as bogus text. Only fall back to the raw
                         * buffer when there's no safe structured offset
                         * to start from at all. */
                        if (have_udp)
                            tlen = extract_tms_text(udp_payload, udp_payload_avail,
                                                     text_buf, sizeof(text_buf),
                                                     &ascii_fallback);
                        else
                            tlen = extract_tms_text(p, (int)n, text_buf, sizeof(text_buf),
                                                     &ascii_fallback);

                        if (tlen > 0) {
                            fprintf(stderr,
                                    "%s [%d] SALVAGE_TMS_TEXT rid=%u (iplen was implausible -- "
                                    "length field untrusted, scanned directly instead) "
                                    "text=%s%s\n",
                                    ts, msgq_id, r.rid, text_buf,
                                    ascii_fallback ? " (ascii fallback)" : "");

                            /* dport is two bytes entirely separate from
                             * the length field. Export it the
                             * same way a normal decode would, so this
                             * doesn't just sit in the log file --
                             * partial=true since this bypassed the length
                             * math rather than confirming the datagram
                             * was byte-exact. */
                            if (have_udp) {
                                const uint8_t* u = p + off + ihl;
                                uint16_t dport = (u[2] << 8) | u[3];
                                if (dport == 4007 || dport == 4005 ||
                                    (dport != 64414 && tlen >= 4))
                                    tms_export(ts, r.rid, dport, text_buf, true);
                            }
                        }
                    }

                    r.buf.clear();
                    r.rid = 0;
                    r.last_ser = -1;
                    break;
                }
                if (iplen < 20)
                    break;
                if (off + iplen > (int)n) {
                    int have = (int)n - off;
                    DBGP("%s [%d] REASM wait rid=%u have=%d need=%d (more blocks expected)\n",
                            ts, msgq_id, r.rid, have, iplen);
                    reasm_print_ip_partial(msgq_id, ts, p + off, have, iplen, r.rid);
                    break;
                }

                DBGP("%s [%d] REASM rid=%u len=%d\n",
                        ts, msgq_id, r.rid, iplen);
                reasm_print_ip(msgq_id, ts, p + off, iplen, r.rid);

                r.buf.erase(r.buf.begin(),
                            r.buf.begin() + off + iplen);
            }
            if (r.buf.size() < 20)
                r.buf.clear();
            if (r.buf.empty()) {
                r.rid = 0;
                r.last_ser = -1;
            }
        }

        /*  If anything is still sitting in the buffer after the final dump 
         * attempt, an unfinished (or unparseable) datagram is being abandoned */
        static void reasm_flush(RidReasm& r, int msgq_id, const char* ts,
                                const char* reason = "unspecified")
        {
            reasm_dump(r, msgq_id, ts);
            if (r.buf.size() >= 20) {
                g_stat_discard[msgq_id & 15]++;
                DBGP("%s [%d] REASM DISCARD rid=%u bytes=%zu reason=%s "
                        "(unfinished datagram abandoned)\n",
                        ts, msgq_id, r.rid, r.buf.size(), reason);
            }
            reasm_reset(r);
        }

        static void reasm_burst(RidReasm& r, int msgq_id, const char* ts, uint32_t rid,
                                double now, const uint8_t d34[][18], int n34)
        {
            if (n34 <= 0)
                return;

            bool same_session = (r.rid == rid) &&
                                 ((now - r.last_ts) < REASM_WINDOW_SEC);

            /* Reject only an exact duplicate of the last block already
             * accepted for this RID -- an air retry of the same block.
             */
            if (same_session && d34[0][0] == (uint8_t)r.last_ser) {
                DBGP("%s [%d] REASM dup rid=%u ser=%u -- ignored (air retry)\n",
                        ts, msgq_id, rid, d34[0][0]);
                return;
            }

            if (!same_session && !r.buf.empty()) {
                char why[64];
                if (r.rid != rid)
                    snprintf(why, sizeof(why), "different_rid(new=%u)", rid);
                else
                    snprintf(why, sizeof(why), "session_cold");
                reasm_flush(r, msgq_id, ts, why);   /* different RID, or session gone cold */
            } else if (same_session && !r.buf.empty() &&
                       d34[0][2] == 0x51 && d34[0][3] == 0x00 && (d34[0][4] >> 4) == 4) {
                /* The buffer still holds an unfinished datagram (reasm_dump
                 * clears it whenever a datagram completes), yet this burst's
                 * first block starts with an SNDCP header (51 00) followed by
                 * an IPv4 header -- i.e. a NEW datagram (typically a
                 * retransmission of the same short burst), not a
                 * continuation. Appending it used to glue the new burst's
                 * "51 00" onto the old datagram's missing tail, producing
                 * the bogus "00 02 51 00" ('Q') replies. Close the old one as
                 * unfinished and start fresh. */
                g_stat_supersede[msgq_id & 15]++;
                reasm_flush(r, msgq_id, ts, "superseded_by_new_datagram");
            }

            r.rid = rid;
            r.last_ts = now;

            for (int i = 0; i < n34; i++) {
                r.buf.insert(r.buf.end(), d34[i] + 2, d34[i] + 18);
                r.last_ser = d34[i][0];
            }

            reasm_dump(r, msgq_id, ts);
        }

        /* ---- Packet-level assembler (TIA-102.BAAA-A 6.3-6.4, TSB102.BAAD 8-9) ----
         *
         * Keyed by (LLID, N(S), FSNF) and shared across all channels (retries
         * for a message can arrive on a different data channel than the
         * first copy), so it is mutex-protected. Blocks are placed by their
         * serial number, so a failed/missing block leaves a hole instead of
         * shifting everything after it, and a selective retry (FMF=0)
         * carrying only the blocks the radio missed can fill the holes in a
         * first copy we only partly heard. The 32-bit packet CRC in the last
         * block verifies the whole message once every block is present.
         *
         * This is additive: the existing reasm path still prints every
         * message exactly as before. This assembler (a) counts/logs first
         * copies that complete and pass or fail the packet CRC, (b) logs
         * first copies that never complete, (c) logs retries whose first
         * transmission we never saw, and (d) emits a message through
         * reasm_print_ip() ONLY when a retry is what completed (or repaired)
         * it -- a case the old path could not handle. */
        static uint32_t crc32_p25(const uint8_t* d, int len)
        {
            /* G(x)=x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1,
             * MSB first, init 0, output inverted (verified on ~1450 real
             * packets: 100% of complete first copies match). */
            uint32_t c = 0;
            for (int i = 0; i < len; i++) {
                c ^= (uint32_t)d[i] << 24;
                for (int k = 0; k < 8; k++)
                    c = (c & 0x80000000u) ? ((c << 1) ^ 0x04C11DB7u) : (c << 1);
            }
            return ~c;
        }

        struct PktAsm {
            uint8_t  blk[128][16];
            bool     have[128];
            int      expected;     /* blocks to follow from the FMF=1 header */
            int      pad;
            int      sap;
            int      dho;
            int      nhave;
            bool     done;         /* all blocks present and CRC evaluated */
            bool     crc_ok;
            uint32_t llid;
            int      msgq_id;
            double   last_ts;
        };
        static std::mutex g_pkt_mu;
        static std::map<uint64_t, PktAsm> g_pkt;
        static const double PKT_WINDOW_SEC = 30.0;

        static bool pkt_crc_ok(const PktAsm& p)
        {
            uint8_t data[128 * 16];
            int len = p.expected * 16;
            if (p.expected < 1 || len > (int)sizeof(data) || len < 5)
                return false;
            for (int i = 0; i < p.expected; i++)
                memcpy(data + i * 16, p.blk[i], 16);
            uint32_t want = ((uint32_t)data[len-4] << 24) | ((uint32_t)data[len-3] << 16) |
                            ((uint32_t)data[len-2] << 8)  |  (uint32_t)data[len-1];
            return crc32_p25(data, len - 4) == want;
        }

        static void pktasm_burst(int msgq_id, const char* ts, double now,
                                 const uint8_t* hdr, const uint8_t d34[][18], int n34,
                                 const bool* bad34 = nullptr)
        {
            int      sap  = hdr[1] & 0x3f;
            uint32_t llid = ((uint32_t)hdr[3] << 16) | ((uint32_t)hdr[4] << 8) | hdr[5];
            bool     fmf  = (hdr[6] & 0x80) != 0;
            int      blks = hdr[6] & 0x7f;
            int      pad  = hdr[7] & 0x1f;
            int      ns   = (hdr[8] >> 4) & 7;
            int      fsnf = hdr[8] & 0x0f;
            int      dho  = hdr[9] & 0x3f;
            int      ci   = msgq_id & 15;
            if (blks < 1 || blks > 127 || n34 < 0 || n34 > 32)
                return;

            uint64_t key = ((uint64_t)llid << 8) | (uint64_t)(ns << 4) | (uint64_t)fsnf;
            std::vector<uint8_t> emit_ip;
            bool do_emit = false;
            int  emit_blocks = 0;

            {
                std::lock_guard<std::mutex> lk(g_pkt_mu);

                for (auto it = g_pkt.begin(); it != g_pkt.end();) {
                    if (now - it->second.last_ts > PKT_WINDOW_SEC) {
                        if (!it->second.done) {
                            g_stat_pkt_incomplete[it->second.msgq_id & 15]++;
                            DBGP("%s [%d] PKT INCOMPLETE llid=%u sap=%d have=%d/%d blocks "
                                    "(first copy never completed, no retry filled the gap)\n",
                                    ts, it->second.msgq_id, it->second.llid, it->second.sap,
                                    it->second.nhave, it->second.expected);
                        }
                        it = g_pkt.erase(it);
                    } else {
                        ++it;
                    }
                }
                if (g_pkt.size() > 512)
                    g_pkt.clear();

                auto it = g_pkt.find(key);
                if (fmf) {
                    PktAsm& np = g_pkt[key];
                    memset(&np, 0, sizeof(np));
                    np.expected = blks;
                    np.pad      = pad;
                    np.sap      = sap;
                    np.dho      = dho;
                    np.llid     = llid;
                    np.msgq_id  = msgq_id;
                    np.last_ts  = now;
                    it = g_pkt.find(key);
                } else if (it == g_pkt.end()) {
                    g_stat_retry_noorig[ci]++;
                    DBGP("%s [%d] PKT RETRY-WITHOUT-ORIGINAL llid=%u ns=%d sap=%d retry_blocks=%d "
                            "(first transmission of this message was never seen)\n",
                            ts, msgq_id, llid, ns, sap, blks);
                    return;
                }
                PktAsm& p = it->second;
                if (p.done && p.crc_ok)
                    return;                       /* already complete and verified */
                p.last_ts = now;

                for (int i = 0; i < n34; i++) {
                    int serial = d34[i][0] >> 1;
                    if (serial >= p.expected)
                        continue;
                    if (bad34 && bad34[i]) {
                        /* block CRC-9 failed. The legacy path still prints it, but it
                         * must not fill a slot -- leave the slot empty so a selective
                         * retry can supply a good copy and the merge can complete. */
                        DBGP("%s [%d] PKT blk serial=%d CRC9 BAD -- not used for assembly (waiting for retry)\n",
                                ts, msgq_id, serial);
                        continue;
                    }
                    if (!p.have[serial]) {
                        memcpy(p.blk[serial], d34[i] + 2, 16);
                        p.have[serial] = true;
                        p.nhave++;
                    }
                }

                if (p.nhave == p.expected) {
                    bool ok = pkt_crc_ok(p);
                    if (!p.done) {
                        p.done   = true;
                        p.crc_ok = ok;
                        if (ok && fmf) {
                            g_stat_pkt_ok[ci]++;      /* normal case, old path already printed it */
                        } else if (ok) {
                            g_stat_pkt_merged[ci]++;
                            do_emit = true;
                        } else {
                            g_stat_pkt_fail[ci]++;
                            DBGP("%s [%d] PKT CRC FAIL llid=%u ns=%d sap=%d blocks=%d "
                                    "(all blocks present but packet CRC32 does not match)\n",
                                    ts, msgq_id, llid, ns, p.sap, p.expected);
                        }
                    } else if (ok) {                  /* was a CRC failure, a retry repaired it */
                        p.crc_ok = true;
                        g_stat_pkt_merged[ci]++;
                        do_emit = true;
                    }
                    if (do_emit) {
                        int userlen = p.expected * 16 - 4 - p.pad;
                        int iplen   = userlen - p.dho;
                        if (p.sap == 0 && p.dho >= 0 && p.dho <= 4 && iplen >= 20 &&
                            p.dho + iplen <= p.expected * 16) {
                            uint8_t data[128 * 16];
                            for (int i = 0; i < p.expected; i++)
                                memcpy(data + i * 16, p.blk[i], 16);
                            emit_ip.assign(data + p.dho, data + p.dho + iplen);
                            emit_blocks = p.expected;
                        } else {
                            do_emit = false;
                        }
                    }
                }
            }

            if (do_emit) {
                DBGP("%s [%d] PKT MERGED llid=%u ns=%d blocks=%d (completed by merging a retry; "
                        "packet CRC32 ok)\n", ts, msgq_id, llid, ns, emit_blocks);
                reasm_print_ip(msgq_id, ts, emit_ip.data(), (int)emit_ip.size(), llid);
            }
        }

        /* Fire-and-forget TSBK export to a local integration point --
         * every TSBK that passes CRC, raw and unframed: byte 0 = opcode,
         * byte 1 = last-block flag, bytes 2-13 = the raw 12-byte TSBK.
         * 14 bytes total, no JSON, nothing to parse but fixed offsets.
         * NAC isn't included */
        static void tsbk_export(const char* ts, int nac, uint8_t op,
                                uint8_t lb, const uint8_t* raw12)
        {
            (void)ts; (void)nac;   /* kept in the signature so call sites don't change */
            int fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd < 0) return;
            struct sockaddr_in a;
            memset(&a, 0, sizeof(a));
            a.sin_family = AF_INET;
            a.sin_port = htons(51002);
            inet_aton("127.0.0.1", &a.sin_addr);
            uint8_t buf[14];
            buf[0] = op;
            buf[1] = lb;
            memcpy(buf + 2, raw12, 12);
            sendto(fd, buf, sizeof(buf), 0, (struct sockaddr*)&a, sizeof(a));
            close(fd);
        }

        static void dump_pdu_burst(const char* ts, uint32_t rid, uint32_t fr_len,
               const uint8_t* hdr12,
               const bit_vector& bv, int bl_len,
               const uint8_t blks[][18], int n34)
            {
                char fn[128];
                snprintf(fn, sizeof(fn), "/tmp/pdu_%u_%s.bin", rid, ts);
                (void)fn;   /* built but intentionally unused -- see below */
                /* sanitize ts if it has spaces */
                /* "wb" (truncate), not "ab" (append): this was appending
                 * forever with no rotation or cap, growing unbounded for
                 * as long as the process runs. The filename itself
                 * ("pdu_last.bin") implies the original intent was
                 * "most recent capture only" -- truncating on each write
                 * matches that and removes the disk-growth risk entirely. */
                FILE* f = fopen("/tmp/pdu_last.bin", "wb");
                if (!f) return;
                fprintf(f, "TS %s rid=%u fr_len=%u bl_len=%d n34=%d\n",
                        ts, rid, fr_len, bl_len, n34);
                fprintf(f, "HDR");
                for (int i = 0; i < 12; i++) fprintf(f, " %02x", hdr12[i]);
                fprintf(f, "\n");
                for (int b = 1; b < bl_len; b++) {
                    unsigned off = 48 + 64 + b * 196;
                    fprintf(f, "RAW%d ", b);
                    for (unsigned i = 0; i < 196 && off + i < bv.size(); i++)
                        fputc(bv[off + i] ? '1' : '0', f);
                    fprintf(f, "\n");
                    if (b <= n34) {
                        fprintf(f, "D34%d", b);
                        for (int i = 0; i < 18; i++) fprintf(f, " %02x", blks[b - 1][i]);
                        fprintf(f, "\n");
                    }
                }
                fputc('\n', f);
                fclose(f);
            }
        
        
        static const int64_t TIMEOUT_THRESHOLD = 1000000;

        p25p1_fdma::~p25p1_fdma()
        {
            delete framer;
        }

        static uint16_t crc16(uint8_t buf[], int len) {
            if (buf == 0)
                return -1;
            uint32_t poly = (1<<12) + (1<<5) + (1<<0);
            uint32_t crc = 0;
            for(int i=0; i<len; i++) {
                uint8_t bits = buf[i];
                for (int j=0; j<8; j++) {
                    uint8_t bit = (bits >> (7-j)) & 1;
                    crc = ((crc << 1) | bit) & 0x1ffff;
                    if (crc & 0x10000)
                        crc = (crc & 0xffff) ^ poly;
                }
            }
            crc = crc ^ 0xffff;
            return crc & 0xffff;
        }

        /* translated from p25craft.py Michael Ossmann <mike@ossmann.com>  */
        static uint32_t crc32(uint8_t buf[], int len) {	/* length is nr. of bits */
            uint32_t g = 0x04c11db7;
            uint64_t crc = 0;
            for (int i = 0; i < len; i++) {
                crc <<= 1;
                int b = ( buf [i / 8] >> (7 - (i % 8)) ) & 1;
                if (((crc >> 32) ^ b) & 1)
                    crc ^= g;
            }
            crc = (crc & 0xffffffff) ^ 0xffffffff;
            return crc;
        }

        /* find_min is from wireshark/plugins/p25/packet-p25cai.c */
        /* Copyright 2008, Michael Ossmann <mike@ossmann.com>  */
        /* return the index of the lowest value in a list */
        static int find_min(uint8_t list[], int len) {
            int min = list[0];	
            int index = 0;	
            int unique = 1;	
            int i;

            for (i = 1; i < len; i++) {
                if (list[i] < min) {
                    min = list[i];
                    index = i;
                    unique = 1;
                } else if (list[i] == min) {
                    unique = 0;
                }
            }
            /* return -1 if a minimum can't be found */
            if (!unique)
                return -1;

            return index;
        }

        /* count_bits is from wireshark/plugins/p25/packet-p25cai.c */
        /* Copyright 2008, Michael Ossmann <mike@ossmann.com>  */
        /* count the number of 1 bits in an int */
        static int count_bits(unsigned int n) {
            int i = 0;
            for (i = 0; n != 0; i++)
                n &= n - 1;
            return i;
        }

        /* adapted from wireshark/plugins/p25/packet-p25cai.c */
        /* Copyright 2008, Michael Ossmann <mike@ossmann.com>  */
        /* deinterleave and trellis1_2 decoding */
        /* buf is assumed to be a buffer of 12 bytes */
        static int block_deinterleave(bit_vector& bv, unsigned int start, uint8_t* buf) {
            if ((size_t)start + 195 >= bv.size()) {   /* v8.2: stock code reads past the end on short frames */
                memset(buf, 0, 12);
                return -1;
            }
            static const uint16_t deinterleave_tb[] = {
                0,  1,  2,  3,  52, 53, 54, 55, 100,101,102,103, 148,149,150,151,
                4,  5,  6,  7,  56, 57, 58, 59, 104,105,106,107, 152,153,154,155,
                8,  9, 10, 11,  60, 61, 62, 63, 108,109,110,111, 156,157,158,159,
                12, 13, 14, 15,  64, 65, 66, 67, 112,113,114,115, 160,161,162,163,
                16, 17, 18, 19,  68, 69, 70, 71, 116,117,118,119, 164,165,166,167,
                20, 21, 22, 23,  72, 73, 74, 75, 120,121,122,123, 168,169,170,171,
                24, 25, 26, 27,  76, 77, 78, 79, 124,125,126,127, 172,173,174,175,
                28, 29, 30, 31,  80, 81, 82, 83, 128,129,130,131, 176,177,178,179,
                32, 33, 34, 35,  84, 85, 86, 87, 132,133,134,135, 180,181,182,183,
                36, 37, 38, 39,  88, 89, 90, 91, 136,137,138,139, 184,185,186,187,
                40, 41, 42, 43,  92, 93, 94, 95, 140,141,142,143, 188,189,190,191,
                44, 45, 46, 47,  96, 97, 98, 99, 144,145,146,147, 192,193,194,195,
                48, 49, 50, 51 };

            uint8_t hd[4];
            int b, d, j;
            int state = 0;
            uint8_t codeword;

            static const uint8_t next_words[4][4] = {
                {0x2, 0xC, 0x1, 0xF},
                {0xE, 0x0, 0xD, 0x3},
                {0x9, 0x7, 0xA, 0x4},
                {0x5, 0xB, 0x6, 0x8}
            };

            memset(buf, 0, 12);

            for (b=0; b < 98*2; b += 4) {
                codeword = (bv[start+deinterleave_tb[b+0]] << 3) + 
                    (bv[start+deinterleave_tb[b+1]] << 2) + 
                    (bv[start+deinterleave_tb[b+2]] << 1) + 
                    bv[start+deinterleave_tb[b+3]]     ;

                /* try each codeword in a row of the state transition table */
                for (j = 0; j < 4; j++) {
                    /* find Hamming distance for candidate */
                    hd[j] = count_bits(codeword ^ next_words[state][j]);
                }
                /* find the dibit that matches the most codeword bits (minimum Hamming distance) */
                state = find_min(hd, 4);
                /* error if minimum can't be found */
                if(state == -1)
                    return -1;	// decode error, return failure
                /* It also might be nice to report a condition where the minimum is
                 * non-zero, i.e. an error has been corrected.  It probably shouldn't
                 * be a permanent failure, though.
                 *
                 * DISSECTOR_ASSERT(hd[state] == 0);
                 */

                /* append dibit onto output buffer */
                d = b >> 2;	// dibit ctr
                if (d < 48) {
                    buf[d >> 2] |= state << (6 - ((d%4) * 2));
                }
            }
            return 0;
        }


        /* Rate 3/4 confirmed data block: same 196-bit interleave as 1/2,
         * TIA Table 7-2 / kchmck p25.rs tribit FSM. Out = 18 octets
         * (2-byte serial/CRC + 16 data). Returns 0 on OK. 
         */
         
        static int block_deinterleave_34(bit_vector& bv, unsigned int start, uint8_t* buf18, bool* crc_bad_out = nullptr)
        {
            static const uint16_t deint[] = {
                 0,  1,  2,  3, 52, 53, 54, 55,100,101,102,103,148,149,150,151,
                 4,  5,  6,  7, 56, 57, 58, 59,104,105,106,107,152,153,154,155,
                 8,  9, 10, 11, 60, 61, 62, 63,108,109,110,111,156,157,158,159,
                12, 13, 14, 15, 64, 65, 66, 67,112,113,114,115,160,161,162,163,
                16, 17, 18, 19, 68, 69, 70, 71,116,117,118,119,164,165,166,167,
                20, 21, 22, 23, 72, 73, 74, 75,120,121,122,123,168,169,170,171,
                24, 25, 26, 27, 76, 77, 78, 79,124,125,126,127,172,173,174,175,
                28, 29, 30, 31, 80, 81, 82, 83,128,129,130,131,176,177,178,179,
                32, 33, 34, 35, 84, 85, 86, 87,132,133,134,135,180,181,182,183,
                36, 37, 38, 39, 88, 89, 90, 91,136,137,138,139,184,185,186,187,
                40, 41, 42, 43, 92, 93, 94, 95,140,141,142,143,188,189,190,191,
                44, 45, 46, 47, 96, 97, 98, 99,144,145,146,147,192,193,194,195,
                48, 49, 50, 51   /* v8.3: this final row was missing (table had 192 of 196 entries) */
            };
            static_assert(sizeof(deint) / sizeof(deint[0]) == 196,
                          "rate-3/4 deinterleave table must have exactly 196 entries");
            static const uint8_t pairs[16][2] = {
                {0,2},{2,2},{1,3},{3,3},{3,2},{1,2},{2,3},{0,3},
                {3,1},{1,1},{2,0},{0,0},{0,1},{2,1},{1,0},{3,0}
            };
            static const uint8_t stmap[8][8] = {
                {0,8,4,12,2,10,6,14},
                {4,12,2,10,6,14,0,8},
                {1,9,5,13,3,11,7,15},
                {5,13,3,11,7,15,1,9},
                {3,11,7,15,1,9,5,13},
                {7,15,1,9,5,13,3,11},
                {2,10,6,14,0,8,4,12},
                {6,14,0,8,4,12,2,10}
            };

            if (start + 195 >= bv.size())
                return -1;

            uint8_t coded[196];
            for (int i = 0; i < 196; i++)
                coded[i] = bv[start + deint[i]] ? 1 : 0;

            const int N = 49; /* 98 dibits / 2 = 49 steps */
            const int INF = 100000;
            int cost[50][8], prevs[50][8];
            for (int t = 0; t <= N; t++)
                for (int s = 0; s < 8; s++) {
                    cost[t][s] = INF;
                    prevs[t][s] = -1;
                }
            cost[0][0] = 0;

            for (int t = 0; t < N; t++) {
                int obs = (coded[4*t] << 3) | (coded[4*t+1] << 2) |
                          (coded[4*t+2] << 1) |  coded[4*t+3];
                for (int s = 0; s < 8; s++) {
                    if (cost[t][s] >= INF)
                        continue;
                    for (int nx = 0; nx < 8; nx++) {
                        int p = stmap[s][nx];
                        int exp = (pairs[p][0] << 2) | pairs[p][1];
                        int ham = 0, x = obs ^ exp;
                        while (x) { ham += x & 1; x >>= 1; }
                        int nc = cost[t][s] + ham;
                        if (nc < cost[t+1][nx]) {
                            cost[t+1][nx] = nc;
                            prevs[t+1][nx] = s;
                        }
                    }
                }
            }

            int end = 0;
            for (int s = 1; s < 8; s++)
                if (cost[N][s] < cost[N][end])
                    end = s;

            int path[49];
            int s = end;
            for (int t = N; t > 0; t--) {
                path[t-1] = s;
                s = prevs[t][s];
                if (s < 0)
                    return -1;
            }

            /* pack 49 tribits -> 18 bytes (last nibble unused) */
            memset(buf18, 0, 18);
            unsigned acc = 0, nbits = 0, o = 0;
            for (int i = 0; i < N && o < 18; i++) {
                acc = (acc << 3) | (path[i] & 7);
                nbits += 3;
                while (nbits >= 8 && o < 18) {
                    nbits -= 8;
                    buf18[o++] = (uint8_t)((acc >> nbits) & 0xff);
                    acc &= (1u << nbits) - 1;
                }
            }
            /* Instrumentation only -- threshold/behavior unchanged. Logs
             * the Viterbi path cost so failed (and near-threshold
             * successful) blocks can be checked for whether real data is
             * clustering just above the cost>8 cutoff -- that's the
             * signal to actually justify raising the threshold, versus
             * failures being spread out (meaning the threshold isn't
             * the bottleneck and loosening it would just admit noise). */
            int c = cost[N][end];
            bool crc_ok = blk_crc9_ok(buf18);
            if (crc_bad_out) *crc_bad_out = !crc_ok;
            if (crc_ok) g_stat_d34_crc_ok++;
            if (c > 8) {
                /* The radio accepts a block on its own CRC-9, not on our
                 * Viterbi cost, so a block that fails the cost gate but
                 * passes CRC-9 is one the radio would have received. */
                if (crc_ok) {
                    g_stat_d34_rescued++;
                    DBGP("  D34 cost=%d CRC9 OK (rescued, was FAIL at threshold=8)\n", c);
                    return 0;
                }
                DBGP("  D34 cost=%d FAIL (threshold=8, CRC9 bad)\n", c);
                return -1;
            }
            if (!crc_ok) {
                g_stat_d34_crc_bad++;
                DBGP("  D34 cost=%d ok but CRC9 BAD (kept)\n", c);
            } else if (c > 0)
                DBGP("  D34 cost=%d ok\n", c);
            return 0;
        }

        void p25p1_fdma::set_debug(int debug)
        {
            d_debug = debug;
            crypt_algs.set_debug(debug);
            framer->set_debug(debug);
        }

        void p25p1_fdma::set_nac(int nac)
        {
            d_nac = nac;
            framer->set_nac(nac);
            if (d_debug >= 10)
                DBGP("%s p25p1_fdma::set_nac: 0x%03x\n", logts.get(d_msgq_id), d_nac);
        }

        void p25p1_fdma::set_dwell_rid(uint32_t rid)
        {
            /* UNUSED as of 2026-09-26 - TODO: Purge it. */
            if (rid != d_dwell_rid && reasm.rid) {
                DBGP("%s set_dwell_rid: %u -> %u, dropping stale reasm session for rid=%u\n",
                        logts.get(d_msgq_id), d_dwell_rid, rid, reasm.rid);
                reasm.rid = 0;
                reasm.last_ser = -1;
                reasm.buf.clear();
            }
            d_dwell_rid = rid;
            if (d_debug >= 10)
                DBGP("%s p25p1_fdma::set_dwell_rid: %u\n", logts.get(d_msgq_id), d_dwell_rid);
        }

        void p25p1_fdma::crypt_behavior(int behavior)
        {
            d_behavior = behavior;
            framer->crypt_behavior(behavior);
        }

        p25p1_fdma::p25p1_fdma(op25_audio& udp, log_ts& logger, int debug, bool do_imbe, bool do_output, bool do_msgq, gr::msg_queue::sptr queue, std::deque<int16_t> &output_queue, bool do_audio_output, int msgq_id) :
            write_bufp(0),
            d_debug(debug),
            d_do_imbe(do_imbe),
            d_do_output(do_output),
            d_do_msgq(do_msgq),
            d_msgq_id(msgq_id),
            d_do_audio_output(do_audio_output),
            d_nac(0),
            d_behavior(0),
            d_msg_queue(queue),
            output_queue(output_queue),
            framer(new p25_framer(logger, debug, msgq_id)),
            qtimer(op25_timer(TIMEOUT_THRESHOLD)),
            op25audio(udp),
            logts(logger),
            crypt_algs(logger, debug, msgq_id),
            ess_keyid(0),
            ess_algid(0x80),
            vf_tgid(0)
        {
        }

        void p25p1_fdma::process_duid(uint32_t const duid, uint32_t const nac, const uint8_t* buf, const int len) {
            char wbuf[256];
            int p = 0;
            if (!d_do_msgq)
                return;
            assert (len+2 <= (int)sizeof(wbuf));
            wbuf[p++] = (nac >> 8) & 0xff;
            wbuf[p++] = nac & 0xff;
            if (buf) {
                memcpy(&wbuf[p], buf, len);	// copy data
                p += len;
            }
            send_msg(std::string(wbuf, p), duid);
            qtimer.reset();
        }

        void p25p1_fdma::process_HDU(const bit_vector& A) {
            if (d_debug >= 10) {
                DBGP("%s NAC 0x%03x HDU:  ", logts.get(d_msgq_id), framer->nac);
            }

            uint32_t MFID;
            int i, j, k, ec;
            size_t errs = 0, gly_errs = 0;
            std::vector<uint8_t> HB(63,0); // hexbit vector
            k = 0;
            for (i = 0; i < 36; i ++) {
                uint32_t CW = 0;
                for (j = 0; j < 18; j++) {  // 18 bits / cw
                    CW = (CW << 1) + A [ hdu_codeword_bits[k++] ];
                }
                HB[27 + i] = gly24128Dec(CW, &errs) & 63;
                gly_errs += errs;
            }
            ec = rs16.decode(HB); // Reed Solomon (36,20,17) error correction

            if ((ec >= 0) && (ec <= 8)) { // upper limit of 8 corrections
                j = 27;												// 72 bit MI
                for (i = 0; i < 9;) {
                    ess_mi[i++] = (uint8_t)  (HB[j  ]         << 2) + (HB[j+1] >> 4);
                    ess_mi[i++] = (uint8_t) ((HB[j+1] & 0x0f) << 4) + (HB[j+2] >> 2);
                    ess_mi[i++] = (uint8_t) ((HB[j+2] & 0x03) << 6) +  HB[j+3];
                    j += 4;
                }
                MFID      =  (HB[j  ]         <<  2) + (HB[j+1] >> 4);						// 8 bit MfrId
                ess_algid = ((HB[j+1] & 0x0f) <<  4) + (HB[j+2] >> 2);						// 8 bit AlgId
                ess_keyid = ((HB[j+2] & 0x03) << 14) + (HB[j+3] << 8) + (HB[j+4] << 2) + (HB[j+5] >> 4);	// 16 bit KeyId
                vf_tgid   = ((HB[j+5] & 0x0f) << 12) + (HB[j+6] << 6) +  HB[j+7];				// 16 bit TGID

                if (d_debug >= 10) {
                    DBGP("ESS: tgid=%d, mfid=%x, algid=%x, keyid=%x, mi=%02x %02x %02x %02x %02x %02x %02x %02x %02x",
                            vf_tgid, MFID, ess_algid, ess_keyid,
                            ess_mi[0], ess_mi[1], ess_mi[2], ess_mi[3], ess_mi[4], ess_mi[5],ess_mi[6], ess_mi[7], ess_mi[8]);
                }
            }

            if (d_debug >= 10) {
                DBGP(", gly_errs=%lu, rs_errs=%d\n", gly_errs, ec);
            }
        }

        void p25p1_fdma::process_LLDU(const bit_vector& A, std::vector<uint8_t>& HB) {
            process_duid(framer->duid, framer->nac, NULL, 0);

            int i, j, k;
            k = 0;
            for (i = 0; i < 24; i ++) { // 24 10-bit codewords
                uint32_t CW = 0;
                for (j = 0; j < 10; j++) {  // 10 bits / cw
                    CW = (CW << 1) + A[ imbe_ldu_ls_data_bits[k++] ];
                }
                HB[39 + i] = hmg1063Dec( CW >> 4, CW & 0x0f );
            }
        }

        void p25p1_fdma::process_LDU1(const bit_vector& A) {
            if (d_debug >= 10) {
                DBGP("%s NAC 0x%03x LDU1: ", logts.get(d_msgq_id), framer->nac);
            }

            std::vector<uint8_t> HB(63,0); // hexbit vector
            process_LLDU(A, HB);
            process_LCW(HB);

            if (d_debug >= 10) {
                DBGP("\n");
            }

            process_voice(A, FT_LDU1);
        }

        void p25p1_fdma::process_LDU2(const bit_vector& A) {
            uint16_t next_keyid;
            uint8_t  next_algid;
            uint8_t  next_mi[9] = {0};
            bool next_ess_valid = false;

            if (d_debug >= 10) {
                DBGP("%s NAC 0x%03x LDU2: ", logts.get(d_msgq_id), framer->nac);
            }

            std::vector<uint8_t> HB(63,0); // hexbit vector
            process_LLDU(A, HB);

            int i, j, ec;
            ec = rs8.decode(HB); // Reed Solomon (24,16,9) error correction
            if ((ec >= 0) && (ec <= 4)) {	// upper limit of 4 corrections
                j = 39;                                                             // 72 bit MI
                for (i = 0; i < 9;) {
                    next_mi[i++] = (uint8_t)  (HB[j  ]         << 2) + (HB[j+1] >> 4);
                    next_mi[i++] = (uint8_t) ((HB[j+1] & 0x0f) << 4) + (HB[j+2] >> 2);
                    next_mi[i++] = (uint8_t) ((HB[j+2] & 0x03) << 6) +  HB[j+3];
                    j += 4;
                }
                next_algid =  (HB[j  ]         <<  2) + (HB[j+1] >> 4);             //  8 bit AlgId
                next_keyid = ((HB[j+1] & 0x0f) << 12) + (HB[j+2] << 6) + HB[j+3];   // 16 bit KeyId
                next_ess_valid = true;

                if (d_debug >= 10) {
                    DBGP("ESS: algid=%x, keyid=%x, mi=%02x %02x %02x %02x %02x %02x %02x %02x %02x, rs_errs=%d\n",
                            next_algid, next_keyid,
                            next_mi[0], next_mi[1], next_mi[2], next_mi[3], next_mi[4], next_mi[5], next_mi[6], next_mi[7], next_mi[8],
                            ec); 
                }
            }

            // When configured to skip encrypted audio (crypt_behavior >= 2) there is no key to
            // decrypt with, so apply a newly received encrypted AlgId before this LDU2's voice is
            // processed. Otherwise LDU2 is decoded under the previous (cleared) crypto state and
            // its 180 ms of vocoder output reaches the audio sink before the gate engages. Key-
            // loaded operation (crypt_behavior 0 and 1) keeps the original ordering so that the MI
            // and keystream sequencing used for decryption are unchanged.
            if ((d_behavior >= 2) && next_ess_valid && (next_algid != 0x80)) {
                ess_algid = next_algid;
            }

            process_voice(A, FT_LDU2);

            // replace existing ess with newly received data now that voice processing is complete
            // if new ess was not received correctly, compute the next ess_mi from the last one
            if (next_ess_valid) {
                ess_algid = next_algid;
                ess_keyid = next_keyid;
                memcpy(ess_mi, next_mi, sizeof(next_mi));
            } else {
                op25_crypt_algs::cycle_p25_mi(ess_mi);
            }

            std::string encr = "{\"encrypted\": " + std::to_string(encrypted() ? 1 : 0) + ", \"algid\": " + std::to_string(ess_algid) + ", \"keyid\": " + std::to_string(ess_keyid) + "}";
            send_msg(encr, M_P25_JSON_DATA);
        }

        void p25p1_fdma::process_TTDU() {
            process_duid(framer->duid, framer->nac, NULL, 0);
            reset_ess();

            if ((d_do_imbe || d_do_audio_output) && (framer->duid == 0x3 || framer->duid == 0xf)) {  // voice termination
                op25audio.send_audio_flag(op25_audio::DRAIN);
            }
        }

        void p25p1_fdma::process_TDU3() {
            if (d_debug >= 10) {
                DBGP("%s NAC 0x%03x TDU3:  ", logts.get(d_msgq_id), framer->nac);
            }

            process_TTDU();

            if (d_debug >= 10) {
                DBGP("\n");
            }
        }

        void p25p1_fdma::process_TDU15(const bit_vector& A) {
            if (d_debug >= 10) {
                DBGP("%s NAC 0x%03x TDU15:  ", logts.get(d_msgq_id), framer->nac);
            }

            process_TTDU();

            int i, j, k;
            size_t gly_errs = 0, errs = 0;
            std::vector<uint8_t> HB(63,0); // hexbit vector
            k = 0;
            for (i = 0; i <= 22; i += 2) {
                uint32_t CW = 0;
                for (j = 0; j < 12; j++) {   // 12 24-bit codewords
                    CW = (CW << 1) + A [ hdu_codeword_bits[k++] ];
                    CW = (CW << 1) + A [ hdu_codeword_bits[k++] ];
                }
                uint32_t D = gly24128Dec(CW, &errs);
                HB[39 + i] = D >> 6;
                HB[40 + i] = D & 63;
            }
            process_LCW(HB);

            if (d_debug >= 10) {
                DBGP(", gly_errs=%lu\n", gly_errs);
            }
        }

        void p25p1_fdma::process_LCW(std::vector<uint8_t>& HB) {
            int ec = rs12.decode(HB); // Reed Solomon (24,12,13) error correction
            if ((ec < 0) || (ec > 6)) // upper limit of 6 corrections
                return; // failed CRC

            int i, j;
            std::vector<uint8_t> lcw(9,0); // Convert hexbits to bytes
            j = 0;
            for (i = 0; i < 9;) {
                lcw[i++] = (uint8_t)  (HB[j+39]         << 2) + (HB[j+40] >> 4);
                lcw[i++] = (uint8_t) ((HB[j+40] & 0x0f) << 4) + (HB[j+41] >> 2);
                lcw[i++] = (uint8_t) ((HB[j+41] & 0x03) << 6) +  HB[j+42];
                j += 4;
            }

            std::string pdu(11,0);
            pdu[0] = (framer->nac >> 8) & 0xff; pdu[1] = framer->nac & 0xff;
            for (int i = 0; i < 9; i++) {
                pdu[2+i] = lcw[i];
            }
            send_msg(pdu, M_P25_FDMA_LCW);

            int pb =   (lcw[0] >> 7);
            int sf =  ((lcw[0] & 0x40) >> 6);
            int lco =   lcw[0] & 0x3f;
            std::string s = "";

            if (d_debug >= 10) {
                DBGP("LCW: ec=%d, pb=%d, sf=%d, lco=%d : %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                        ec, pb, sf, lco, lcw[0], lcw[1], lcw[2], lcw[3], lcw[4], lcw[5], lcw[6], lcw[7], lcw[8]);
            }
        }


    static void dump_p25_udp_payload(const std::vector<std::array<uint8_t, 18>>& confirmed_blocks, int msgq_id, double timestamp, uint32_t source_rid) 
    {
        // 1. Stitch rule: Extract bytes [2..17] from every data block
        std::vector<uint8_t> raw_stream;
        raw_stream.reserve(confirmed_blocks.size() * 16);
        for (size_t i = 0; i < confirmed_blocks.size(); i++) {
            raw_stream.insert(raw_stream.end(), confirmed_blocks[i].begin() + 2, confirmed_blocks[i].end());
        }

        // 2. Absolute baseline bounds check (2 bytes SNDCP + 20 bytes IP + 8 bytes UDP = 30 bytes)
        if (raw_stream.size() < 30) return;

        // Verify the packet is IPv4 (Index 2 holds the '45' signature)
        if ((raw_stream[2] >> 4) != 4) return;

        // Verify the transport layer is UDP (Protocol field is at Index 11)
        if (raw_stream[11] != 0x11) return;

        // 3. Extract IP Addresses from absolute indices [14..17] and [18..21]
        char src_ip[16];
        char dst_ip[16];
        sprintf(src_ip, "%d.%d.%d.%d", raw_stream[14], raw_stream[15], raw_stream[16], raw_stream[17]);
        sprintf(dst_ip, "%d.%d.%d.%d", raw_stream[18], raw_stream[19], raw_stream[20], raw_stream[21]);

        // 4. Extract UDP Sockets from absolute indices
        uint16_t src_port = (raw_stream[22] << 8) | raw_stream[23];
        uint16_t dst_port = (raw_stream[24] << 8) | raw_stream[25];
        uint16_t udp_len  = (raw_stream[26] << 8) | raw_stream[27];

        // 5. Explicit Payload Isolation (Jumps exactly past the 30 network header bytes)
        size_t payload_offset = 30;
        if (raw_stream.size() <= payload_offset) return;

        size_t payload_len = raw_stream.size() - payload_offset;
        if (udp_len > 8) {
            size_t true_udp_len = (size_t)(udp_len - 8);
            if (true_udp_len < payload_len) {
                payload_len = true_udp_len;
            }
        }

        if (payload_len == 0) return;

        // 6. Format Time Status Clock
        char time_str[32];
        time_t raw_time = (time_t)timestamp;
        int millis = (int)((timestamp - raw_time) * 1000);
        struct tm *time_info = localtime(&raw_time);
        if (time_info) {
            strftime(time_str, sizeof(time_str), "%m/%d/%y %H:%M:%S", time_info);
            sprintf(time_str + strlen(time_str), ".%03d", millis);
        } else {
            sprintf(time_str, "%.3f", timestamp);
        }

        // 7. Output clean, verified network telemetry row
        DBGP("%s [%d] RID:%u | CAD UDP: %s:%u -> %s:%u (Bytes=%zu)\n", 
                time_str, msgq_id, source_rid, src_ip, src_port, dst_ip, dst_port, payload_len);

        // 8. Visual Data Grid View
        DBGP("-----------------------------------------------------------------\n");
        for (size_t i = 0; i < payload_len; i += 16) {
            DBGP("  %04zu: ", i);

            for (size_t j = 0; j < 16; j++) {
                if (i + j < payload_len) {
                    DBGP("%02x ", raw_stream[payload_offset + i + j]);
                } else {
                    DBGP("   ");
                }
            }
            DBGP(" | ");

            for (size_t j = 0; j < 16; j++) {
                if (i + j < payload_len) {
                    uint8_t ch = raw_stream[payload_offset + i + j];
                    if (ch >= 32 && ch < 127) {
                        DBGC((int)ch);
                    } else {
                        DBGC('.');
                    }
                }
            }
            DBGP("\n");
        }
        DBGP("-----------------------------------------------------------------\n");
    }

        struct pdu_reasm {
            uint32_t rid;
            double   last_ts;
            uint8_t  last_ser;
            std::vector<std::array<uint8_t, 18>> blocks;
        };

        
        static void dump_p25_udp_payload(const std::vector<std::array<uint8_t, 18>>& blocks,
                                         int msgq_id, const char* ts, uint32_t rid)
        {
            std::vector<uint8_t> buf;
            buf.reserve(blocks.size() * 16);
            for (size_t i = 0; i < blocks.size(); i++)
                buf.insert(buf.end(), blocks[i].begin() + 2, blocks[i].end());

            /* SNDCP SN-DATA is often 51 00; slide to IPv4 */
            size_t off = 0;
            while (off + 1 < buf.size() && buf[off] != 0x45)
                off++;
            if (off + 28 > buf.size())
                return;
            if ((buf[off] >> 4) != 4)
                return;
            if (buf[off + 9] != 0x11)
                return;

            uint16_t ip_len  = (buf[off+2] << 8) | buf[off+3];
            uint16_t src_prt = (buf[off+20] << 8) | buf[off+21];
            uint16_t dst_prt = (buf[off+22] << 8) | buf[off+23];
            uint16_t udp_len = (buf[off+24] << 8) | buf[off+25];
            size_t po = off + 28;
            size_t plen = (udp_len > 8) ? (size_t)(udp_len - 8) : 0;
            if (po + plen > buf.size())
                plen = buf.size() - po;

            DBGP("%s [%d] RID:%u CAD-UDP %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u ip=%u udp_pl=%zu\n",
                ts, msgq_id, rid,
                buf[off+12], buf[off+13], buf[off+14], buf[off+15], src_prt,
                buf[off+16], buf[off+17], buf[off+18], buf[off+19], dst_prt,
                ip_len, plen);

            for (size_t i = 0; i < plen; i += 16) {
                DBGP("  %04zu: ", i);
                for (size_t j = 0; j < 16; j++) {
                    if (i + j < plen) DBGP("%02x ", buf[po + i + j]);
                    else DBGP("   ");
                }
                DBGP(" | ");
                for (size_t j = 0; j < 16 && i + j < plen; j++) {
                    uint8_t c = buf[po + i + j];
                    DBGC((c >= 32 && c < 127) ? c : '.');
                }
                DBGP("\n");
            }
        }

//         static void reasm_flush(int msgq_id, const char* ts)
//         {
//             if (g_reasm.blocks.empty())
//                 return;
//             dump_p25_udp_payload(g_reasm.blocks, msgq_id, ts, g_reasm.rid);
//             g_reasm.blocks.clear();
//             g_reasm.rid = 0;
//             g_reasm.last_ser = 0xff;
//         }


        void p25p1_fdma::process_TSBK(const bit_vector& fr, uint32_t fr_len) {
            uint8_t op, lb = 0;
            block_vector deinterleave_buf;
            d_stat_tsbk_attempted++;
            if (process_blocks(fr, fr_len, deinterleave_buf) == 0) {
                for (size_t j = 0; (j < deinterleave_buf.size()) && (lb == 0); j++) {
                    if (crc16(deinterleave_buf[j].data(), 12) != 0) {
                        /* Previously a silent return -- TSBK CRC failures
                         * had ZERO logging at any debug level, ever,
                         * unlike PDU headers which at least print
                         * unconditionally. TSBKs carry grants and
                         * pages -- exactly the events this whole
                         * investigation exists to not miss -- so a
                         * CRC-failed TSBK could be a lost page with
                         * literally no trace anywhere. Printing the raw
                         * bytes makes it visible and correlatable
                         * against a CAD reference, the same way
                         * PDU-level misses have been checked throughout
                         * this project. bytes[7:10] is where target_rid
                         * lives for grant/page opcodes specifically
                         * (matches parse_op25_tsbk's bit layout on the
                         * Python side) -- but the opcode itself is
                         * unverified when CRC has failed, so this is a
                         * plausible guess, not a confirmed field, unlike
                         * the PDU header case where the RID position is
                         * fixed regardless of fmt. */
                        uint32_t maybe_rid = ((uint32_t)deinterleave_buf[j][7] << 16) |
                                              ((uint32_t)deinterleave_buf[j][8] << 8)  |
                                               (uint32_t)deinterleave_buf[j][9];
                        DBGP("%s TSBK CRC fail maybe_rid=%u (unverified, opcode-dependent) :",
                                logts.get(d_msgq_id), maybe_rid);
                        for (int i = 0; i < 12; i++)
                            DBGP(" %02x", deinterleave_buf[j][i]);
                        DBGP("\n");
                        return;
                    }

                    if (j == 0) d_stat_tsbk_passed++;  // count once per TSBK call
                    lb = deinterleave_buf[j][0] >> 7;	// last block flag
                    op = deinterleave_buf[j][0] & 0x3f;	// opcode
                    process_duid(framer->duid, framer->nac, deinterleave_buf[j].data(), 10);

                    tsbk_export(logts.get(d_msgq_id), framer->nac, op, lb,
                               deinterleave_buf[j].data());

                    if (d_debug >= 10) {
                        DBGP("%s NAC 0x%03x TSBK: op=%02x : %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                logts.get(d_msgq_id), framer->nac, op,
                                deinterleave_buf[j][0], deinterleave_buf[j][1], deinterleave_buf[j][2], deinterleave_buf[j][3],
                                deinterleave_buf[j][4], deinterleave_buf[j][5], deinterleave_buf[j][6], deinterleave_buf[j][7],
                                deinterleave_buf[j][8], deinterleave_buf[j][9], deinterleave_buf[j][10], deinterleave_buf[j][11]);
                    }
                }
            }
        }



        void p25p1_fdma::process_PDU(const bit_vector& fr, uint32_t fr_len) {
            uint8_t fmt, sap, blks, op = 0;
            block_vector deinterleave_buf;
            d_stat_pdu_attempted++;
            Ledger& LP = g_led[d_msgq_id & 15];
            LP.pdu_in++;

            //int rc = process_blocks(fr, fr_len, deinterleave_buf);
            int rc = process_blocks(fr, fr_len, deinterleave_buf);
            DBGP("%s PDU process_blocks rc=%d nblk=%zu fr_len=%u\n",
                    logts.get(d_msgq_id), rc, deinterleave_buf.size(), fr_len);
                    
            if (rc != 0 || fr_len > 400) {
                DBGP("%s PDU LONG hex:", logts.get(d_msgq_id));
                for (size_t b = 0; b < deinterleave_buf.size(); b++) {
                    DBGP("  [");
                    for (int i = 0; i < 12; i++)
                        DBGP("%02x", deinterleave_buf[b][i]);
                    DBGP("]");
                }
                DBGP("\n");
            }
            
            if (d_debug >= 1)
                DBGP("%s PDU process_blocks rc=%d nblk=%zu\n",
                        logts.get(d_msgq_id), rc, deinterleave_buf.size());

            if (rc != 0 || deinterleave_buf.size() == 0) {
                LP.rc_fail++;
                return;
            }

            if (crc16(deinterleave_buf[0].data(), 12) != 0) {
                if (deinterleave_buf.size() >= HDRFAIL_BIG_BLKS) LP.hf_big++;
                else if (deinterleave_buf.size() >= 3) LP.hf_mid++;
                else LP.hf_small++;
                /* Header CRC failed, so these bytes aren't verified --
                 * but the RID field position is fixed regardless of
                 * whether the rest of the header is trustworthy, and
                 * printing it (clearly marked unverified) makes it much
                 * easier to spot a pattern -- e.g. the near-immediate
                 * CRC-failing repeat that follows a real delivery,
                 * worth digging into further. */
                uint32_t maybe_rid = ((uint32_t)deinterleave_buf[0][3] << 16) |
                                      ((uint32_t)deinterleave_buf[0][4] << 8)  |
                                       (uint32_t)deinterleave_buf[0][5];
                DBGP("%s PDU header CRC fail nblk=%zu maybe_rid=%u (unverified, CRC failed) :",
                        logts.get(d_msgq_id), deinterleave_buf.size(), maybe_rid);
                for (int i = 0; i < 12; i++)
                    DBGP(" %02x", deinterleave_buf[0][i]);
                DBGP("\n");
                {
                    /* remember this header-less PDU so the ACK that follows can name the radio */
                    int hci = d_msgq_id & 15;
                    g_stat_hdrfail[hci]++;
                    /* a 1-2 block header failure is almost always a damaged ACK, not
                     * data -- don't let it be "linked" to the next radio's ACK */
                    if (deinterleave_buf.size() >= 3) {
                        g_hdrfail[hci].ts = logts.get_ts();
                        g_hdrfail[hci].nblk = (uint32_t)deinterleave_buf.size();
                        g_hdrfail[hci].linked = 0;
                    }
                    if (deinterleave_buf.size() >= HDRFAIL_BIG_BLKS) {
                        g_stat_hdrfail_big[hci]++;
                        DBGP("%s [%d] HDRFAIL_BIG nblk=%zu (text-sized PDU, header undecodable; waiting for the ACK to name the radio)\n",
                                logts.get(d_msgq_id), d_msgq_id, deinterleave_buf.size());
                    }
                }
                return;
            }
            d_stat_pdu_passed++;
            {
                uint8_t lf = deinterleave_buf[0][0] & 0x1f, ls = deinterleave_buf[0][1] & 0x3f;
                if (lf == 0x03) LP.resp[(deinterleave_buf[0][1] >> 6) & 3]++;
                else if (lf == 0x16 || lf == 0x17) LP.data_hdr++;
                else LP.other_fmt++;
                if (LP.fs[lf][ls] < 0xffffffffu) LP.fs[lf][ls]++;
            }

            /* remember when a downlink data PDU (anything but a bare
             * response) was last decoded for this RID on this channel. */
            if ((deinterleave_buf[0][0] & 0x1f) != 0x03) {
                uint32_t r = ((uint32_t)deinterleave_buf[0][3] << 16) |
                             ((uint32_t)deinterleave_buf[0][4] << 8)  |
                              (uint32_t)deinterleave_buf[0][5];
                std::map<uint32_t, double>& m = g_last_dl[d_msgq_id & 15];
                if (m.size() > 4096) m.clear();
                m[r] = logts.get_ts();
            }

            fmt  = deinterleave_buf[0][0] & 0x1f;
            sap  = deinterleave_buf[0][1] & 0x3f;
            blks = deinterleave_buf[0][6] & 0x7f;

            {
                uint32_t rid = ((uint32_t)deinterleave_buf[0][3] << 16) |
                                ((uint32_t)deinterleave_buf[0][4] << 8) |
                                 (uint32_t)deinterleave_buf[0][5];
                activity_ping(rid);
                /* fmt=0x03, blks=0 is a bare single-block frame with no
                 * confirmed data blocks -- structurally there's no IP/UDP
                 * payload here to show (that only exists once confirmed
                 * data blocks are present, handled separately in
                 * process_blocks() for fmt=0x16/0x17). This is very
                 * plausibly the SNDCP confirmed-delivery ack/response
                 * that follows a real ARS/TMS exchange -- same RID,
                 * arriving a few hundred ms later, different sap per
                 * transaction. Worth watching whether sap correlates
                 * consistently with which exchange (ARS vs TMS) it's
                 * acking. */
                if (fmt == 0x03 && blks == 0) {
                    DBGP("%s ACK/RX rid=%u sap=%u fmt=0x%02x (no data blocks -- delivery ack/response, no IP/UDP to show)\n",
                            logts.get(d_msgq_id), rid, sap, fmt);
                }

                /*  A data PDU whose header failed CRC has no RID.
                 * The tower ACKs the radio that received it ~0.5 s later; if that ACK is
                 * the next one on this channel within HDRFAIL_ACK_WIN, name the radio.
                 * "probable" because an unrelated radio's ACK could in principle land
                 * first -- dt is printed so that can be judged. */
                if (fmt == 0x03 && blks == 0) {
                    int hci = d_msgq_id & 15;
                    double tnow = logts.get_ts();
                    HdrFail& hf = g_hdrfail[hci];
                    if (hf.ts > 0 && !hf.linked && (tnow - hf.ts) >= 0 && (tnow - hf.ts) <= HDRFAIL_ACK_WIN) {
                        hf.linked = 1;
                        g_stat_attrib[hci]++;
                        DBGP("%s [%d] HDRFAIL-ACK-LINK rid=%u nblk=%u dt=%.2fs (probable recipient of the header-less PDU)%s\n",
                                logts.get(d_msgq_id), d_msgq_id, rid, hf.nblk, tnow - hf.ts,
                                hf.nblk >= HDRFAIL_BIG_BLKS ? " LOST_TEXT_SIZED" : "");
                        SalvText& sv = g_salv[hci];
                        if (sv.valid && sv.ts > 0 && (sv.ts - hf.ts) > -0.5 && (sv.ts - hf.ts) < 0.5) {
                            fprintf(stderr, "%s [%d] SALVAGE_TMS_TEXT_ATTRIBUTED rid=%u verified=%d text=%s\n",
                                    logts.get(d_msgq_id), d_msgq_id, rid, sv.verified, sv.text);
                            if (sv.verified && export_salvage_enabled())
                                tms_export(logts.get(d_msgq_id), rid, 4007, sv.text, false, true);
                            sv.valid = 0;
                        }
                    }
                }

                /* ACK-NODL. The tower only acknowledges an uplink the radio
                 * actually sent, usually its reply to a downlink. If we have
                 * decoded no downlink for this RID on this channel in the last
                 * NODL_AGE_SEC, the downlink was on the air and we missed it
                 * (or the radio sent an unsolicited uplink). Print one line
                 * always, and the detection ring for the RID range given by
                 * P25_TRACE_RIDS. */
                if (fmt == 0x03 && blks == 0) {
                    int ci = d_msgq_id & 15;
                    double now_t = logts.get_ts();
                    std::map<uint32_t, double>::iterator it = g_last_dl[ci].find(rid);
                    double age = (it == g_last_dl[ci].end()) ? -1.0 : (now_t - it->second);
                    if (age < 0 || age > NODL_AGE_SEC) {
                        g_stat_nodl[ci]++;
                        int nf = 0, nu = 0, nok = 0;
                        uint32_t cnt = g_nid_cnt[ci];
                        uint32_t span = cnt < (uint32_t)NID_RING ? cnt : (uint32_t)NID_RING;
                        for (uint32_t k = 0; k < span; k++) {
                            const NidEv& e = g_nid_ring[ci][(cnt - 1 - k) % NID_RING];
                            if (now_t - e.ts > NODL_LOOKBACK) break;
                            if (!e.ok) nf++; else { nok++; if (!e.processed) nu++; }
                        }
                        DBGP("%s [%d] ACK-NODL rid=%u last_downlink_age=%s%.1fs nid_fail_%.0fs=%d nid_ok_%.0fs=%d "
                                "nid_ok_not_processed=%d\n",
                                logts.get(d_msgq_id), d_msgq_id, rid,
                                age < 0 ? ">" : "", age < 0 ? (double)NODL_AGE_SEC : age,
                                NODL_LOOKBACK, nf, NODL_LOOKBACK, nok, nu);
                    }
                }

                /* Response packet (TIA-102.BAAA-A 6.5): octet 1 = class(2) type(3)
                 * status(3). Class 00 = ACK, 01 = NACK, 10 = ACK with selective
                 * retry (flag block follows: bit set = block received, bit clear
                 * = retry that block). "sap" in the older lines above is just
                 * the low 6 bits of this octet (type+status), not a SAP. */
                if (fmt == 0x03) {
                    uint8_t o1  = deinterleave_buf[0][1];
                    int     cls = o1 >> 6, ty = (o1 >> 3) & 7, st = o1 & 7;
                    static const char* cname[4] = {"ACK", "NACK", "SELECTIVE_RETRY", "RESERVED"};
                    static const char* nname[8] = {"illegal_format", "packet_crc_fail", "memory_full",
                                                   "out_of_seq_FSN", "undeliverable", "out_of_seq_NS",
                                                   "invalid_user", "reserved"};
                    char extra[96] = "";
                    if (cls == 1)
                        snprintf(extra, sizeof(extra), " nack=%s", nname[ty]);
                    DBGP("%s RESP rid=%u class=%s type=%d status=%d btf=%u%s",
                            logts.get(d_msgq_id), rid, cname[cls], ty, st, blks, extra);
                    if (cls == 2 && blks >= 1 && deinterleave_buf.size() >= 2) {
                        DBGP(" retry_blocks=");
                        bool any = false;
                        for (int bi = 0; bi < 64; bi++) {
                            if (!((deinterleave_buf[1][bi >> 3] >> (bi & 7)) & 1)) {
                                DBGP("%s%d", any ? "," : "", bi);
                                any = true;
                            }
                        }
                        if (!any) DBGP("none");
                    }
                    DBGP("\n");
                }

                /* Every other CRC-passing header used to fall through to
                 * only the raw-hex line below with no readable rid= --
                 * meaning genuine, real over-the-air content could be
                 * logged in a form invisible to any rid= text search. fmt=0x16/17
                 * still gets its own richer print from process_blocks;
                 * this covers everything else so nothing is ever
                 * captured-but-unlabeled again. */
                if (!(fmt == 0x03 && blks == 0) && fmt != 0x16 && fmt != 0x17)
                    DBGP("%s PDU rid=%u fmt=0x%02x sap=%u blks=%u (other/unhandled fmt)\n",
                            logts.get(d_msgq_id), rid, fmt, sap, blks);
            }

            DBGP("%s NAC 0x%03x PDU hex: fmt=0x%02x sap=%d blks=%d nblk=%zu :",
                    logts.get(d_msgq_id), framer->nac, fmt, sap, blks, deinterleave_buf.size());
            for (size_t b = 0; b < deinterleave_buf.size(); b++) {
                DBGP("  [");
                for (int i = 0; i < 12; i++)
                    DBGP("%02x", deinterleave_buf[b][i]);
                DBGP("]");
            }
            DBGP("\n");
            
            if (fmt == 0x16 || fmt == 0x17) {
                if (sap == 6)
                    reasm_flush(reasm, d_msgq_id, logts.get(d_msgq_id), "sap6_control");
            }
            
            // ********
            /* rest of function unchanged (MBT sap==61, non-MBT ignored) */
                if ((sap == 61) && ((fmt == 0x17) || (fmt == 0x15))) { // Multi Block Trunking messages
                    if (blks >= deinterleave_buf.size())
                        return; // insufficient blocks available

                    uint32_t crc1 = crc32(deinterleave_buf[1].data(), ((blks * 12) - 4) * 8);
                    uint32_t crc2 = (deinterleave_buf[blks][8] << 24) + (deinterleave_buf[blks][9] << 16) +
                        (deinterleave_buf[blks][10] << 8) + deinterleave_buf[blks][11];

                    if (crc1 != crc2)
                        return; // payload crc check failed

                    process_duid(framer->duid, framer->nac, deinterleave_buf[0].data(), ((blks + 1) * 12) - 4);

                    if (d_debug >= 10) {
                        if (fmt == 0x15) {
                            op =   deinterleave_buf[1][0] & 0x3f; // Unconfirmed MBT format
                        } else if (fmt == 0x17) {
                            op =   deinterleave_buf[0][7] & 0x3f; // Alternate MBT format
                        }

                        char s0[40], s1[40], s2[40], s3[40];
                        sprintf(s0, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[0][0], deinterleave_buf[0][1], deinterleave_buf[0][2], deinterleave_buf[0][3],
                                deinterleave_buf[0][4], deinterleave_buf[0][5], deinterleave_buf[0][6], deinterleave_buf[0][7],
                                deinterleave_buf[0][8], deinterleave_buf[0][9], deinterleave_buf[0][10], deinterleave_buf[0][11]);
                        sprintf(s1, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[1][0], deinterleave_buf[1][1], deinterleave_buf[1][2], deinterleave_buf[1][3],
                                deinterleave_buf[1][4], deinterleave_buf[1][5], deinterleave_buf[1][6], deinterleave_buf[1][7],
                                deinterleave_buf[1][8], deinterleave_buf[1][9], deinterleave_buf[1][10], deinterleave_buf[1][11]);
                        sprintf(s2, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[2][0], deinterleave_buf[2][1], deinterleave_buf[2][2], deinterleave_buf[2][3],
                                deinterleave_buf[2][4], deinterleave_buf[2][5], deinterleave_buf[2][6], deinterleave_buf[2][7],
                                deinterleave_buf[2][8], deinterleave_buf[2][9], deinterleave_buf[2][10], deinterleave_buf[2][11]);
                        sprintf(s3, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[3][0], deinterleave_buf[3][1], deinterleave_buf[3][2], deinterleave_buf[3][3],
                                deinterleave_buf[3][4], deinterleave_buf[3][5], deinterleave_buf[3][6], deinterleave_buf[3][7],
                                deinterleave_buf[3][8], deinterleave_buf[3][9], deinterleave_buf[3][10], deinterleave_buf[3][11]);
                        DBGP("%s NAC 0x%03x PDU:  fmt=%02x, op=0x%02x : %s %s %s %s\n",
                                logts.get(d_msgq_id), framer->nac, fmt, op, s0, s1, s2, s3);
                    }
                } else if (d_debug >= 10) {
                    DBGP("%s NAC 0x%03x PDU:  non-MBT message ignored\n", logts.get(d_msgq_id), framer->nac);
                }

            }
        



        int p25p1_fdma::process_blocks(const bit_vector& fr, uint32_t& fr_len, block_vector& dbuf) {
            bit_vector bv;
            /* v8.2 hardening: never read past the end of the caller's frame bits
             * (fr_len is a claimed length; fr.size() is what actually exists). */
            uint32_t src_len = fr_len;
            if ((size_t)src_len > fr.size())
                src_len = (uint32_t)fr.size();
            bv.reserve(src_len >> 1);
            for (unsigned int d = 0; d < src_len >> 1; d++) {
                if ((d + 1) % 36 == 0)
                    continue;
                bv.push_back(fr[d * 2]);
                bv.push_back(fr[d * 2 + 1]);
            }

            /* v8.2: size_t arithmetic wrapped when bv.size() < 112 */
            int bl_len = (bv.size() > (48 + 64)) ? (int)((bv.size() - (48 + 64)) / 196) : 1;
            if (bl_len < 1)
                bl_len = 1;

            for (int bl_cnt = 0; bl_cnt < bl_len; bl_cnt++) {
                dbuf.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
                /*  -Dont log the 1/2 trellis issue now, pointless as fails here are actually d3/4 trellis-
                    if (block_deinterleave(bv, 48 + 64 + bl_cnt * 196, dbuf[bl_cnt].data()) != 0) {
                    DBGP("%s process_blocks trellis1/2 fail blk=%d (kept)\n",
                            logts.get(d_msgq_id), bl_cnt);
                }*/
            }

            if (dbuf.size() >= 1) {
                const char* ts = logts.get(d_msgq_id);
                double now = logts.get_ts();

                if (crc16(dbuf[0].data(), 12) != 0) {
                    DBGP("%s PDU header CRC fail nblk=%zu\n", ts, dbuf.size());

                    /* Header trellis/CRC failed, which normally means the
                     * whole burst -- header AND any confirmed data blocks
                     * after it -- gets dropped here, even when the later
                     * blocks decoded to real (non-zero) rate-3/4 content.
                     * Confirmed with captured bytes: bursts have shown up
                     * with an all-zero header/block1 but a clearly non-zero
                     * block2 sitting where a live confirmed data block
                     * belongs (leading byte matches the FSN pattern of
                     * genuine confirmed blocks), seconds into an already
                     * active session to a known RID. A header can fade out
                     * mid-burst without the data blocks that follow it
                     * fading too -- so if a data session is already active
                     * and recent, decode this burst's blocks against the
                     * *cached* RID rather than discarding them outright. */
                    if (bl_len > 1 && reasm.rid &&
                        (now - reasm.last_ts) <= REASM_WINDOW_SEC) {
                        /* No d_dwell_rid check anymore -- that guard existed
                         * only to protect against a stale RID left over from
                         * a PREVIOUS physical channel after a retune (see
                         * set_dwell_rid()'s comment). Fixed-frequency
                         * multi_rx.py operation with no external
                         * dwell-tracking feed removes that scenario
                         * entirely: this instance's reasm is per-channel
                         * (own member, not a shared global -- see RidReasm
                         * in p25p1_fdma.h) and this channel is never
                         * retuned, so reasm.rid can only ever have been set
                         * by real traffic actually heard on THIS frequency.
                         * The remaining, narrower risk -- a genuinely
                         * different RID's headerless burst landing on this
                         * same fixed channel within REASM_WINDOW_SEC of the
                         * cached one -- is accepted for now.
                         * set_dwell_rid()/d_dwell_rid are left in place,
                         * unused but harmless, in case that decision gets
                         * revisited later. */
                        uint8_t d34[32][18];
                        int n34 = 0;
                        memset(d34, 0, sizeof(d34));
                        for (int b = 1; b < bl_len && n34 < 32; b++) {
                            uint8_t blk[18];
                            memset(blk, 0, sizeof(blk));
                            if (block_deinterleave_34(bv, 48 + 64 + b * 196, blk) != 0)
                                continue;
                            bool allzero = true;
                            for (int i = 0; i < 18 && allzero; i++)
                                if (blk[i]) allzero = false;
                            if (allzero)
                                continue;   /* nothing decoded here, not a real block */
                            memcpy(d34[n34], blk, 18);
                            n34++;
                            DBGP("%s SALVAGE blk%d rid=%u (headerless):", ts, b, reasm.rid);
                            for (int i = 0; i < 18; i++)
                                DBGP(" %02x", blk[i]);
                            DBGP("\n");
                        }
                        if (n34 > 0)
                            reasm_burst(reasm, d_msgq_id, ts, reasm.rid, now, d34, n34);
                    } else if (bl_len > 1) {
                        /* No live session to attribute this to -- the case
                         * that swallows a page with zero trace today. The
                         * header is only how we NAME the burst; the rate-3/4
                         * data blocks after it are independently coded and
                         * decodable on their own regardless of whether the
                         * header survived. So decode them anyway and gate
                         * acceptance on the DATA looking real -- Viterbi
                         * cost (unchanged threshold), a sane serial, and an
                         * actual IPv4/UDP/TMS shape once reassembled --
                         * instead of gating on session state. Added
                         * 2026-09-30; see chat log for the false-positive
                         * vs missed-page tradeoffs this was built against,
                         * and why a block-level CRC check isn't part of
                         * the gate (tested empirically against 36k real
                         * accepted blocks -- doesn't validate, so it's not
                         * a real signal here). */

                        /* Ghost-burst guard, scoped to this branch only --
                         * doesn't touch the live-session path above. Large
                         * fake bursts (seen for real: nblk=20, header
                         * `76 c0 00 3c 00 00 00 00 00 00 00 00`) decode to
                         * a header whose only non-zero content is byte 0.
                         * Skip the Viterbi work entirely on those instead
                         * of burning cycles on every block for nothing. */
                        bool header_ghost = true;
                        for (int i = 1; i < 12 && header_ghost; i++)
                            if (dbuf[0][i]) header_ghost = false;

                        /* v6: an all-zero header with a text-sized burst behind it
                         * (seen: 18 blocks, the second radio's copy of a dispatch) used
                         * to be skipped here as a "ghost", so a whole message was never
                         * even tried. Try it, but only accept blocks that pass their
                         * own CRC-9 and a datagram whose IPv4 header checksum is valid;
                         * noise cannot pass both. */
                        const bool strict = header_ghost && (bl_len - 1) >= 8;
                        const int sci = d_msgq_id & 15;

                        if (!header_ghost || strict) {
                            uint8_t d34[32][18];
                            int n34 = 0;
                            int crc_bad_cnt = 0;
                            memset(d34, 0, sizeof(d34));
                            bool sndcp_shape = false;

                            for (int b = 1; b < bl_len && n34 < 32; b++) {
                                uint8_t blk[18];
                                bool blk_crc_bad = false;
                                memset(blk, 0, sizeof(blk));
                                if (block_deinterleave_34(bv, 48 + 64 + b * 196, blk, &blk_crc_bad) != 0)
                                    continue;   // cost > 8, same threshold as always
                                if (strict && blk_crc_bad)
                                    continue;   // v6: strict mode takes CRC-9-verified blocks only

                                bool allzero = true, allf5 = true;
                                for (int i = 0; i < 18; i++) {
                                    if (blk[i]) allzero = false;
                                    if (blk[i] != 0xf5) allf5 = false;
                                }
                                if (allzero || allf5)
                                    continue;   // nothing decoded / trailing filler

                                /* Real FSNs observed on this system run
                                 * 0-37; only 3 of 36,372 accepted blocks
                                 * ever exceeded 127, all three at the same
                                 * outlier value. Cheap, evidence-backed
                                 * sanity check -- not a CRC, just a range. */
                                if (blk[0] > 127)
                                    continue;

                                if (n34 > 0 && blk[0] == d34[n34-1][0])
                                    continue;   // exact repeat serial -- air retry

                                if (n34 == 0 && blk[2] == 0x01 && blk[3] == 0x03 && blk[4] == 0xf1) {
                                    /* SNDCP-ctrl shape -- flush,
                                     * don't stitch this onto a fake TMS
                                     * message. */
                                    sndcp_shape = true;
                                    break;
                                }

                                memcpy(d34[n34], blk, 18);
                                if (blk_crc_bad) crc_bad_cnt++;
                                n34++;
                            }

                            if (n34 > 0 && !sndcp_shape) {
                                /* Local, disposable buffer -- never touches
                                 * reasm.buf/reasm.rid, so a wrong guess here
                                 * can't corrupt an actual live session. */
                                std::vector<uint8_t> scratch;
                                for (int i = 0; i < n34; i++)
                                    scratch.insert(scratch.end(), d34[i] + 2, d34[i] + 18);

                                /* Same IPv4 signature reasm_dump() keys on,
                                 * bounded to a few bytes of slack in case
                                 * the first accepted block isn't block 1. */
                                int off = -1;
                                for (size_t i = 0; i + 20 <= scratch.size() && i < 4; i++) {
                                    if (scratch[i] == 0x45) { off = (int)i; break; }
                                }

                                bool strict_ok = true;
                                if (strict) {
                                    int sihl = (off >= 0) ? ((scratch[off] & 0x0f) * 4) : 0;
                                    strict_ok = (off >= 0 && sihl >= 20 && (int)scratch.size() - off >= sihl &&
                                                 ip_hdr_cksum_ok(scratch.data() + off, sihl));
                                }

                                if (off >= 0 && (int)scratch.size() - off >= 20 && strict_ok) {
                                    /* Never a data block's bytes as RID --
                                     * that mistake's already been made and
                                     * fixed once in this file. Try the
                                     * original 12-byte header's RID field
                                     * (unverified, but it's still the right
                                     * field, just an unverified header), or
                                     * fall back to 0 and export anyway --
                                     * a page with no RID beats a silent
                                     * drop. */
                                    bool header_nonzero = dbuf[0][3] || dbuf[0][4] || dbuf[0][5];
                                    uint32_t maybe_rid = header_nonzero ?
                                        (((uint32_t)dbuf[0][3] << 16) |
                                         ((uint32_t)dbuf[0][4] << 8)  |
                                          (uint32_t)dbuf[0][5]) : 0;

                                    DBGP("%s [%d] SALVAGE_TMS rid=%u (unverified -- no live "
                                        "session, header CRC failed, %d/%d blocks accepted, crc9_bad=%d%s)\n",
                                        ts, d_msgq_id, maybe_rid, n34, bl_len - 1, crc_bad_cnt,
                                        strict ? ", strict" : "");
                                    g_salv_tmp[sci].valid = 0;
                                    g_salv_clean_next[sci] = (crc_bad_cnt == 0) ? 1 : 0;
                                    reasm_print_ip(d_msgq_id, ts, scratch.data() + off,
                                                    (int)scratch.size() - off, maybe_rid, true);
                                    if (g_salv_tmp[sci].valid) {
                                        g_salv[sci] = g_salv_tmp[sci];
                                        g_salv[sci].ts = now;
                                    }
                                }
                            }
                        }
                    }
                } else {
                    uint8_t fmt0 = dbuf[0][0] & 0x1f;
                    uint8_t sap0 = dbuf[0][1] & 0x3f;
                    uint32_t rid = ((uint32_t)dbuf[0][3] << 16) |
                                   ((uint32_t)dbuf[0][4] << 8)  |
                                    (uint32_t)dbuf[0][5];

                    if (reasm.rid && (now - reasm.last_ts) > REASM_WINDOW_SEC)
                        reasm_flush(reasm, d_msgq_id, ts, "timeout");

                    if (fmt0 == 0x16 || fmt0 == 0x17) {
                        uint8_t blks_hdr = dbuf[0][6] & 0x7f;
                        uint8_t d34[32][18];
                        bool bad34[32];
                        int n34 = 0;
                        memset(d34, 0, sizeof(d34));
                        memset(bad34, 0, sizeof(bad34));
                        for (int b = 1; b < bl_len && n34 < 32; b++) {
                            uint8_t blk[18];
                            bool crc_bad = false;
                            memset(blk, 0, sizeof(blk));
                            if (block_deinterleave_34(bv, 48 + 64 + b * 196, blk, &crc_bad) != 0) {
                                DBGP("%s CONFIRMED blk%d fail rid=%u (skipped -- later blocks of this burst shift up 16 bytes in reassembly)\n", ts, b, rid);
                                continue;
                            }
                            memcpy(d34[n34], blk, 18);
                            bad34[n34] = crc_bad;
                            n34++;
                            DBGP("%s CONFIRMED blk%d rid=%u:", ts, b, rid);
                            for (int i = 0; i < 18; i++)
                                DBGP(" %02x", blk[i]);
                            DBGP("\n");

                            if (b > blks_hdr) {
                                /* Peek at extra blocks */
                                uint32_t maybe_rid = ((uint32_t)blk[3] << 16) |
                                                      ((uint32_t)blk[4] << 8)  |
                                                       (uint32_t)blk[5];
                                DBGP("%s *** EXTRA BLOCK blk%d (beyond blks_hdr=%u) rid=%u -- "
                                    "check for leak: bytes[3:6]=%06x (as RID: %u)\n",
                                    ts, b, blks_hdr, rid, maybe_rid, maybe_rid);
                            }
                        }
                        {
                            /* Header fields per TIA-102.BAAA-A 6.4.1 (appended to
                             * the existing line so older greps still match). */
                            const uint8_t* h = dbuf[0].data();
                            int hfmf = (h[6] >> 7) & 1, hpad = h[7] & 0x1f;
                            int hns = (h[8] >> 4) & 7, hfsnf = h[8] & 0x0f, hdho = h[9] & 0x3f;
                            int exp_data = hfmf ? (16 * (int)blks_hdr - 4 - hpad) : -1;
                            DBGP("%s PDU hdr fmt=0x%02x sap=%u rid=%u blks_hdr=%u n34=%d fr_len=%u "
                                "fmf=%d pad=%d ns=%d fsnf=%d dho=%d exp_data=%d\n",
                                ts, fmt0, sap0, rid, blks_hdr, n34, fr_len,
                                hfmf, hpad, hns, hfsnf, hdho, exp_data);
                        }
                        if (fmt0 == 0x16)
                            pktasm_burst(d_msgq_id, ts, now, dbuf[0].data(), d34, n34, bad34);
                        
                        if (sap0 == 6) {
                            if (reasm.rid)
                                reasm_flush(reasm, d_msgq_id, ts, "sap6_control");
                            DBGP("%s SNDCP ctrl rid=%u n34=%d\n", ts, rid, n34);
                        } else if (sap0 == 0) {
                            reasm_burst(reasm, d_msgq_id, ts, rid, now, d34, n34);
                            if (bl_len >= 4 || fr_len >= 900) {
                                /* diagnostic dump; unrelated to holding. */
                                dump_pdu_burst(ts, rid, fr_len,
                                               dbuf[0].data(), bv, bl_len,
                                               d34, n34);
                            }
                        }
                    }
                }
            }
            
            return (dbuf.size() > 0) ? 0 : -1;
            }

        void p25p1_fdma::process_voice(const bit_vector& A, const frame_type fr_type) {
            if (d_do_imbe || d_do_audio_output) {
                if (encrypted()) {
                    crypt_algs.prepare(ess_algid, ess_keyid, PT_P25_PHASE1, ess_mi);
                }

                for(size_t i = 0; i < nof_voice_codewords; ++i) {
                    voice_codeword cw(voice_codeword_sz);
                    uint32_t E0, ET;
                    uint32_t u[8];
                    char s[128];
                    bool audio_valid = !encrypted();
                    size_t errs = 0;
                    imbe_deinterleave(A, cw, i);

                    errs = imbe_header_decode(cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], E0, ET);

                    if (d_debug >= 9) {
                        packed_codeword p_cw;
                        if (!encrypted()) {
                            imbe_pack(p_cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                            sprintf(s,"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                    p_cw[0], p_cw[1], p_cw[2], p_cw[3], p_cw[4], p_cw[5],
                                    p_cw[6], p_cw[7], p_cw[8], p_cw[9], p_cw[10]);
                            DBGP("%s IMBE (CLEARTEXT) %s errs %lu\n", logts.get(d_msgq_id), s, errs); // print to log in one operation
                        } else {
                            imbe_pack(p_cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                            sprintf(s,"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                    p_cw[0], p_cw[1], p_cw[2], p_cw[3], p_cw[4], p_cw[5],
                                    p_cw[6], p_cw[7], p_cw[8], p_cw[9], p_cw[10]);
                            DBGP("%s IMBE (CIPHERTXT) %s errs %lu\n", logts.get(d_msgq_id), s, errs); // print to log in one operation

                        }
                    }

                    // Silence unencrypted traffic when crypt_behavior = -1
                    if (!encrypted() && d_behavior == -1) {
                        return;
                        // packed_codeword p_cw;
                        // imbe_pack(p_cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                        // p_cw[0] = 0x04;
                        // p_cw[1] = 0x0C;
                        // p_cw[2] = 0xFD;
                        // p_cw[3] = 0x7B;
                        // p_cw[4] = 0xFB;
                        // p_cw[5] = 0x7D;
                        // p_cw[6] = 0xF2;
                        // p_cw[7] = 0x7B;
                        // p_cw[8] = 0x3D;
                        // p_cw[9] = 0x9E;
                        // p_cw[10] = 0x44;
                        // imbe_unpack(p_cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                    }

                    if (encrypted()) {
                        packed_codeword ciphertext;
                        imbe_pack(ciphertext, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                        audio_valid = crypt_algs.process(ciphertext, fr_type, i);
                        // sprintf(s,"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
//                                 ciphertext[0], ciphertext[1], ciphertext[2], ciphertext[3], ciphertext[4], ciphertext[5],
//                                 ciphertext[6], ciphertext[7], ciphertext[8], ciphertext[9], ciphertext[10]);
//                         DBGP("%s IMBE (PLAINTEXT) %s errs %lu\n", logts.get(d_msgq_id), s, errs); // print to log in one operation
                        imbe_unpack(ciphertext, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                    }

                    if (d_do_audio_output && audio_valid) {
                        software_decoder.decode_fullrate(u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], E0, ET);
                        audio_samples *samples = software_decoder.audio();
                        for (int i=0; i < SND_FRAME; i++) {
                            if (samples->size() > 0) {
                                snd[i] = (int16_t)(samples->front());
                                samples->pop_front();
                            } else {
                                snd[i] = 0;
                            }
                        }
                        if (op25audio.enabled()) {      // decoded audio goes out via UDP (normal code path)
                            op25audio.send_audio(snd, SND_FRAME * sizeof(int16_t));
                        } else {                        // decoded audio back to gnuradio (still supported?)
                            for (int i = 0; i < SND_FRAME; i++) {
                                output_queue.push_back(snd[i]);
                            }
                        }
                    }

                    if (d_do_output && !d_do_audio_output) { // ugh! - legacy wireshark support
                        sprintf(s, "%03x %03x %03x %03x %03x %03x %03x %03x\n", u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                        for (size_t j=0; j < strlen(s); j++) {
                            output_queue.push_back(s[j]);
                        }
                    }
                }
            }
        }

        void p25p1_fdma::reset_timer() {
            qtimer.reset();
        }

        void p25p1_fdma::call_end() {
            if (d_do_audio_output)
                op25audio.send_audio_flag(op25_audio::DRAIN);
            reset_ess();
        }

        void p25p1_fdma::crypt_reset() {
            crypt_algs.reset();
        }

        void p25p1_fdma::crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            crypt_algs.key(keyid, algid, key);
        }

        void p25p1_fdma::send_msg(const std::string msg_str, long msg_type) {
            if (!d_do_msgq)
                return;

            gr::message::sptr msg = gr::message::make_from_string(msg_str, get_msg_type(PROTOCOL_P25, msg_type), (d_msgq_id << 1), logts.get_ts());
            if (!d_msg_queue->full_p())
                d_msg_queue->insert_tail(msg);
        }

        void p25p1_fdma::process_frame() {
            {
                int ci = d_msgq_id & 15;
                if (g_nid_cnt[ci] > 0)
                    g_nid_ring[ci][(g_nid_cnt[ci] - 1) % NID_RING].processed = 1;
            }
            // Debug looking for other data
//             DBGP("RDBG: %s NAC 0x%03x DUID=0x%02x flen=%u\n",
//                 logts.get(d_msgq_id), framer->nac, framer->duid, framer->frame_size);
            // extract additional signalling information and voice codewords
            switch(framer->duid) {
                case 0x00:
                    process_HDU(framer->frame_body);
                    break;
                case 0x03:
                    process_TDU3();
                    break;
                case 0x05:
                    process_LDU1(framer->frame_body);
                    break;
                case 0x07:
                    process_TSBK(framer->frame_body, framer->frame_size);
                    break;
                case 0x0a:
                    process_LDU2(framer->frame_body);
                    break;
                case 0x0c:
                    process_PDU(framer->frame_body, framer->frame_size);
                    break;
                case 0x0f:
                    process_TDU15(framer->frame_body);
                    break;
                default:
                    DBGP("RDBG: %s unknown DUID 0x%02x\n",
                        logts.get(d_msgq_id), framer->duid);
                    break;
            }

            if (!d_do_imbe) { // send raw frame to wireshark
                              // pack the bits into bytes, MSB first
                size_t obuf_ct = 0;
                uint8_t obuf[P25_VOICE_FRAME_SIZE/2];
                for (uint32_t i = 0; i < framer->frame_size; i += 8) {
                    uint8_t b = 
                        (framer->frame_body[i+0] << 7) +
                        (framer->frame_body[i+1] << 6) +
                        (framer->frame_body[i+2] << 5) +
                        (framer->frame_body[i+3] << 4) +
                        (framer->frame_body[i+4] << 3) +
                        (framer->frame_body[i+5] << 2) +
                        (framer->frame_body[i+6] << 1) +
                        (framer->frame_body[i+7]     );
                    obuf[obuf_ct++] = b;
                }
                op25audio.send_to(obuf, obuf_ct);

                if (d_do_output) {
                    for (size_t j=0; j < obuf_ct; j++) {
                        output_queue.push_back(obuf[j]);
                    }
                }
            }
        }

        // Construct a frame one symbol at a time (used by rx.py)
        void p25p1_fdma::rx_sym (const uint8_t *syms, int nsyms) {
            for (int i1 = 0; i1 < nsyms; i1++) {
                if(framer->rx_sym(syms[i1])) {   // complete frame was detected
                    if (framer->nac == 0) {  // discard frame if NAC is invalid
                        continue;
                    }

                    process_frame();
                }  // end of complete frame
            }
            check_timeout();
        }

        // Load a frame starting with NID block (used by multi_rx.py)
        uint32_t p25p1_fdma::load_nid(const uint8_t *syms, int nsyms, const uint64_t fs) {
            uint32_t fr_len = framer->load_nid(syms, nsyms, fs);
            {
                int ci = d_msgq_id & 15;
                if (fr_len == 0) g_stat_nid_fail[ci]++; else g_stat_nid_ok[ci]++;
                if (g_nid_cnt[ci] > 0) {
                    const NidEv& pv = g_nid_ring[ci][(g_nid_cnt[ci] - 1) % NID_RING];
                    if (pv.ok && !pv.processed) {
                        /* the previous frame had a good NID but its body never
                         * completed before the next NID arrived. */
                        g_stat_abandoned[ci]++;
                        DBGP("%s [%d] FRAME-ABANDONED duid=0x%x nac=0x%03x fr_len=%u body_calls=%u "
                                "body_syms=%llu age=%.3fs\n",
                                logts.get(d_msgq_id), d_msgq_id, pv.duid, pv.nac, pv.fr_len,
                                g_body_calls[ci], (unsigned long long)g_body_syms[ci],
                                logts.get_ts() - pv.ts);
                    }
                }
                g_body_calls[ci] = 0;
                g_body_syms[ci]  = 0;
                {
                    NidEv& ev = g_nid_ring[ci][g_nid_cnt[ci] % NID_RING];
                    ev.ts = logts.get_ts();
                    ev.ok = (fr_len != 0);
                    ev.duid = ev.ok ? (uint8_t)(framer->duid & 0xf) : 0;
                    ev.nac  = ev.ok ? (uint16_t)(framer->nac & 0xfff) : 0;
                    ev.fr_len = (uint16_t)(fr_len > 65535 ? 65535 : fr_len);
                    ev.processed = 0;
                    g_nid_cnt[ci]++;
                    if (ev.ok) g_stat_duid[ci][ev.duid]++;
                }
                double now = logts.get_ts();
                if (g_stat_last_ts[ci] == 0)
                    g_stat_last_ts[ci] = now;
                else if (now - g_stat_last_ts[ci] >= 300.0) {
                    g_stat_last_ts[ci] = now;
                    ledger_print(ci, logts.get(d_msgq_id));
                    /* nid_fail includes noise that merely looked like a sync,
                     * so it is only meaningful as a trend, not a miss count. */
                    DBGP("%s [%d] STATS nid_ok=%llu nid_fail=%llu pdu_attempted=%llu "
                            "pdu_passed=%llu reasm_discard=%llu reasm_supersede=%llu "
                            "pkt_ok=%llu pkt_crc_fail=%llu pkt_merged=%llu "
                            "pkt_incomplete=%llu retry_no_original=%llu "
                            "d34_crc_ok=%llu d34_rescued=%llu d34_crc_bad=%llu "
                            "ack_nodl=%llu duid_pdu=%llu duid_tsbk=%llu duid_hdu=%llu duid_ldu1=%llu "
                            "duid_ldu2=%llu duid_tdu=%llu abandoned=%llu hdrfail=%llu hdrfail_big=%llu attributed=%llu\n",
                            logts.get(d_msgq_id), d_msgq_id,
                            (unsigned long long)g_stat_nid_ok[ci],
                            (unsigned long long)g_stat_nid_fail[ci],
                            (unsigned long long)d_stat_pdu_attempted,
                            (unsigned long long)d_stat_pdu_passed,
                            (unsigned long long)g_stat_discard[ci],
                            (unsigned long long)g_stat_supersede[ci],
                            (unsigned long long)g_stat_pkt_ok[ci],
                            (unsigned long long)g_stat_pkt_fail[ci],
                            (unsigned long long)g_stat_pkt_merged[ci],
                            (unsigned long long)g_stat_pkt_incomplete[ci],
                            (unsigned long long)g_stat_retry_noorig[ci],
                            (unsigned long long)g_stat_d34_crc_ok,
                            (unsigned long long)g_stat_d34_rescued,
                            (unsigned long long)g_stat_d34_crc_bad,
                            (unsigned long long)g_stat_nodl[ci],
                            (unsigned long long)g_stat_duid[ci][0xc],
                            (unsigned long long)g_stat_duid[ci][0x7],
                            (unsigned long long)g_stat_duid[ci][0x0],
                            (unsigned long long)g_stat_duid[ci][0x5],
                            (unsigned long long)g_stat_duid[ci][0xa],
                            (unsigned long long)(g_stat_duid[ci][0x3] + g_stat_duid[ci][0xf]),
                            (unsigned long long)g_stat_abandoned[ci],
                            (unsigned long long)g_stat_hdrfail[ci],
                            (unsigned long long)g_stat_hdrfail_big[ci],
                            (unsigned long long)g_stat_attrib[ci]);
                }
            }
            check_timeout();
            return fr_len;
        }

        // Load remainder of frame body (used by multi_rx.py)
        bool p25p1_fdma::load_body(const uint8_t *syms, int nsyms) {
            {
                int ci = d_msgq_id & 15;
                g_body_calls[ci]++;
                g_body_syms[ci] += (nsyms > 0 ? (uint64_t)nsyms : 0);
            }
            bool rc = framer->load_body(syms, nsyms);
            if (rc) {
                process_frame();
            }
            check_timeout();
            return rc;
        }

        // Check for timer expiry
        void p25p1_fdma::check_timeout() {
            if (d_do_msgq) {
                // check for timeout
                if (qtimer.expired()) {
                    if (d_debug >= 10)
                        DBGP("%s p25p1_fdma::check_timeout: expired\n", logts.get(d_msgq_id));

                    if (d_do_audio_output) {
                        op25audio.send_audio_flag(op25_audio::DRAIN);
                    }

                    qtimer.reset();
                    d_stat_timeouts++;
                    gr::message::sptr msg = gr::message::make(get_msg_type(PROTOCOL_P25, M_P25_TIMEOUT), (d_msgq_id << 1), logts.get_ts());
                    if (!d_msg_queue->full_p())
                        d_msg_queue->insert_tail(msg);
                }
            }
        }

    }  // namespace
}  // namespace
