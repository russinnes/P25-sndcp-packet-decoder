#!/bin/sh
# Copyright 2008-2011 Steve Glass
# 
# Copyright 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 Max H. Parke KA1RBI
# 
# Copyright 2018-2023 Graham J. Norbury
# 
# Copyright 2003,2004,2005,2006 Free Software Foundation, Inc.
#         (from radiorausch)
# 
# This file is part of OP25 and part of GNU Radio
# 
# OP25 is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# OP25 is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
# License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with OP25; see the file COPYING. If not, write to the Free
# Software Foundation, Inc., 51 Franklin Street, Boston, MA
# 02110-1301, USA.

# Modified for SNDCP Russ Innes

"true" '''\'
DEFAULT_PYTHON2=/usr/bin/python
DEFAULT_PYTHON3=/usr/bin/python3
if [ -f op25_python ]; then
    OP25_PYTHON=$(cat op25_python)
else
    OP25_PYTHON="/usr/bin/python"
fi

if [ -x $OP25_PYTHON ]; then
    echo Using Python $OP25_PYTHON >&2
    exec $OP25_PYTHON "$0" "$@"
elif [ -x $DEFAULT_PYTHON2 ]; then
    echo Using Python $DEFAULT_PYTHON2 >&2
    exec $DEFAULT_PYTHON2 "$0" "$@"
elif [ -x $DEFAULT_PYTHON3 ]; then
    echo Using Python $DEFAULT_PYTHON3 >&2
    exec $DEFAULT_PYTHON3 "$0" "$@"
else
    echo Unable to find Python >&2
fi
exit 127
'''
import io
import os
import pickle
import shutil
import sys
import subprocess
import threading
import socket
import math
import numpy
import time
import re
import json
import traceback
try:
    import Hamlib
except:
    pass

try:
    import Numeric
except:
    pass

from gnuradio import audio, eng_notation, gr, filter, blocks, fft, analog, digital
from gnuradio.eng_option import eng_option
from math import pi
from optparse import OptionParser

import gnuradio.op25 as op25
import gnuradio.op25_repeater as op25_repeater

import trunking

import p25_demodulator_dev as p25_demodulator
import p25_decoder

sys.path.append('tdma')
import lfsr

from gr_gnuplot import constellation_sink_c
from gr_gnuplot import fft_sink_c
from gr_gnuplot import symbol_sink_f
from gr_gnuplot import eye_sink_f
from gr_gnuplot import mixer_sink_c
from gr_gnuplot import fll_sink_c

from terminal import op25_terminal
from sockaudio  import audio_thread
from log_ts import log_ts
from helper_funcs import *

#speeds = [300, 600, 900, 1200, 1440, 1800, 1920, 2400, 2880, 3200, 3600, 3840, 4000, 4800, 6000, 6400, 7200, 8000, 9600, 14400, 19200]
speeds = [4800, 6000]

os.environ['IMBE'] = 'soft'

WIRESHARK_PORT = 23456

_def_interval = 1.0    # sec
_def_file_dir = '../www/images'


class _FeedControlClient(object):
    """Counterpart to sdr_feed.py's FeedControlServer. Used only in
    --stdin mode, where self.src doesn't exist in this process -- the
    radio lives in sdr_feed.py instead, so tuning has to reach it over
    this socket rather than through a direct method call.

    Fire-and-forget by design, matching this file's existing rule that
    nothing on the tuning/capture path may block: a lost or delayed
    retune is far cheaper than stalling the flowgraph waiting on a
    socket round trip. Reconnects lazily on the next send if the
    connection ever drops (e.g. sdr_feed.py restarted) rather than
    trying to maintain a persistent connection through failures."""

    def __init__(self, sock_path):
        self.sock_path = sock_path
        self.sock = None

    def _ensure_connected(self):
        if self.sock is not None:
            return True
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect(self.sock_path)
            self.sock = s
            return True
        except OSError as e:
            sys.stderr.write("%s feed control: connect FAILED: %s (%s)\n" % (
                log_ts.get(), self.sock_path, e))
            self.sock = None
            return False

    def send(self, d):
        if not self._ensure_connected():
            return
        try:
            self.sock.sendall((json.dumps(d) + "\n").encode())
        except OSError as e:
            sys.stderr.write("%s feed control: send FAILED: %s (%s)\n" % (
                log_ts.get(), d, e))
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None   # next send() reconnects


# The P25 receiver
#
class p25_rx_block (gr.top_block):

    # Initialize the P25 receiver
    #
    def __init__(self, options):

        self.trunk_rx = None
        self.plot_sinks = []

        gr.top_block.__init__(self)

        self.channel_rate = 0
        self.baseband_input = False
        self.rtl_found = False
        self.channel_rate = options.sample_rate
        self.fft_sink = None
        self.constellation_sink = None
        self.symbol_sink = None
        self.eye_sink = None
        self.mixer_sink = None
        self.fll_sink = None
        self.demod = None
        self.feed_ctl = None   # set in open_stdin() only; None means "tune self.src directly"

        # Conditional raw-IQ capture: off by default, started/stopped on
        # demand via the 'capture' JSON command (see process_qmsg()).
        # Taps self.source directly (post-DDC in every mode, including
        # --stdin where there's no self.src at all) -- fan-out alongside
        # the live decode chain, not instead of it. Written in GNU
        # Radio's standard raw complex64 format, so it's directly
        # replayable later through this same class's existing open_ifile
        # path with zero new replay code needed.
        #
        # THIS USED TO LIVE INSIDE __set_rx_from_osmosdr, which only
        # open_usrp() ever calls -- open_stdin() (and open_ifile/
        # open_audio/open_audio_c/open_ifile2, all of which go through
        # __set_rx_from_audio instead) never set these at all, so the
        # first 'capture' command in any of those modes hit an
        # AttributeError (confirmed in the field, --stdin mode). Moved
        # here so it runs for every mode regardless of which source path
        # gets used.
        self.capture_sink = None
        self.capture_filename = None
        # Bounds how many replay children can run concurrently (see
        # _spawn_replay). A plain threading.Semaphore, not a lock --
        # replay_concurrency may be >1 if that's ever worth allowing,
        # default is 1 (fully serialized) for the A/B test this exists
        # for. Acquired in the (daemon) thread that launches each child,
        # never on this flowgraph's own real-time path. Same
        # every-mode-not-just-osmosdr fix as capture_sink above.
        self._replay_slots = threading.Semaphore(max(1, options.replay_concurrency))
        self.target_freq = 0.0
        self.last_error_update = 0
        self.tuning_error = 0
        self.freq_correction = 0
        self.last_set_freq = 0
        self.last_set_freq_at = time.time()
        self.last_set_ppm = 0
        self.last_change_freq = 0
        self.last_change_freq_at = time.time()
        self.last_freq_params = {'freq' : 0.0, 'tgid' : None, 'tag' : "", 'tdma' : None}
        self.meta_server = None
        self.stream_url = ""
        self.ui_last_update = 0
        self.ui_timeout = 5.0

        self.src = None
        if (not options.stdin) and (not options.ifile) and (not options.input) and (not options.audio) and (not options.audio_if) and (not options.symbols):
            # check if osmocom is accessible
            try:
                import osmosdr
                self.src = osmosdr.source(options.args)
            except Exception:
                sys.stdout.write("osmosdr source_c creation failure\n")
                ignore = True
 
            if any(x in options.args.lower() for x in ['rtl', 'airspy', 'hackrf', 'uhd', 'hydrasdr']):
                self.rtl_found = True

            if options.gain_mode is not None:
                if options.gain_mode:
                    self.src.set_gain_mode(True, 0)
                else:
                    self.src.set_gain_mode(True, 0)  # UGH! Ugly workaround for gr-osmosdr airspy bug
                    self.src.set_gain_mode(False, 0)
                sys.stderr.write("gr-osmosdr driver gain_mode: %s\n" % self.src.get_gain_mode())

            gain_names = self.src.get_gain_names()
            for name in gain_names:
                g_range = self.src.get_gain_range(name)
                sys.stderr.write("gain: name: %s range: start %d stop %d step %d\n" % (name, g_range.start(), g_range.stop(), g_range.step()))
            if options.gains:
                for tup in options.gains.split(","):
                    name, gain = tup.split(":")
                    gain = int(gain)
                    sys.stderr.write("setting gain %s to %d\n" % (name, gain))
                    self.src.set_gain(gain, name)

            rates = self.src.get_sample_rates()
            try:
                sys.stderr.write("supported sample rates %d-%d step %d\n" % (rates.start(), rates.stop(), rates.step()))
            except:
                pass    # ignore

            if options.freq_corr:
                self.src.set_freq_corr(options.freq_corr)
                self.last_set_ppm = options.freq_corr

        if options.audio:
            self.channel_rate = 48000
            self.baseband_input = True

        if options.audio_if:
            self.channel_rate = 96000

        # setup (read-only) attributes
        if options.tdma_cc:
            self.symbol_rate = 6000
        else:
            self.symbol_rate = 4800
        self.symbol_deviation = 600.0
        self.basic_rate = 24000
        _default_speed = self.symbol_rate
        self.options = options
        #
        self.set_sps(_default_speed)

        # keep track of flow graph connections
        self.cnxns = []

        self.datascope_raw_input = False
        self.data_scope_connected = False

        self.constellation_scope_connected = False

        for i in range(len(speeds)):
            if speeds[i] == _default_speed:
                self.current_speed = i
                self.default_speed_idx = i

        if options.hamlib_model:
            self.hamlib_attach(options.hamlib_model)

        # wait for gdb
        if options.pause:
            sys.stdout.write("Ready for GDB to attach (pid = %d)\n" % (os.getpid(),))
            if sys.version[0] > '2':
                input("Press 'Enter' to continue...")
            else:
                raw_input("Press 'Enter' to continue...")

        self.input_q = gr.msg_queue(10)
        self.output_q = gr.msg_queue(10)
        self.meta_q = gr.msg_queue(10)
 
        # configure specified data source
        if options.stdin:
            self.open_stdin(self.channel_rate, options.gain)
            # Mirrors open_usrp's own post-setup call below: demod only
            # exists once __set_rx_from_audio (inside open_stdin) has
            # run, so this can't happen any earlier. Without this,
            # sdr_feed.py's own -f is the RAW frequency with none of
            # calibration/offset/fine_tune applied -- this is what
            # sends the corrected tune_freq to it for the first time.
            if self.options.frequency:
                self.last_freq_params['freq'] = self.options.frequency
                self.set_freq(self.options.frequency)
        elif options.input:
            self.open_ifile(self.channel_rate, options.gain, options.input, 0)
        elif (self.rtl_found or options.frequency):
            self.open_usrp()
        elif options.audio_if:
            self.open_audio_c(self.channel_rate, options.gain, options.audio_input)
        elif options.audio:
            self.open_audio(self.channel_rate, options.gain, options.audio_input)
        elif options.ifile:
            self.open_ifile2(self.channel_rate, options.ifile)
        elif options.symbols:
            self.open_symbols(self.symbol_rate, options.symbols, options.seek)
        else:
            pass

        # attach terminal thread and make sure currently tuned frequency is displayed
        self.terminal = op25_terminal(self.input_q, self.output_q, self.options.terminal_type)
        if self.terminal is None:
            sys.exit(1)
        ui_rsp = []
        js = self.send_terminal_config()
        js['uuid'] = "no-uuid"
        ui_rsp.append(js)
        msg = gr.message().make_from_string(json.dumps(ui_rsp), -4, 0, 0)
        if not self.input_q.full_p():
            self.input_q.insert_tail(msg)

        # attach meta server thread
        if self.options.metacfg is not None:
            from icemeta import meta_server
            self.meta_server = meta_server(self.meta_q, self.options.metacfg, debug=self.options.verbosity)
            try:
                with open(self.options.metacfg) as json_file:
                    meta_cfg = json.load(json_file)
                self.stream_url = "http://" + meta_cfg['icecastServerAddress'] + "/" + meta_cfg['icecastMountpoint'] + meta_cfg['icecastMountExt']
                sys.stderr.write("streaming server url=\"%s\"\n" % self.stream_url)
            except (ValueError, KeyError):
                sys.stderr.write("error reading metadata config file: %s, streaming server url disabled\n" % self.options.metacfg)
        else:
            self.meta_server = None
            sys.stderr.write("metadata update not enabled\n")

        # attach audio thread
        if self.options.udp_player:
            self.audio = audio_thread("127.0.0.1", self.options.wireshark_port, self.options.audio_output, False, self.options.audio_gain)
        else:
            self.audio = None

    # setup common flow graph elements
    #
    def __build_graph(self, source, capture_rate):
        global speeds
        global WIRESHARK_PORT

        self.rx_q = gr.msg_queue(100)
        udp_port = 0

        if self.options.udp_player:
            self.options.vocoder = True
            self.options.wireshark = True
            self.options.wireshark_host = "127.0.0.1"

        if self.options.wireshark or (self.options.wireshark_host != "127.0.0.1"):
            udp_port = self.options.wireshark_port

        self.tdma_state = False
        self.xor_cache = {}

        self.fft_state  = False
        self.c4fm_state = False
        self.fscope_state = False
        self.corr_state = False
        self.fac_state = False
        self.fsk4_demod_connected = False
        self.psk_demod_connected = False
        self.fsk4_demod_mode = False
        self.corr_i_chan = False

        if self.baseband_input:
            self.demod = p25_demodulator.p25_demod_fb(msgq_id=0, debug=self.options.verbosity, input_rate=capture_rate, excess_bw=self.options.excess_bw)
        elif self.options.symbols:
            self.demod = None
        else:    # complex input
            # local osc
            self.lo_freq = self.options.offset
            if self.options.audio_if or self.options.ifile or self.options.input:
                self.lo_freq += self.options.calibration
            self.demod = p25_demodulator.p25_demod_cb( msgq_id = 0,
                                                       debug = self.options.verbosity,
                                                       input_rate = capture_rate,
                                                       demod_type = self.options.demod_type,
                                                       relative_freq = self.lo_freq,
                                                       offset = self.options.offset,
                                                       if_rate = self.sps * 4800,
                                                       gain_mu = self.options.gain_mu,
                                                       costas_alpha = self.options.costas_alpha,
                                                       excess_bw = self.options.excess_bw,
                                                       symbol_rate = self.symbol_rate)

        num_ambe = 0
        if self.options.phase2_tdma:
            num_ambe = 1

        if self.options.crypt_behavior > 0:
            self.options.nocrypt = True

        self.decoder = p25_decoder.p25_decoder_sink_b(dest='audio', do_imbe=self.options.vocoder, num_ambe=num_ambe, wireshark_host=self.options.wireshark_host, udp_port=udp_port, do_msgq = True, msgq=self.rx_q, audio_output=self.options.audio_output, debug=self.options.verbosity, nocrypt=self.options.nocrypt)

        # connect it all up
        if self.options.symbols:
            self.connect(source, self.decoder)
        else:
            self.connect(source, self.demod, self.decoder)

            if self.options.plot_mode == 'constellation':
                self.toggle_constellation()
            elif self.options.plot_mode == 'symbol':
                self.toggle_symbol()
            elif self.options.plot_mode == 'fft':
                self.toggle_fft()
            elif self.options.plot_mode == 'datascope':
                self.toggle_eye()
            elif self.options.plot_mode == 'mixer':
                self.toggle_mixer()
            elif self.options.plot_mode == 'fll':
                self.toggle_fll()

            if self.options.raw_symbols:
                sys.stderr.write("Saving raw symbols to file: %s\n" % self.options.raw_symbols)
                self.sink_sf = blocks.file_sink(gr.sizeof_char, self.options.raw_symbols)
                self.connect(self.demod, self.sink_sf)

        logfile_workers = []
        if self.options.phase2_tdma:
            num_ambe = 2
        if self.options.logfile_workers:
            for i in range(self.options.logfile_workers):
                demod = p25_demodulator.p25_demod_cb(msgq_id=0,
                                                     debug=self.options.verbosity,
                                                     input_rate=capture_rate,
                                                     demod_type=self.options.demod_type,
                                                     offset=self.options.offset)
                decoder = p25_decoder.p25_decoder_sink_b(debug = self.options.verbosity, do_imbe = self.options.vocoder, num_ambe=num_ambe)
                logfile_workers.append({'demod': demod, 'decoder': decoder, 'active': False})
                self.connect(source, demod, decoder)

        self.trunk_rx = trunking.rx_ctl(frequency_set = self.change_freq, fa_ctrl = self.control, debug = self.options.verbosity, conf_file = self.options.trunk_conf_file, logfile_workers=logfile_workers, meta_update = self.meta_update, crypt_behavior = self.options.crypt_behavior)

        self.du_watcher = du_queue_watcher(self.rx_q, self.trunk_rx.process_qmsg)

        # Dowload encryption keys if provided
        if self.options.crypt_keys is not None:
            sys.stderr.write("%s reading crypt_keys file: %s\n" % (log_ts.get(), self.options.crypt_keys))
            crypt_keys = get_key_dict(self.options.crypt_keys, 0)
            for keyid in crypt_keys.keys():
                self.decoder.control({'tuner': 0, 'cmd': 'crypt_key', 'keyid': int(keyid), 'algid': int(crypt_keys[keyid]['algid']), 'key': crypt_keys[keyid]['key']})

    # Connect up the flow graph
    #
    def __connect(self, cnxns):
        for l in cnxns:
            for b in l:
                if b == l[0]:
                    p = l[0]
                else:
                    self.connect(p, b)
                    p = b
        self.cnxns.extend(cnxns)

    # Disconnect the flow graph
    #
    def __disconnect(self):
        for l in self.cnxns:
            for b in l:
                if b == l[0]:
                    p = l[0]
                else:
                    self.disconnect(p, b)
                    p = b
        self.cnxns = []

    def set_speed(self, new_speed):
     # assumes that lock is held, or that we are in init
        self.disconnect_demods()
        self.current_speed = new_speed
        self.connect_fsk4_demod()

    def control(self, params):
        self.decoder.control(params)

    def configure_tdma(self, params):
        if params['tdma'] is not None and not self.options.phase2_tdma:
            sys.stderr.write("***TDMA request for frequency %d failed- phase2_tdma option not enabled" % params['freq'])
            return
        set_tdma = False
        if params['tdma'] is not None:
            set_tdma = True
            self.decoder.control({'tuner': 0, 'cmd': 'set_slotid', 'slotid': params['tdma']})
        if self.demod is not None:
            self.demod.set_tdma(set_tdma)
        if set_tdma == self.tdma_state:
            return    # already in desired state
        self.tdma_state = set_tdma
        if set_tdma:
            hash = '%x%x%x' % (params['nac'], params['sysid'], params['wacn'])
            if hash not in self.xor_cache:
                self.xor_cache[hash] = lfsr.p25p2_lfsr(params['nac'], params['sysid'], params['wacn']).xor_chars
            self.decoder.control({'tuner': 0, 'cmd': 'set_xormask', 'xormask': self.xor_cache[hash]})
            rate = 6000
        else:
            rate = self.symbol_rate

        self.set_sps(rate)
        if not self.options.symbols:
            self.demod.set_omega(rate)

    def set_sps(self, rate):
        self.sps = self.basic_rate // rate
        if (self.eye_sink is not None):
            self.eye_sink.set_sps(self.sps)

    def error_tracking(self):
        UPDATE_TIME = 3.0
        if self.last_error_update + UPDATE_TIME > time.time() \
            or self.last_change_freq_at + UPDATE_TIME > time.time():
            return
        self.last_error_update = time.time()
        freq_error = self.demod.get_freq_error()
        if abs(freq_error) >= 200: # avoid hunting by only compensating errors over 200hz
            self.freq_correction += freq_error * 0.15
            do_freq_update = 1
        else:
            do_freq_update = 0
        if self.freq_correction > 600:
            self.freq_correction -= 1200
        elif self.freq_correction < -600:
            self.freq_correction += 1200
        self.tuning_error = self.freq_correction
        err_hz = 0
        err_ppm = 0
        if self.last_change_freq > 0:
            err_ppm = round((self.tuning_error*1e6) / float(self.last_change_freq))
            err_hz = -int(self.tuning_error - (err_ppm * (self.last_change_freq / 1e6)))
        if self.options.verbosity >= 10:
            sys.stderr.write('%s frequency_tracking\t%d\t%d\t%d\t%d\n' % (log_ts.get(), freq_error, self.tuning_error, err_ppm, err_hz))
        if do_freq_update:
            corrected_ppm = self.options.freq_corr + err_ppm  # compute new device ppm based on starting point plus adjustment
            if corrected_ppm != self.last_set_ppm:
                self.src.set_freq_corr(corrected_ppm)
                self.last_set_ppm = corrected_ppm
                if self.options.verbosity >= 1:
                    sys.stderr.write('%s Adjusting tuning correction: ppm(%d) ["-q %d"]\n' % (log_ts.get(), corrected_ppm, corrected_ppm))
            self.options.fine_tune = err_hz                   # replace existing fine_tune with new correction value
            self.set_freq(self.target_freq)
            if self.options.verbosity >= 2:
                sys.stderr.write('%s Adjusting tuning: ppm(%d), fine_tune(%d) ["-q %d -d %d"]\n' % (log_ts.get(), corrected_ppm, err_hz, corrected_ppm, err_hz))

    def change_freq(self, params):
        last_freq = self.last_freq_params['freq']
        self.last_freq_params = params
        freq = params['freq']
        offset = params['offset']
        center_freq = params['center_frequency']
        #if self.options.freq_error_tracking:
        #    self.error_tracking()
        self.last_change_freq = freq
        self.last_change_freq_at = time.time()

        if freq != last_freq:                               # ignore requests to tune to same freq
            if self.options.hamlib_model:
                self.hamlib.set_freq(freq)
            elif (not self.options.symbols) and params['center_frequency']:
                relative_freq = center_freq - freq
                if abs(relative_freq + self.options.offset) > self.channel_rate / 2:
                    self.lo_freq = self.options.offset                       # relative tune not possible
                    self.demod.set_relative_frequency(self.lo_freq)              # reset demod relative freq
                    self.set_freq(freq + offset)                                 # direct tune instead
                else:    
                    self.lo_freq = self.options.offset + relative_freq
                    if self.demod.set_relative_frequency(self.lo_freq):      # relative tune successful
                        self.demod.reset()                                       # reset gardner-costas loop
                        self.set_freq(center_freq + offset)
                        if self.fft_sink:
                            self.fft_sink.set_relative_freq(relative_freq)
                    else:
                        self.lo_freq = self.options.offset                   # relative tune unsuccessful
                        self.demod.set_relative_frequency(self.lo_freq)          # reset demod relative freq
                        self.set_freq(freq + offset)                             # direct tune instead
            elif not self.options.symbols:
                self.set_freq(freq + offset)
            else:
                pass                                        # fake tuning when playing back symbols file
            self.decoder.control({'tuner': 0, 'cmd': 'reset_timer'})

        self.configure_tdma(params)
        self.freq_update()

    def freq_update(self):
        params = self.last_freq_params
        params['json_type'] = 'change_freq'
        params['fine_tune'] = self.options.fine_tune
        error = None
        if self.demod is not None:
            error = self.demod.get_freq_error()
        params['error'] = error
        params['stream_url'] = self.stream_url
        return params

    def meta_update(self, tgid, tag, rid = None):
        if self.meta_server is None:
            return
        d = {'json_type': 'meta_update'}
        d['tgid'] = tgid
        d['tag'] = tag
        d['rid'] = rid
        msg = gr.message().make_from_string(json.dumps(d), -2, time.time(), 0)
        if not self.meta_q.full_p():
            self.meta_q.insert_tail(msg)

    def hamlib_attach(self, model):
        Hamlib.rig_set_debug (Hamlib.RIG_DEBUG_NONE)    # RIG_DEBUG_TRACE

        self.hamlib = Hamlib.Rig (model)
        self.hamlib.set_conf ("serial_speed","9600")
        self.hamlib.set_conf ("retry","5")

        self.hamlib.open ()

    def q_action(self, action):
        msg = gr.message().make_from_string(action, -2, 0, 0)
        if not self.rx_q.full_p():
            self.rx_q.insert_tail(msg)

    def set_gain(self, gain):
        if self.rtl_found:
            self.src.set_gain(gain, 'LNA')
            if self.options.verbosity:
                sys.stderr.write('RTL Gain of %d set to: %.1f\n' % (gain, self.src.get_gain('LNA')))
        else:
            if self.baseband_input:
                f = 1.0
            else:
                f = 0.1
            self.demod.set_baseband_gain(float(gain) * f)

    def set_audio_scaler(self, vol):
        if hasattr(self.decoder, 'set_scaler_k'):
            self.decoder.set_scaler_k((1 / 32768.0) * (vol * 0.1))

    def set_rtl_ppm(self, ppm):
        if self.feed_ctl is not None:
            self.feed_ctl.send({"command": "set_freq_corr", "ppm": ppm})
            return
        self.src.set_freq_corr(ppm)

    def set_freq_tune(self, val):
        if self.demod is not None:
            self.demod.set_relative_frequency(val + self.lo_freq)

    def set_freq(self, target_freq):
        """
        Set the center frequency we're interested in.

        @param target_freq: frequency in Hz
        @rypte: bool

        Tuning is a two step process.  First we ask the front-end to
        tune as close to the desired frequency as it can.  Then we use
        the result of that operation and our target_frequency to
        determine the value for the digital down converter.

        In --stdin mode there is no local front-end (self.src is None,
        the radio lives in sdr_feed.py) -- self.feed_ctl is set instead,
        and the tune request is forwarded there. All the correction
        terms (calibration/offset/fine_tune) are still applied HERE,
        the same as always, so sdr_feed.py only ever receives a final
        absolute frequency and doesn't need to know about any of them.
        Fire-and-forget: there's no synchronous confirmation the remote
        tune actually landed, so the True/False this returns in that
        mode means "sent", not "confirmed" -- that's an accepted
        trade-off for never blocking the tuning path on a round trip.
        """
        if not self.src and self.feed_ctl is None:
            return False
        self.target_freq = target_freq
        tune_freq = target_freq + self.options.calibration + self.options.offset + self.options.fine_tune

        if self.feed_ctl is not None:
            self.feed_ctl.send({"command": "set_freq", "freq": tune_freq})
            r = True
        else:
            r = self.src.set_center_freq(tune_freq)

        self.demod.reset()      # reset gardner-costas loop

        if self.fft_sink:
            self.fft_sink.set_center_freq(target_freq)
            self.fft_sink.set_width(self.options.sample_rate)

        if r:
            #self.myform['freq'].set_value(target_freq)     # update displayed va
            #if self.show_debug_info:
            #    self.myform['baseband'].set_value(r.baseband_freq)
            #    self.myform['ddc'].set_value(r.dxc_freq)
            return True

        return False

    def adj_tune(self, tune_incr):
        if self.target_freq == 0.0:
            return False
        self.options.fine_tune += tune_incr;
        self.set_freq(self.target_freq)
        return True

    def set_debug(self, dbglvl):
        self.options.verbosity = dbglvl
        self.decoder.set_debug(dbglvl)
        if callable(getattr(self.demod, 'set_debug', None)):
            self.demod.set_debug(dbglvl)
        if self.trunk_rx is not None:
            self.trunk_rx.set_debug(dbglvl)

    def toggle_plot(self, plot_type):
        if self.options.symbols:
            return              # plots not supported when replacing symbol

        plot_off = 0
        if (self.fft_sink is not None):
            self.toggle_fft()
            plot_off = 1
        elif (self.constellation_sink is not None):
            self.toggle_constellation()
            plot_off = 2
        elif (self.symbol_sink is not None):
            self.toggle_symbol()
            plot_off = 3
        elif (self.eye_sink is not None):
            self.toggle_eye()
            plot_off = 4
        elif (self.mixer_sink is not None):
            self.toggle_mixer()
            plot_off = 5
        elif (self.fll_sink is not None):
            self.toggle_fll()
            plot_off = 6

        if (plot_type == 1) and (plot_off != 1):    # fft
            self.toggle_fft()
        elif (plot_type == 2) and (plot_off != 2):  # constellation
            self.toggle_constellation()
        elif (plot_type == 3) and (plot_off != 3):  # symbol
            self.toggle_symbol()
        elif (plot_type == 4) and (plot_off != 4):  # datascope
            self.toggle_eye()
        elif (plot_type == 5) and (plot_off != 5):  # mixer output
            self.toggle_mixer()
        elif (plot_type == 6) and (plot_off != 6):  # fll output
            self.toggle_fll()

    def toggle_mixer(self):
        if (self.mixer_sink is None):
            self.mixer_sink = mixer_sink_c()
            self.add_plot_sink(self.mixer_sink)
            self.lock()
            self.demod.connect_complex('agc', self.mixer_sink)
            self.mixer_sink.set_width(self.basic_rate)
            self.unlock()
        elif (self.mixer_sink is not None):
            self.lock()
            self.demod.disconnect_complex(self.mixer_sink)
            self.unlock()
            self.mixer_sink.kill()
            self.remove_plot_sink(self.mixer_sink)
            self.mixer_sink = None

    def toggle_fft(self):
        if (self.fft_sink is None):
            self.fft_sink = fft_sink_c()
            self.add_plot_sink(self.fft_sink)
            if self.options.decim_amt > 1:
                self.spectrum_decim = filter.rational_resampler_ccf(1, self.options.decim_amt)
            else:
                self.spectrum_decim = None
            self.fft_sink.set_offset(self.options.offset)
            self.fft_sink.set_center_freq(self.target_freq)
            self.fft_sink.set_width(self.options.sample_rate)
            self.lock()
            if self.spectrum_decim is not None:
                self.connect(self.spectrum_decim, self.fft_sink)
                self.demod.connect_complex('src', self.spectrum_decim)
            else:
                self.demod.connect_complex('src', self.fft_sink)
            self.unlock()
        elif (self.fft_sink is not None):
            self.lock()
            if self.spectrum_decim is not None:
                self.disconnect(self.spectrum_decim, self.fft_sink)
                self.demod.disconnect_complex(self.spectrum_decim)
            else:
                self.demod.disconnect_complex(self.fft_sink)
            self.unlock()
            self.fft_sink.kill()
            self.remove_plot_sink(self.fft_sink)
            self.spectrum_decim = None
            self.fft_sink = None

    def toggle_constellation(self):
        if (self.constellation_sink is None):
            if self.options.demod_type != 'cqpsk':
                sys.stderr.write("Constellation Plot requires 'cqpsk' modulation\n")
                return
            self.constellation_sink = constellation_sink_c()
            self.add_plot_sink(self.constellation_sink)
            self.lock()
            self.demod.connect_complex('costas', self.constellation_sink)
            self.unlock()
        elif (self.constellation_sink is not None):
            self.lock()
            self.demod.disconnect_complex(self.constellation_sink)
            self.unlock()
            self.constellation_sink.kill()
            self.remove_plot_sink(self.constellation_sink)
            self.constellation_sink = None
 
    def toggle_symbol(self):
        if (self.symbol_sink is None):
            self.symbol_sink = symbol_sink_f()
            self.add_plot_sink(self.symbol_sink)
            self.lock()
            self.demod.connect_float(self.symbol_sink)
            self.unlock()
        elif (self.symbol_sink is not None):
            self.lock()
            self.demod.disconnect_float(self.symbol_sink)
            self.unlock()
            self.symbol_sink.kill()
            self.remove_plot_sink(self.symbol_sink)
            self.symbol_sink = None

    # Start/stop raw-IQ capture on demand, via the 'capture' JSON
    # command. Same dynamic-connect pattern as the scope/plot toggles
    # below (lock/connect/unlock) -- taps self.src directly, so it's a
    # fan-out alongside the live decode chain rather than replacing it.
    # filename is caller-supplied (e.g. tied to a specific dwell) so a
    # controller script can name it meaningfully; auto-generated if
    # omitted. No automatic max-duration cap here by design -- that's
    # the caller's responsibility (a controller already tracking
    # dwell/max-dwell timing for its own tune logic, e.g.
    # op25_sdr_only.py, should send 'stop' from the same place it
    # already decides to release back to the control channel).
    def start_capture(self, filename=None):
        if self.capture_sink is not None:
            return  # already capturing; caller should stop() first
        if not filename:
            # RAM-backed by default: /dev/shm is POSIX/Linux-conventional
            # tmpfs (unlike /tmp, which isn't guaranteed RAM-backed on
            # every distro). Zero real disk I/O for the common case
            # where this capture ends up discarded.
            filename = "/dev/shm/op25_capture_%d.cf32" % int(time.time())
        self.capture_filename = filename
        self.capture_sink = blocks.file_sink(gr.sizeof_gr_complex, filename)
        self.lock()
        # self.src is None in --stdin mode (no local radio); self.source
        # (post-gain, same complex64 format) always exists regardless of
        # source path and is the closest equivalent -- raw except for
        # whatever the -g digital scalar applied (1.0/no-op by default).
        # Also confirmed in the field: tapping self.src unconditionally
        # here crashed --stdin mode the same way the missing
        # capture_sink init did, right after that fix.
        self.connect(self.src if self.src is not None else self.source, self.capture_sink)
        self.unlock()
        sys.stderr.write("%s capture started: %s\n" % (log_ts.get(), filename))

    def stop_capture(self, keep_as=None):
        if self.capture_sink is None:
            return
        self.lock()
        self.disconnect(self.src if self.src is not None else self.source, self.capture_sink)
        self.unlock()
        fn = self.capture_filename
        self.capture_sink = None
        self.capture_filename = None
        if keep_as:
            # The only point any of this touches real disk I/O -- moving
            # a capture actually worth keeping out of RAM and onto
            # persistent storage. Captures that get discarded (the
            # common case) never reach this branch at all.
            try:
                d = os.path.dirname(keep_as)
                if d:
                    os.makedirs(d, exist_ok=True)
                shutil.move(fn, keep_as)
                sys.stderr.write("%s capture kept: %s -> %s\n" % (log_ts.get(), fn, keep_as))
                self._spawn_replay(keep_as)   # fire-and-forget -- does NOT block here
            except OSError as e:
                sys.stderr.write("%s capture keep FAILED: %s -> %s (%s)\n" % (
                    log_ts.get(), fn, keep_as, e))
                # shutil.move() only unlinks the source once the copy to
                # the destination succeeds, so a failed move (e.g. the
                # destination filesystem being full, as seen in the
                # field) leaves fn orphaned in /dev/shm -- nothing else
                # references it once capture_filename is cleared above,
                # so without this it just sits there taking up RAM
                # forever, on top of whatever caused the move to fail
                # in the first place.
                try:
                    if os.path.exists(fn):
                        os.remove(fn)
                        sys.stderr.write("%s cleaned up orphaned %s after failed keep\n" % (
                            log_ts.get(), fn))
                except OSError:
                    pass
        else:
            sys.stderr.write("%s capture stopped (left at %s)\n" % (log_ts.get(), fn))

    def discard_capture(self):
        # Stop (if still running) and delete the file -- done here,
        # not by the controller, since the controller may be on a
        # different host than rx.py and the file is local to this one.
        # Was RAM-only (/dev/shm) the whole time unless the caller had
        # already moved it via stop_capture(keep_as=...), so this is
        # just freeing memory pages, not a real disk write+delete.
        fn = self.capture_filename
        if self.capture_sink is not None:
            self.stop_capture()
        if fn:
            try:
                os.remove(fn)
                sys.stderr.write("%s capture discarded: %s\n" % (log_ts.get(), fn))
            except OSError as e:
                sys.stderr.write("%s capture discard FAILED: %s (%s)\n" % (log_ts.get(), fn, e))
        self.capture_filename = None

    def _replay_port(self):
        if self.options.replay_http_port:
            return self.options.replay_http_port
        # Default: live port + 1. Parse it out of terminal_type
        # ("http:0.0.0.0:8085"); fall back to a fixed port if that
        # somehow doesn't parse rather than crash a live capture over
        # a replay-scheduling detail.
        try:
            live_port = int(self.options.terminal_type.rsplit(':', 1)[1])
            return live_port + 1
        except (ValueError, IndexError, AttributeError):
            return 8086

    def _supervise_replay(self, proc, tag, timeout, filename, slot_sem=None):
        """Guarantees this replay child actually terminates. rx.py's
        file-replay mode currently loops the input file forever
        (repeat=1 in open_ifile) -- without an active watchdog per
        child, it just runs indefinitely. The previous design (a
        shared list, reaped opportunistically whenever the *next*
        capture got kept) was not sufficient for an infinite loop
        specifically: confirmed in the field -- one replay ran for
        28+ seconds, 673 lines of the same file on repeat, because no
        other capture happened to get kept in that window to trigger
        a check. This runs in its own thread per spawned replay, so
        termination never depends on any other event happening.

        Also deletes filename once the replay is done with it, whether
        it exited naturally or had to be killed. Without this, kept
        captures accumulate forever -- confirmed in the field as a
        real cascade failure: the keep-directory filled up completely
        (ENOSPC), after which every subsequent capture-keep also
        failed, compounding indefinitely. Any decoded content was
        already printed/exported live during the replay itself
        (including TMS_TEXT_PARTIAL for incomplete messages), so the
        raw IQ file has no further purpose once replay finishes --
        deleting it here doesn't lose anything downstream consumers
        would see.

        slot_sem, if given, is released here -- after wait/terminate/kill
        and the delete attempt, whatever the outcome -- so a queued
        replay's turn starts only once this one is genuinely finished
        with the CPU/IO it was using, not just once it was launched."""
        try:
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                sys.stderr.write("%s [%s] replay exceeded %.1fs (still looping), terminating\n" % (
                    log_ts.get(), tag, timeout))
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            try:
                os.remove(filename)
                sys.stderr.write("%s [%s] replay done, deleted %s\n" % (log_ts.get(), tag, filename))
            except OSError as e:
                sys.stderr.write("%s [%s] replay done, delete FAILED: %s (%s)\n" % (
                    log_ts.get(), tag, filename, e))
        finally:
            if slot_sem is not None:
                slot_sem.release()

    def _pipe_replay_output(self, proc, tag):
        """Runs in a background thread: reads the replay child's stderr
        line by line and re-writes each line into our own stderr,
        tagged so it's clearly distinguishable -- and grep/sort-able --
        from live decode output despite sharing the same stream. The
        decode-stage lines themselves (CONFIRMED blk/PDU hdr/REASM/
        TMS_TEXT/etc.) are otherwise byte-for-byte identical to live
        output, since it's the same p25p1_fdma.cc pipeline either way.
        Exits naturally on EOF, which _supervise_replay's terminate()
        causes by closing the pipe -- no coordination needed between
        the two threads beyond that."""
        try:
            for line in proc.stderr:
                sys.stderr.write("[%s] %s" % (tag, line))
        except Exception:
            pass
        sys.stderr.write("%s [%s] replay finished\n" % (log_ts.get(), tag))

    # Cached once per process: whether nice/ionice binaries exist, so
    # every replay doesn't re-probe the filesystem. None = not checked yet.
    _nice_bin = None
    _ionice_bin = None

    def _spawn_replay(self, filename):
        # Fire-and-forget from the CALLER's perspective: returns
        # immediately either way. MUST NOT block the live tuning loop --
        # that would reintroduce a blind window right in front of the
        # next grant, exactly what quiet-cancellation and this whole
        # capture feature exist to avoid.
        #
        # --no-replay: for the A/B test of whether replay children are
        # themselves contributing to live sample loss under CPU
        # pressure. The capture is already safely on disk by the time
        # we're called (stop_capture did that); we just don't act on
        # it. Left in place rather than deleted -- "nothing lost", per
        # --no-replay's help text -- so a run with replay off doesn't
        # also throw away the evidence a run with it on would have kept.
        if self.options.no_replay:
            sys.stderr.write("%s replay SKIPPED (--no-replay): %s\n" % (log_ts.get(), filename))
            return
        threading.Thread(target=self._spawn_replay_worker, args=(filename,), daemon=True).start()

    def _spawn_replay_worker(self, filename):
        # Runs entirely off the real-time path (its own daemon thread).
        # Blocks on the semaphore here, not in _spawn_replay, so a burst
        # of kept captures queues up harmlessly instead of piling on
        # concurrent replay children -- each of which is itself a full
        # second rx.py + GNU Radio flowgraph, i.e. real CPU competition
        # with the live process on the same box. Default concurrency is
        # 1 (fully serialized) for exactly that reason.
        acquired_immediately = self._replay_slots.acquire(blocking=False)
        if not acquired_immediately:
            sys.stderr.write("%s replay QUEUED (waiting for slot): %s\n" % (log_ts.get(), filename))
            self._replay_slots.acquire()

        # stderr goes through a pipe + background reader thread (not
        # direct fd inheritance) specifically so every line can be
        # tagged -- decode-stage output is otherwise indistinguishable
        # from live once interleaved into the same stream. Note the
        # timestamps replay output prints are wall-clock at REPLAY time,
        # not the original capture time -- don't read them as when the
        # event actually happened over the air.
        try:
            size = os.path.getsize(filename)
        except OSError:
            self._replay_slots.release()
            return
        rate = self.options.sample_rate or 1000000
        duration = size / 8.0 / rate   # complex64 = 8 bytes/sample
        timeout = duration + 5.0

        cmd = [
            # NOTE: must stay "--input" (options.input), not "-F"
            # (options.ifile). __init__'s data-source dispatch checks
            # `if options.input: ... elif (self.rtl_found or
            # options.frequency): open_usrp() ... elif options.ifile:
            # ...` in that order -- since this command also passes
            # -f/--frequency (for target_freq bookkeeping), using -F
            # instead of --input lets the options.frequency branch
            # fire first and try to open real SDR hardware that was
            # never initialized for a file replay, crashing with
            # "'NoneType' object has no attribute 'set_sample_rate'"
            # (confirmed in the field). --input's branch is checked
            # first regardless of options.frequency, so it's the only
            # safe choice here even though it shares a name with the
            # thing that isn't the bug. The actual repeat/looping fix
            # lives in open_ifile() itself (repeat=False), not here.
            sys.executable, os.path.abspath(sys.argv[0]),
            "--input", filename,
            "-S", str(rate),
            "-f", str(int(self.target_freq)),
            "-q", str(self.options.freq_corr),
            "-d", str(self.options.fine_tune),
            "-g", "1.0",
            "-v", str(self.options.verbosity),
            "-l", "http:0.0.0.0:%d" % self._replay_port(),
        ]

        # Best-effort: run the child at lower CPU scheduling priority
        # (nice) and idle IO priority (ionice -c3) so it can never
        # outcompete the live flowgraph for the same two cores under
        # load -- it just takes longer to finish, which is fine, it has
        # no real-time deadline. Probed once per process and skipped
        # silently if either binary is missing (e.g. a minimal image
        # without util-linux's ionice) rather than failing the replay.
        if self.options.replay_nice:
            if p25_rx_block._nice_bin is None:
                p25_rx_block._nice_bin = shutil.which("nice") or ""
            if p25_rx_block._ionice_bin is None:
                p25_rx_block._ionice_bin = shutil.which("ionice") or ""
            prefix = []
            if p25_rx_block._ionice_bin:
                prefix += [p25_rx_block._ionice_bin, "-c3"]  # idle class
            if p25_rx_block._nice_bin:
                prefix += [p25_rx_block._nice_bin, "-n", str(self.options.replay_nice)]
            cmd = prefix + cmd

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            tag = "REPLAY pid=%d %s" % (proc.pid, os.path.basename(filename))
            threading.Thread(target=self._pipe_replay_output, args=(proc, tag), daemon=True).start()
            threading.Thread(target=self._supervise_replay,
                              args=(proc, tag, timeout, filename, self._replay_slots),
                              daemon=True).start()
            sys.stderr.write("%s [%s] replay spawned (port %d, timeout %.1fs, nice=%s)\n" % (
                log_ts.get(), tag, self._replay_port(), timeout, self.options.replay_nice))
        except OSError as e:
            sys.stderr.write("%s replay spawn FAILED: %s (%s)\n" % (log_ts.get(), filename, e))
            self._replay_slots.release()

    def toggle_eye(self):
        if (self.eye_sink is None):
            self.eye_sink = eye_sink_f(sps=self.sps)
            self.add_plot_sink(self.eye_sink)
            self.lock()
            self.demod.connect_fm_demod() # make sure fm demod exists in flowgraph
            self.demod.connect_bb('symbol_filter', self.eye_sink)
            self.unlock()
        elif (self.eye_sink is not None):
            self.lock()
            self.demod.disconnect_bb(self.eye_sink)    # attempt to remove fm demod if not needed
            self.demod.disconnect_fm_demod()
            self.unlock()
            self.eye_sink.kill()
            self.remove_plot_sink(self.eye_sink)
            self.eye_sink = None

    def toggle_fll(self):
        if (self.fll_sink is None):
            self.fll_sink = fll_sink_c()
            self.add_plot_sink(self.fll_sink)
            self.lock()
            self.demod.connect_complex('fll', self.fll_sink)
            self.fll_sink.set_width(self.basic_rate)
            self.unlock()
        elif (self.fll_sink is not None):
            self.lock()
            self.unlock()
            self.fll_sink.kill()
            self.remove_plot_sink(self.fll_sink)
            self.fll_sink = None

    def add_plot_sink(self, plot):
        if plot not in self.plot_sinks:
            self.plot_sinks.append(plot)
        if self.options.terminal_type.startswith('http:'):
            plot.gnuplot.set_interval(_def_interval)
            plot.gnuplot.set_output_dir(_def_file_dir)

    def remove_plot_sink(self, plot):
        if plot in self.plot_sinks:
            self.plot_sinks.remove(plot)

    # read capture file properties (decimation etc.)
    #
    def __read_file_properties(self, filename):
        f = open(filename, "r")
        self.info = pickle.load(f)
        ToDo = True
        f.close()

    # setup to rx from file
    #
    def __set_rx_from_file(self, filename, capture_rate):
        file = blocks.file_source(gr.sizeof_gr_complex, filename, True)
        gain = blocks.multiply_const_cc(self.options.gain)
        throttle = blocks.throttle(gr.sizeof_gr_complex, capture_rate)
        self.__connect([[file, gain, throttle]])
        self.__build_graph(throttle, capture_rate)

    # setup to rx from Audio
    #
    def __set_rx_from_audio(self, capture_rate):
        self.__build_graph(self.source, capture_rate)

    # setup to rx from USRP
    #
    def __set_rx_from_osmosdr(self):
        # setup osmosdr
        capture_rate = self.src.set_sample_rate(self.options.sample_rate)
        if self.options.antenna:
            self.src.set_antenna(self.options.antenna)
        self.info["capture-rate"] = capture_rate
        self.src.set_bandwidth(capture_rate)
        # everything else
        self.__build_graph(self.src, capture_rate)

    # Write capture file properties
    #
    def __write_file_properties(self, filename):
        f = open(filename, "w")
        pickle.dump(self.info, f)
        f.close()

    def open_ifile(self, capture_rate, gain, input_filename, file_seek):
        # Was hardcoded to speed = 96000 ("TODO: fixme" in the original
        # source) -- silently ignored whatever real sample rate the file
        # was actually captured at, e.g. our own 1,000,000 sps IQ
        # captures. That wouldn't crash; it would just throttle/replay
        # the file at the wrong rate, producing a badly mistimed stream
        # the demod/decode chain could never lock onto -- indistinguishable
        # from "genuinely nothing there" without actually being that.
        # repeat=False (was 1/True): a looping file_source was replaying
        # the capture forever. Confirmed in the field to corrupt
        # reassembly -- a still-incomplete message at end-of-file got a
        # second pass appended onto its own reassembly buffer (same-RID
        # sessions within REASM_WINDOW_SEC aren't reset by a file loop
        # restart), so blocks already seen once were silently
        # re-appended as if they were new content, splicing a message's
        # own early blocks onto its own tail (e.g. real text immediately
        # followed by another copy of the fixed "AHS_Messaging_" prefix).
        # A single honest pass reports a message still incomplete at EOF
        # as exactly that, instead of "completing" it with stale bytes.
        ifile = blocks.file_source(gr.sizeof_gr_complex, input_filename, False)
        if file_seek > 0:
            rc = ifile.seek(file_seek*1024, gr.SEEK_SET)
            assert rc == True
        throttle = blocks.throttle(gr.sizeof_gr_complex, capture_rate)
        self.source = blocks.multiply_const_cc(gain)
        self.connect(ifile, throttle, self.source)
        self.__set_rx_from_audio(capture_rate)

    def open_stdin(self, capture_rate, gain):
        # Companion to sdr_feed.py: reads raw complex64 IQ from fd 0.
        # Deliberately file_descriptor_source, NOT file_source --
        # file_source does fseek/ftell bookkeeping internally that a
        # pipe can't satisfy (pipes aren't seekable), which is exactly
        # the kind of thing that fails in ways indistinguishable from
        # "no signal" rather than a clean error. file_descriptor_source
        # has no such assumption, it just reads bytes as they arrive.
        #
        # No throttle here, unlike open_ifile: a file replay throttles
        # itself down to a realistic pace because the whole file is
        # instantly available and would otherwise blast through faster
        # than any timing-sensitive block downstream expects. Stdin has
        # no such surplus -- sdr_feed.py on the other end is already
        # producing samples at real hardware rate (or slower, if it's
        # catching up from its own ring buffer), so an extra throttle
        # here would only add pacing jitter of its own for no benefit.
        # gain here is a post-DDC digital scalar (same reuse of -g/--gain
        # as open_ifile), NOT the hardware RF gain -- that's sdr_feed.py's
        # own -g/-N, already applied on its side before samples ever
        # reach this pipe. options.gain defaults to None (meant for the
        # USRP-gain-in-dB case); the replay-child command always passes
        # an explicit "-g 1.0" so it never hits this, but a manual
        # --stdin invocation without -g does. None * anything crashes
        # multiply_const_cc's constructor, so default to a no-op 1.0.
        if gain is None:
            gain = 1.0
        ifile = blocks.file_descriptor_source(gr.sizeof_gr_complex, 0, False)
        self.source = blocks.multiply_const_cc(gain)
        self.connect(ifile, self.source)
        # self.src stays None in this mode (there's no local radio to
        # hold a reference to) -- set_freq/set_rtl_ppm check for that
        # and forward to sdr_feed.py over this instead. See
        # _FeedControlClient's docstring for why fire-and-forget.
        self.feed_ctl = _FeedControlClient(self.options.feed_control_sock)
        self.__set_rx_from_audio(capture_rate)

    def open_ifile2(self, capture_rate, file_name):
        source = blocks.file_source(gr.sizeof_gr_complex, file_name, False)
        throttle = blocks.throttle(gr.sizeof_gr_complex, capture_rate)
        self.connect(source, throttle)
        self.__build_graph(throttle, capture_rate)

    def open_symbols(self, symbol_rate, file_name, file_seek):
        sys.stderr.write("Reading raw symbols from file: %s\n" % self.options.symbols)
        source = blocks.file_source(gr.sizeof_char, file_name, False)
        if file_seek > 0:
            rc = source.seek(file_seek*4800, 0) # Seek in seconds (4800sps)
            assert rc == True
        throttle = blocks.throttle(gr.sizeof_char, symbol_rate)
        throttle.set_max_noutput_items(int(symbol_rate/50));
        self.connect(source, throttle)
        self.__build_graph(throttle, symbol_rate)

    def open_audio_c(self, capture_rate, gain, audio_input_filename):
        self.info = {
                "capture-rate": capture_rate,
                "center-freq": 0,
                "source-dev": "AUDIO",
                "source-decim": 1 }
        self.audio_source = audio.source(capture_rate, audio_input_filename)
        self.audio_cvt = blocks.float_to_complex()
        self.connect((self.audio_source, 0), (self.audio_cvt, 0))
        self.connect((self.audio_source, 1), (self.audio_cvt, 1))
        self.source = blocks.multiply_const_cc(gain)
        self.connect(self.audio_cvt, self.source)
        self.__set_rx_from_audio(capture_rate)

    def open_audio(self, capture_rate, gain, audio_input_filename):
            self.info = {
                "capture-rate": capture_rate,
                "center-freq": 0,
                "source-dev": "AUDIO",
                "source-decim": 1 }
            self.audio_source = audio.source(capture_rate, audio_input_filename)
            self.source = blocks.multiply_const_ff(gain)
            self.connect(self.audio_source, self.source)
            self.__set_rx_from_audio(capture_rate)

    # Open the USRP
    #
    def open_usrp(self):
        # try:
            self.info = {
                "capture-rate": "unknown",
                "center-freq": self.options.frequency,
                "source-dev": "USRP",
                "source-decim": 1 }
            self.__set_rx_from_osmosdr()
            if self.options.frequency:
                self.last_freq_params['freq'] = self.options.frequency
                self.set_freq(self.options.frequency)
        # except Exception, x:
        #     wx.MessageBox("Cannot open USRP: " + x.message, "USRP Error", wx.CANCEL | wx.ICON_EXCLAMATION)

    def process_ajax(self):
        if not self.options.terminal_type.startswith('http:'):
            return
        filenames = [sink.gnuplot.filename for sink in self.plot_sinks if sink.gnuplot.filename]
        error = None
        if self.demod is not None:
            error = self.demod.get_freq_error()
        d = {'json_type': 'rx_update', 'error': error, 'fine_tune': self.options.fine_tune, 'files': filenames}
        return d

    def send_terminal_config(self):
        self.terminal_config = {'json_type': 'terminal_config', 'terminal_interface': 'legacy'}
        return self.terminal_config

    def process_qmsg(self, msg):
        # return true = end top block
        RX_COMMANDS = 'skip lockout hold whitelist reload'.split()
        s = msg.to_string()
        ui_rsp = []
        if type(s) is not str and isinstance(s, bytes):
            # should only get here if python3
            s = s.decode()
        try:    # See if we can treat the incoming message as JSON format (from HTTP UI)
            d = json.loads(s)
            s = d['command'] if "command" in d and d['command'] is not None else ""
            m_uuid = d['uuid'] if "uuid" in d and d['uuid'] is not None else "no-uuid"
        except (json.JSONDecodeError): # otherwise fall back to string format (Curses)
            m_uuid = "no-uuid"

        if s == 'quit':
            return True
        elif s == 'update':
            self.ui_last_update = time.time()
            if self.trunk_rx is None:
                return False    ## possible race cond - just ignore
            js = json.loads(self.trunk_rx.to_json())
            js['uuid'] = m_uuid
            ui_rsp.append(js)
            ui_rsp.append(self.freq_update())
            ui_rsp.append(self.process_ajax())
        elif s == 'set_debug':
            self.set_debug(int(msg.arg1()))
        elif s == 'get_terminal_config':
            js = self.send_terminal_config()
            js['uuid'] = m_uuid
            ui_rsp.append(js)
        elif s == 'set_freq':
            freq = msg.arg1()
            self.last_freq_params['freq'] = freq
            self.set_freq(freq)
        elif s == 'set_dwell_rid':
            # Companion to set_freq, sent by op25_sdr_only.py right before
            # (or instead of) a data-channel retune so the C++ decoder
            # knows which RID this dwell belongs to. Lets process_blocks()
            # in p25p1_fdma.cc gate its header-CRC-fail salvage path on
            # g_reasm.rid == d_dwell_rid, instead of trusting whatever RID
            # a stale, still-recent g_reasm session happens to be holding
            # from the PREVIOUS dwell -- see p25p1_fdma::set_dwell_rid().
            # arg2 unused, but sent (as 0) for the same reason http_capture()
            # always sends it: some post_req paths index arg2 directly.
            rid = int(msg.arg1())
            if self.decoder is not None:
                self.decoder.control({'tuner': 0, 'cmd': 'set_dwell_rid', 'rid': rid})
        elif s == 'adj_tune':
            freq = msg.arg1()
            self.adj_tune(freq)
        elif s == 'toggle_plot':
            plot_type = msg.arg1()
            self.toggle_plot(plot_type)
        elif s == 'dump_tgids':
            self.trunk_rx.dump_tgids()
        elif s == 'add_default_config':
            nac = msg.arg1()
            self.trunk_rx.add_default_config(int(nac))
        elif s == 'capture':
            # {"command":"capture","arg1":"start","arg2":"/dev/shm/file.cf32"}
            # {"command":"capture","arg1":"stop"}                        -- stop, leave in place
            # {"command":"capture","arg1":"stop","arg2":"/path/keep.cf32"} -- stop AND move to disk
            # {"command":"capture","arg1":"discard"}                     -- stop (if needed) and delete
            action = d.get('arg1', 'start')
            if action == 'start':
                fn = d.get('arg2')
                self.start_capture(fn if isinstance(fn, str) and fn else None)
            elif action == 'stop':
                keep_as = d.get('arg2')
                self.stop_capture(keep_as if isinstance(keep_as, str) and keep_as else None)
            elif action == 'discard':
                self.discard_capture()
        elif s == 'watchdog':
            if self.ui_last_update > 0 and (time.time() > (self.ui_last_update + self.ui_timeout)):
                self.ui_last_update = 0
                if self.options.verbosity >= 10:
                    sys.stderr.write("%s UI Timeout\n" % log_ts.get())
                self.toggle_plot(0)

        elif s in RX_COMMANDS:
            if not self.rx_q.full_p():
                self.rx_q.insert_tail(msg)

        if len(ui_rsp) > 0:
            msg = gr.message().make_from_string(json.dumps(ui_rsp), -4, 0, 0)
            if not self.input_q.full_p():
                self.input_q.insert_tail(msg)

        return False

############################################################################

# data unit receive queue
#
class du_queue_watcher(threading.Thread):

    def __init__(self, msgq,  callback, **kwds):
        threading.Thread.__init__ (self, **kwds)
        self.daemon = True
        self.msgq = msgq
        self.callback = callback
        self.keep_running = True
        self.start()

    def run(self):
        while(self.keep_running):
            if not self.msgq.empty_p(): # check queue before trying to read a message to avoid deadlock at startup
                msg = self.msgq.delete_head()
                if msg is not None:
                    self.callback(msg)
                else:
                    self.keep_running = False
            else: # empty queue
                time.sleep(0.01)            

class rx_main(object):
    def __init__(self):
        self.keep_running = True
        self.cli_options()
        self.tb = p25_rx_block(self.options)
        self.q_watcher = du_queue_watcher(self.tb.output_q, self.process_qmsg)
        sys.stderr.write('python version detected: %s\n' % sys.version)

    def process_qmsg(self, msg):
        if self.tb.process_qmsg(msg):
            #self.tb.stop()
            self.keep_running = False

    def run(self):
        try:
            self.tb.start()
            if self.options.symbols:
                self.tb.wait()
            else:
                while self.keep_running:
                    time.sleep(1)
                    msg = gr.message().make_from_string("watchdog", -2, 0, 0)
                    if not self.tb.output_q.full_p():
                        self.tb.output_q.insert_tail(msg)
            sys.stderr.write('Flowgraph completed. Exiting\n')
        except:
            sys.stderr.write('main: exception occurred\n')
            sys.stderr.write('main: exception:\n%s\n' % traceback.format_exc())
        if self.tb.terminal:
            self.tb.terminal.end_terminal()
        if self.tb.meta_server:
            self.tb.meta_server.stop()
        if self.tb.audio:
            self.tb.audio.stop()
        self.tb.stop()
        for sink in self.tb.plot_sinks:
            sink.kill()

    def cli_options(self):
        # command line argument parsing
        parser = OptionParser(option_class=eng_option)
        parser.add_option("--args", type="string", default="", help="device args")
        parser.add_option("--antenna", type="string", default="", help="select antenna")
        parser.add_option("-a", "--audio", action="store_true", default=False, help="use direct audio input")
        parser.add_option("-A", "--audio-if", action="store_true", default=False, help="soundcard IF mode (use --calibration to set IF freq)")
        parser.add_option("-I", "--audio-input", type="string", default="", help="pcm input device name.  E.g., hw:0,0 or /dev/dsp")
        parser.add_option("-i", "--input", default=None, help="input file name")
        parser.add_option("--stdin", action="store_true", default=False,
                          help="read raw complex64 IQ from stdin (fd 0) instead of a "
                               "file or the SDR directly -- for piping in a separate "
                               "acquisition process (e.g. sdr_feed.py) that owns the "
                               "radio and buffers, so this process is decoupled from "
                               "the hardware's real-time deadline. Sample rate must "
                               "match the producer's -S. Checked before --input/-F.")
        parser.add_option("--feed-control-sock", type="string",
                          default="/tmp/op25_feed_control.sock",
                          help="With --stdin: Unix domain socket to send set_freq/"
                               "set_freq_corr commands to, since this process has no "
                               "self.src of its own to tune directly -- must match "
                               "sdr_feed.py's --control-sock. Ignored without --stdin.")
        parser.add_option("-b", "--excess-bw", type="eng_float", default=0.2, help="for RRC filter", metavar="Hz")
        parser.add_option("-c", "--calibration", type="eng_float", default=0.0, help="USRP offset or audio IF frequency", metavar="Hz")
        parser.add_option("-C", "--costas-alpha", type="eng_float", default=0.001, help="value of alpha for Costas loop", metavar="Hz")
        parser.add_option("-D", "--demod-type", type="choice", default="cqpsk", choices=('cqpsk', 'fsk4'), help="cqpsk | fsk4")
        parser.add_option("-P", "--plot-mode", type="choice", default=None, choices=(None, 'constellation', 'fft', 'symbol', 'datascope', 'mixer', 'fll'), help="constellation | fft | symbol | datascope | mixer | tuner")
        parser.add_option("-f", "--frequency", type="eng_float", default=0.0, help="USRP center frequency", metavar="Hz")
        parser.add_option("-F", "--ifile", type="string", default=None, help="read input from complex capture file")
        parser.add_option("-H", "--hamlib-model", type="int", default=None, help="specify model for hamlib")
        parser.add_option("-s", "--seek", type="int", default=0, help="ifile seek in K, symbols file seek in seconds")
        parser.add_option("-l", "--terminal-type", type="string", default='curses', help="'curses' or udp port or 'http:host:port'")
        parser.add_option("-L", "--logfile-workers", type="int", default=None, help="number of demodulators to instantiate")
        parser.add_option("-M", "--metacfg", type="string", default=None, help="Icecast Metadata Config File")
        parser.add_option("-S", "--sample-rate", type="int", default=960000, help="source samp rate")
        parser.add_option("-t", "--tone-detect", action="store_true", default=False, help="use experimental tone detect algorithm")
        parser.add_option("-T", "--trunk-conf-file", type="string", default=None, help="trunking config file name")
        parser.add_option("-v", "--verbosity", type="int", default=0, help="message debug level")
        parser.add_option("--replay-http-port", type="int", default=None,
                          help="Fire-and-forget replay children spawned by a kept "
                               "capture use this HTTP port instead of the live one "
                               "(--capture 'discard'/'stop' handling). Must differ "
                               "from the live -l port. Defaults to live port + 1.")
        parser.add_option("--no-replay", action="store_true", default=False,
                          help="Disable spawning replay-child subprocesses entirely. "
                               "Kept captures are still moved to disk (nothing lost), "
                               "just never auto-replayed. For A/B testing whether the "
                               "replay children are themselves contributing to live "
                               "sample loss under CPU pressure -- run one window with "
                               "this on, one with it off, and compare the live "
                               "had_ping-blank rate between them.")
        parser.add_option("--replay-nice", type="int", default=10,
                          help="Run replay children at this 'nice' level (and best-"
                               "effort idle IO priority) so they never outcompete the "
                               "live flowgraph for CPU/IO under load. 0 disables "
                               "niceing. Ignored if --no-replay is set.")
        parser.add_option("--replay-concurrency", type="int", default=1,
                          help="Max replay children allowed to run at once; further "
                               "kept captures queue and replay once a slot frees up. "
                               "Serializing (the default, 1) avoids replay subprocesses "
                               "piling up and adding their own CPU pressure on top of "
                               "whatever caused the live miss in the first place.")
        parser.add_option("-V", "--vocoder", action="store_true", default=False, help="voice codec")
        parser.add_option("-n", "--nocrypt", action="store_true", default=False, help="silence encrypted traffic")
        parser.add_option("--crypt-behavior", type="int", default=2, help="encrypted traffic behavior: 0=allow, 1=silence, 2=skip")
        parser.add_option("-k", "--crypt-keys", type="string", default=None, help="decryption keys file (in json format)")
        parser.add_option("-o", "--offset", type="eng_float", default=0.0, help="tuning offset frequency [to circumvent DC offset]", metavar="Hz")
        parser.add_option("-p", "--pause", action="store_true", default=False, help="block on startup")
        parser.add_option("-w", "--wireshark", action="store_true", default=False, help="output data to Wireshark")
        parser.add_option("-W", "--wireshark-host", type="string", default="127.0.0.1", help="Wireshark host")
        parser.add_option("-u", "--wireshark-port", type="int", default=23456, help="Wireshark udp port")
        parser.add_option("-r", "--raw-symbols", type="string", default=None, help="dump decoded symbols to file")
        parser.add_option("--symbols", type="string", default="", help="playback symbols file (captured using -r)")
        parser.add_option("-R", "--rx-subdev-spec", type="subdev", default=(0, 0), help="select USRP Rx side A or B (default=A)")
        parser.add_option("-g", "--gain", type="eng_float", default=None, help="set USRP gain in dB (default is midpoint) or set audio gain")
        parser.add_option("--gain-mode", type="int", help="Control SDR AGC with set_gain_mode()")
        parser.add_option("-G", "--gain-mu", type="eng_float", default=0.025, help="gardner gain")
        parser.add_option("-N", "--gains", type="string", default=None, help="gain settings")
        parser.add_option("-O", "--audio-output", type="string", default="default", help="audio output device name")
        parser.add_option("-x", "--audio-gain", type="eng_float", default="1.0", help="audio gain (default = 1.0)")
        parser.add_option("-X", "--freq-error-tracking", action="store_true", default=False, help="enable experimental frequency error tracking")
        parser.add_option("-U", "--udp-player", action="store_true", default=False, help="enable built-in udp audio player")
        parser.add_option("-q", "--freq-corr", type="eng_float", default=0.0, help="frequency correction")
        parser.add_option("-d", "--fine-tune", type="eng_float", default=0.0, help="fine tuning")
        parser.add_option("-2", "--phase2-tdma", action="store_true", default=False, help="enable phase2 tdma decode")
        parser.add_option("--tdma-cc", action="store_true", default=False, help="enable tdma control channel")
        parser.add_option("-Z", "--decim-amt", type="int", default=1, help="spectrum decimation")
        (options, args) = parser.parse_args()
        if len(args) != 0:
            parser.print_help()
            sys.exit(1)
        self.options = options

# Start the receiver
#

if __name__ == "__main__":
    if sys.version[0] > '2':
        sys.stderr = io.TextIOWrapper(sys.stderr.detach().detach(), write_through=True) # disable stderr buffering
    rx = rx_main()
    rx.run()