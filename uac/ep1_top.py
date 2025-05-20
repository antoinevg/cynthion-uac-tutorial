#!/usr/bin/env python3
#
# Copyright (c) 2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import logging

from amaranth             import *
from amaranth.lib         import fifo
from amaranth.lib.memory  import Memory

from luna                 import top_level_cli

from .ep1_uac2            import USBAudioClass2Device
from .nco                 import NCO, sinusoid_lut
from .vu                  import VU


class Top(Elaboratable):
    def __init__(self):
        self.clock_frequencies   = {
            "fast": 240,
            "sync": 120,
            "usb":  60,
        }

        self.sample_rate         = 48e3
        self.bit_depth           = 24
        self.channels            = 2
        self.lut_length          = 32768

    def elaborate(self, platform):
        m = Module()

        # Generate our clock domains and resets.
        m.submodules.car = platform.clock_domain_generator(clock_frequencies=self.clock_frequencies)

        # Instantiate our UAC2 Device.
        m.submodules.uac2 = uac2 = USBAudioClass2Device(
            sample_rate = self.sample_rate,
            bit_depth   = self.bit_depth,
            channels    = self.channels
        )

        # Instantiate our sin LUT.
        #gain = 0.794328 # -2dB
        gain = 0.501187 # - 6dB
        m.submodules.lut = lut = Memory(
            shape  = signed(self.bit_depth),
            depth  = self.lut_length,
            init   = sinusoid_lut(self.bit_depth, self.lut_length, gain=gain, signed=True),
        )

        # Instantiate our NCOs.
        fs = self.clock_frequencies["usb"] * 1e6
        m.submodules.nco0 = nco0 = DomainRenamer({"sync": "usb"})(NCO(lut))
        m.submodules.nco1 = nco1 = DomainRenamer({"sync": "usb"})(NCO(lut))
        phi0_delta   = int(100. * nco0.phi_tau / fs)
        # TODO why go crazy when > 4096 when my LUT is 32768 samples?
        phi1_delta   = int(1000. * nco1.phi_tau / fs)
        m.d.usb += [
            nco0.phi_delta.eq(phi0_delta),
            nco1.phi_delta.eq(phi1_delta),
        ]

        # Instantiate our VU meter.
        m.submodules.vu = vu = DomainRenamer({"sync": "usb"})(VU(bit_depth=self.bit_depth))


        # - connections ---

        # Connect our UAC device's inputs to our NCO's.
        m.d.comb += [
            uac2.input0.eq(nco0.output),
            uac2.input1.eq(nco1.output),
        ]

        # Connect the UAC device's outputs to our VU meter.
        m.submodules.vu_fifo = vu_fifo = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=128)
        m.d.comb += [
            vu_fifo.w_en   .eq(uac2.ordy0 & vu_fifo.w_rdy),
            vu_fifo.w_data .eq(uac2.output0),
            vu_fifo.r_en.eq(vu_fifo.r_rdy),
            vu.input.eq(vu_fifo.r_data),
            vu.latch.eq(vu_fifo.r_rdy),
        ]

        # Connect the VU meter's output to Cynthion USER LEDs.
        def logscale(x):
            U = (2.**24) - 1.
            y = (10.**(x/10.)) / 10.
            l = int(y * U)
            return l
        leds: Signal(6) = Cat(platform.request("led", n).o for n in range(0, 6))
        with m.If(vu.output >= logscale(0)):
            m.d.comb += leds[0].eq(1)
        with m.If(vu.output >= logscale(0.5)):
            m.d.comb += leds[1].eq(1)
        with m.If(vu.output >= logscale(1)):
            m.d.comb += leds[2].eq(1)
        with m.If(vu.output >= logscale(1.5)):
            m.d.comb += leds[3].eq(1)
        with m.If(vu.output >= logscale(2)):
            m.d.comb += leds[4].eq(1)
        with m.If(vu.output >= logscale(3)):
            m.d.comb += leds[5].eq(1)

        return m


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    top_level_cli(Top)
