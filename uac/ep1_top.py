#!/usr/bin/env python3
#
# Copyright (c) 2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import logging

from amaranth             import *
from amaranth.lib         import fifo, wiring
from amaranth.lib.memory  import Memory

from luna                 import top_level_cli

from .ep1_uac2            import USBAudioClass2Device
from .nco                 import NCO, sinusoid_lut
from .vu                  import VU


from .dac                 import DAC      # TODO


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

        self.lut_length          = 256


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
        #gain  = 1.0
        gain = 0.794328 # -2dB
        #gain = 0.501187 # - 6dB
        m.submodules.lut = lut = Memory(
            shape  = signed(self.bit_depth),
            depth  = self.lut_length,
            init   = sinusoid_lut(self.bit_depth, self.lut_length, gain=gain, signed=True),
        )

        # Instantiate our NCOs.
        fs = self.sample_rate
        m.submodules.nco0 = nco0 = DomainRenamer({"sync": "usb"})(NCO(lut))
        m.submodules.nco1 = nco1 = DomainRenamer({"sync": "usb"})(NCO(lut))
        m.d.comb += [
            nco0.phi_delta.eq(int(1000. * nco0.phi_tau / fs)),
            nco1.phi_delta.eq(int(10000. * nco1.phi_tau / fs)),
        ]

        # Connect our UAC device's inputs to our NCO's.
        wiring.connect(m, nco0.output, uac2.inputs[0])
        wiring.connect(m, nco1.output, uac2.inputs[1])


        # - DAC ---------------------------------------------------------------
        # Instantiate our DACs.
        m.submodules.dac = dac = DomainRenamer({"sync": "usb"})(
            DAC(
                sample_rate     = self.sample_rate,
                bit_depth       = self.bit_depth,
                channels        = self.channels,
                clock_frequency = self.clock_frequencies["usb"]  * 1e6,
                signed          = True,
            )
        )

        # Connect our UAC device's outputs to our ∆Σ DAC's inputs
        wiring.connect(m, uac2.outputs[0], dac.inputs[0])
        wiring.connect(m, uac2.outputs[1], dac.inputs[1])

        # Connect our ∆Σ DAC outputs to our USER PMOD pins.
        pmod1 = platform.request("user_pmod", 1)
        m.d.comb += [
            pmod1.oe.eq(1),
            pmod1.o[0].eq(dac.outputs[0]),
            pmod1.o[1].eq(dac.outputs[1]),
        ]
        # ---------------------------------------------------------------------


        # Instantiate our VU meter.
        # m.submodules.vu = vu = DomainRenamer({"sync": "usb"})(VU(bit_depth=self.bit_depth))

        # Connect the UAC device's outputs to our VU meter.
        # m.submodules.vu_fifo = vu_fifo = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=128)
        # m.d.comb += [
        #     vu_fifo.w_en   .eq(uac2.ordy0 & vu_fifo.w_rdy),
        #     vu_fifo.w_data .eq(uac2.output0),
        #     vu_fifo.r_en.eq(vu_fifo.r_rdy),
        #     vu.input.eq(vu_fifo.r_data),
        #     vu.latch.eq(vu_fifo.r_rdy),
        # ]

        # # Connect the VU meter's output to Cynthion USER LEDs.
        # def logscale(x):
        #     U = (2.**24) - 1.
        #     y = (10.**(x/10.)) / 10.
        #     l = int(y * U)
        #     return l
        # leds: Signal(6) = Cat(platform.request("led", n).o for n in range(0, 6))
        # with m.If(vu.output >= logscale(0)):
        #     m.d.comb += leds[0].eq(1)
        # with m.If(vu.output >= logscale(0.5)):
        #     m.d.comb += leds[1].eq(1)
        # with m.If(vu.output >= logscale(1)):
        #     m.d.comb += leds[2].eq(1)
        # with m.If(vu.output >= logscale(1.5)):
        #     m.d.comb += leds[3].eq(1)
        # with m.If(vu.output >= logscale(2)):
        #     m.d.comb += leds[4].eq(1)
        # with m.If(vu.output >= logscale(3)):
        #     m.d.comb += leds[5].eq(1)

        return m


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    top_level_cli(Top)
