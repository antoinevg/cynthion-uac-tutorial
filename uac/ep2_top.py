#!/usr/bin/env python3
#
# Copyright (c) 2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import logging
import os

from amaranth                             import *
from amaranth.lib                         import fifo, wiring
from amaranth.lib.memory                  import Memory

from luna                                 import top_level_cli

from .clockgen                            import ClockGen
from .dac                                 import DAC
from .nco                                 import NCO, sinusoid_lut
from .uac                                 import USBAudioClass2Device
from .vu                                  import VU

class Top(Elaboratable):
    def __init__(self):
        self.clock_frequencies = {
            "fast": 240,
            "sync": 120,
            "usb":  60,
        }

        self.sample_rate         = 48e3
        #self.sample_rate         = 768e3
        self.bit_depth           = 24
        self.channels            = 2
        self.lut_length          = 32768
        self.dac_clk_freq        = self.clock_frequencies["usb"]  * 1e6
        self.dac_modulation_freq = 30e6 # pulse  cycles
        self.dac_sample_rate     = 48e3 # sample cycles

        self.dac_pulse_cycles  = ClockGen.derive(
            clock_name = "modulation",
            input_hz   = self.dac_clk_freq,
            output_hz  = self.dac_modulation_freq,
            logger     = logging,
        )
        self.dac_sample_cycles = ClockGen.derive(
            clock_name = "sampling",
            input_hz   = self.dac_clk_freq,
            output_hz  = self.dac_sample_rate,
            logger     = logging,
            max_deviation_ppm = 0,
        )

    def elaborate(self, platform):
        m = Module()

        # Generate our clock domains and resets.
        m.submodules.car = platform.clock_domain_generator(clock_frequencies=self.clock_frequencies)

        # Instantiate our UAC Device.
        m.submodules.uac = uac = USBAudioClass2Device(
            sample_rate = self.sample_rate,
            bit_depth   = self.bit_depth,
            channels    = self.channels
        )

        # Instantiate our sin LUT.
        m.submodules.lut = lut = Memory(
            shape  = signed(self.bit_depth),
            depth  = self.lut_length,
            init   = sinusoid_lut(self.bit_depth, self.lut_length, gain=0.501187, signed=True),
        )
        #gain = 0.794328 # -2dB
        #gain = 0.501187 # - 6dB

        # Instantiate our NCOs.
        m.submodules.nco0 = nco0 = DomainRenamer({"sync": "usb"})(NCO(lut))
        m.submodules.nco1 = nco1 = DomainRenamer({"sync": "usb"})(NCO(lut))
        phi0_delta   = int(100. * nco0.phi_tau / self.dac_clk_freq)
        # TODO why go crazy when > 4096 when my LUT is 32768 samples?
        phi1_delta   = int(1000. * nco1.phi_tau / self.dac_clk_freq)
        m.d.usb += [
            nco0.phi_delta.eq(phi0_delta),
            nco1.phi_delta.eq(phi1_delta),
        ]

        # Instantiate our VU meter.
        m.submodules.vu = vu = DomainRenamer({"sync": "usb"})(VU(bit_depth=self.bit_depth))

        # Instantiate our DACs.
        m.submodules.dac = dac = DomainRenamer({"sync": "usb"})(
            DAC(
                pulse_cycles  = self.dac_pulse_cycles,
                sample_cycles = self.dac_sample_cycles,
                width         = self.bit_depth // 8, # 3 bytes = 24 bits
                signed        = True,
            )
        )


        # - connections ---

        # Connect our UAC device's inputs to our NCO's.
        m.d.comb += [
            uac.input0.eq(nco0.output),
            uac.input1.eq(nco1.output),
        ]

        # Connect our UAC device's outputs to our ∆Σ DAC's inputs
        m.submodules.dac_fifo = dac_fifo = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=128)
        m.d.comb += [
            dac_fifo.w_en   .eq(uac.ordy0 & dac_fifo.w_rdy),
            dac_fifo.w_data .eq(uac.output0),
            dac_fifo.r_en.eq(dac.latch & dac_fifo.r_rdy),
            dac.input.eq(dac_fifo.r_data),
        ]

        # Alternatively: Connect an oscillator to our ∆Σ DAC.
        m.submodules.nco2 = nco2 = DomainRenamer({"sync": "usb"})(NCO(lut))
        phi2_delta   = int(1000. * nco2.phi_tau / self.dac_clk_freq)
        m.d.comb += nco2.phi_delta.eq(phi2_delta)
        m.d.comb += [
            #dac.input.eq(nco2.output),
        ]

        # Connect our ∆Σ DAC outputs to our USER PMOD pins.
        pmod1 = platform.request("user_pmod", 1)
        m.d.comb += [
            pmod1.oe.eq(1),
            pmod1.o[0].eq(dac.output),
            pmod1.o[1].eq(dac.output),
        ]

        # Connect the UAC device's outputs to our VU meter.
        m.submodules.vu_fifo = vu_fifo = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=128)
        m.d.comb += [
            vu_fifo.w_en   .eq(uac.ordy0 & vu_fifo.w_rdy),
            vu_fifo.w_data .eq(uac.output0),
            vu_fifo.r_en.eq(dac.latch & vu_fifo.r_rdy),
            vu.input.eq(vu_fifo.r_data),
            vu.latch.eq(dac.latch),
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

        # Debug: Signals
        pmod0 = platform.request("user_pmod", 0)
        m.d.comb += [
            pmod0.oe.eq(1),
            pmod0.o[0:3].eq(dac.debug[0:3]),
            pmod0.o[3].eq(dac_fifo.w_en),
            pmod0.o[4].eq(dac_fifo.r_en),
            pmod0.o[5:8].eq(uac.debug[5:8]),
        ]

        return m


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    top_level_cli(Top)
