#!/usr/bin/env python3
#
# Copyright (c) 2025 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import logging
import os

from amaranth                             import *
from amaranth.lib                         import wiring
from amaranth.lib.memory                  import Memory

from luna                                 import top_level_cli

from .clockgen                            import ClockGen
from .dac                                 import DAC
from .nco                                 import NCO, sinusoid_lut
from .uac                                 import USBAudioClass2Device

class Top(Elaboratable):
    def __init__(self):
        self.clock_frequencies = {
            "fast": 240,
            "sync": 120,
            "usb":  60,
        }
        self.sync_clk_freq = self.clock_frequencies["sync"] * 1e6

        self.sample_rate         = 48e3
        self.bit_depth           = 24
        self.channels            = 2
        self.lut_length          = 2048
        self.dac_modulation_freq = 12e6
        self.dac_sample_rate     =  3e6

        self.dac_pulse_cycles  = ClockGen.derive(
            clock_name = "modulation",
            input_hz   = self.sync_clk_freq,
            output_hz  = self.dac_modulation_freq,
            logger     = logging,
        )
        self.dac_sample_cycles = ClockGen.derive(
            clock_name = "sampling",
            input_hz   = self.sync_clk_freq,
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
            init   = sinusoid_lut(self.bit_depth, self.lut_length, signed=True),
        )

        # Instantiate our NCOs.
        m.submodules.nco0 = nco0 = DomainRenamer({"sync": "usb"})(NCO(lut))
        m.submodules.nco1 = nco1 = DomainRenamer({"sync": "usb"})(NCO(lut))
        sys_clk_freq = int(60e6)
        phi0_delta   = int(220. * nco0.phi_tau / sys_clk_freq)
        phi1_delta   = int(440. * nco1.phi_tau / sys_clk_freq)
        m.d.usb += [
            nco0.phi_delta.eq(phi0_delta),
            nco1.phi_delta.eq(phi1_delta),
        ]

        # TODO Instantiate our VU meter.

        # Instantiate our DACs.
        m.submodules.dac0 = dac0 = DomainRenamer({"sync": "sync"})(
            DAC(
                pulse_cycles  = self.dac_pulse_cycles,
                sample_cycles = self.dac_sample_cycles,
                width         = self.bit_depth // 8, # 3 bytes = 24 bits
                signed        = True,
            )
        )

        # Connect our NCO's to our UAC device's inputs.
        m.d.comb += [
            uac.input0.eq(nco0.output),
            uac.input1.eq(nco1.output),
        ]

        # Connect the UAC device's outputs to Cynthion USER LEDs.
        # TODO VU Meter
        leds: Signal(6) = Cat(platform.request("led", n).o for n in range(0, 6))
        m.d.comb += leds.eq(uac.output0[2:8])

        # Connect the UAC device's outputs to our ∆Σ DAC inputs.
        m.d.comb += [
            #dac0.input.eq(uac.output0),
        ]

        # Alt: Connect an oscillator to our ∆Σ DAC.
        m.submodules.nco2 = nco2 = DomainRenamer({"sync": "sync"})(NCO(lut))
        phi2_delta   = int(440. * nco2.phi_tau / self.sync_clk_freq)
        m.d.comb += nco2.phi_delta.eq(phi2_delta)
        m.d.comb += [
            dac0.input.eq(nco2.output),
        ]

        # Connect the our ∆Σ DAC outputs to our USER PMOD pins.
        pmod0 = platform.request("user_pmod", 0)
        pmod1 = platform.request("user_pmod", 1)
        m.d.comb += [
            pmod0.oe.eq(1),
            pmod0.o[0].eq(dac0.output),
            pmod0.o[1].eq(dac0.output),
        ]


        # Debug
        #
        m.d.comb += [
        ]

        return m


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    top_level_cli(Top)
