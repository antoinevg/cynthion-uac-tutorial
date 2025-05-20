import logging

from amaranth  import *
from luna      import top_level_cli

from .ep1_uac2     import USBAudioClass2Device


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

        return m


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    top_level_cli(Top)
