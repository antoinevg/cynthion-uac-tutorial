from amaranth             import *
from amaranth.lib         import wiring
from amaranth.lib.wiring  import In, Out

from .                    import clockgen


class VU(wiring.Component):
    def __init__(self, bit_depth=24):
        super().__init__({
            "input"  : In(signed(bit_depth)),
            "output" : Out(unsigned(bit_depth)),
        })

        self.latch     = Signal()
        self.bit_depth = bit_depth

    def elaborate(self, platform):
        m = Module()

        U  = (2**self.bit_depth) - 1
        dP    = int(0.0001   * U)
        kN = int(0.000001 * U)
        print(f"dP:{dP}  kN:{kN}")

        c  = Signal.like(self.output)
        x1 = Signal.like(self.output)

        m.d.comb += x1.eq(abs(self.input))
        m.d.comb += self.output.eq(x1)

        with m.If(self.latch):
            with m.If(x1 > c):
                m.d.sync += c.eq(c * (U - dP) + (x1 * dP))
            with m.Else():
                m.d.sync += c.eq(c * (U - kN))

        return m
