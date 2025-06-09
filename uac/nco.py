import math

from amaranth             import *
from amaranth.lib         import stream, wiring
from amaranth.lib.wiring  import In, Out
from amaranth.lib.memory  import Memory
from amaranth.utils       import log2_int

from amaranth.sim         import *


# - lut generation ------------------------------------------------------------

TAU = math.pi * 2.

def fsin(x, fs, phi=0):
    T = 1.0 / float(fs)
    w = TAU
    return math.cos(w * x * T + phi)

def sinusoid_lut(bit_depth, length, gain=1.0, signed=False):
    fs = length
    scale = math.pow(2, bit_depth) - 1

    ys = [fsin(x, fs) for x in range(int(fs))]

    # scale signal to integer range
    ys = [y * (scale/2) for y in ys]

    # optional: convert to unsigned
    if not signed:
        ys = [y + (scale/2) for y in ys]

    # signal gain
    ys = [y * gain for y in ys]

    # convert to integer
    ys = [int(y) for y in ys]

    return ys


# - gateware ------------------------------------------------------------------

class NCO(wiring.Component):
    def __init__(self, lut, twos_complement=False):
        # create a read port for the lut
        self.read_port0 = lut.read_port(domain="comb")
        self.read_port1 = lut.read_port(domain="comb")

        # calculate accumulator parameters
        self.phi_bits   = 32
        self.phi_tau    = 1 << self.phi_bits
        self.index_bits = log2_int(lut.depth)

        super().__init__({
            "phi_delta" : In  (signed(self.phi_bits)), # frequency (in terms of phi_tau)
            "output"    : Out (stream.Signature(lut.shape)),
        })

    def elaborate(self, platform):
        m = Module()

        stream = self.output

        # accumulator
        phi = Signal(self.phi_bits)

        # lut indices for current and next sample
        index0 = Signal(self.index_bits)
        index1 = Signal(self.index_bits)

        # connect stream to lut
        m.d.comb += [
            self.read_port0.addr .eq(index0),
            self.read_port1.addr .eq(index1),
            stream.valid         .eq(1), # driven by producer (us)
            stream.payload       .eq((self.read_port0.data + self.read_port1.data) >> 1),
        ]

        # for each sample
        with m.If(stream.ready):
            # increment phase and update lut indices
            m.d.sync += [
                phi.eq(phi + self.phi_delta),
                index0.eq(phi[-self.index_bits:]),
                index1.eq(index0 + 1),
            ]

        return m
