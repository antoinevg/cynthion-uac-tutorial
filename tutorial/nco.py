import math

from amaranth             import *
from amaranth.lib         import wiring
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

def sinusoid_lut(bit_depth, length, gain=1, signed=False, twos_complement=False):
    fs = length
    scale = math.pow(2, bit_depth) - 1

    ys = [fsin(x, fs) for x in range(int(fs))]

    # scale signal to integer range
    ys = [y * (scale/2) for y in ys]

    # optional: convert to unsigned
    if not signed:
        ys = [y + (scale/2) for y in ys]

    # signal gain
    #gain = 0.872
    #gain = 0.794328 # -2dB
    #gain = 0.794328 # -2dB
    #gain = 0.501187 # -6dB
    ys = [y * gain for y in ys]

    # convert to integer
    ys = [int(y) for y in ys]

    # 2s complement - I think amaranth is natively 2's complement so this may not be needed
    if twos_complement:
        print("converting nco data to two's complement")
        ys = [twos_comp(y, 24) for y in ys]

    return ys

def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val


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
        #self.fract_bits = self.phi_bits - self.index_bits

        super().__init__({
            # TODO add lut read port
            "phi_delta" : In (unsigned(self.phi_bits)), # frequency (in terms of phi_tau)
            "output"    : Out(lut.shape),               # lut sample
        })

    def elaborate(self, platform):
        m = Module()

        # accumulator
        phi = Signal(self.phi_bits)
        m.d.sync += phi.eq(phi + self.phi_delta)

        # calculate indices of current and next sample
        index0 = Signal(self.index_bits)
        index1 = Signal(self.index_bits)
        #indexf = Signal(self.fract_bits)
        #print(f"index_bits: {self.index_bits}  fract_bits: {self.fract_bits}")
        m.d.sync += index0.eq(phi[-self.index_bits:])
        m.d.sync += index1.eq(index0 + 1)
        #m.d.sync += indexf.eq(phi[:self.fract_bits])

        # select and output sample from lut
        m.d.sync += self.read_port0.addr.eq(index0)
        m.d.sync += self.read_port1.addr.eq(index1)
        #m.d.sync += self.output.eq((self.read_port0.data + self.read_port1.data) >> 1)
        m.d.sync += self.output.eq(self.read_port0.data)

        return m
