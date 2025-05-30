import logging

from amaranth             import *
from amaranth.lib         import fifo, stream, wiring
from amaranth.lib.wiring  import In, Out
from amaranth.utils       import log2_int, ceil_log2

from .clockgen            import ClockGen



class VU(wiring.Component):
    def __init__(self, sample_rate, bit_depth, clock_frequency, segments):
        super().__init__({
            "input"  : In  (stream.Signature(signed(bit_depth))),
            "output" : Out (unsigned(bit_depth)),
            "leds"   : Out (segments),
        })

        self.bit_depth = bit_depth
        self.segments  = segments

        self.sample_cycles = ClockGen.derive(
            clock_name = "sample",
            input_hz   = clock_frequency,
            output_hz  = sample_rate,
            logger     = logging,
            max_deviation_ppm = 0,
        )

        self.clock = ClockGen(self.sample_cycles)
        self.fifo  = fifo.SyncFIFOBuffered(width=bit_depth, depth=16)


    def logscale(self, x):
        """ scale an input value logarithmically """
        U = (2. ** (self.bit_depth - 1)) - 1.
        y = (self.segments ** (x / self.segments)) / self.segments
        l = int(y * U)
        return l


    def elaborate(self, platform):
        m = Module()

        m.submodules.clock = clock = self.clock
        m.submodules.fifo  = fifo  = self.fifo

        # connect input to fifo
        wiring.connect(m, wiring.flipped(self.input), fifo.w_stream)

        # always accept data from producer
        m.d.comb += self.input.ready.eq(1)

        # calculate vu raw output signal
        with m.If(clock.stb_r):
            m.d.comb += fifo.r_en.eq(fifo.r_rdy)
            m.d.sync += self.output.eq(abs(fifo.r_data.as_signed()))

            # led control
            with m.If(self.output >= self.logscale(5)):
                m.d.sync += self.leds.eq(0b111111)
            with m.Elif(self.output >= self.logscale(4)):
                m.d.sync += self.leds.eq(0b011111)
            with m.Elif(self.output >= self.logscale(3)):
                m.d.sync += self.leds.eq(0b001111)
            with m.Elif(self.output >= self.logscale(2)):
                m.d.sync += self.leds.eq(0b000111)
            with m.Elif(self.output >= self.logscale(1)):
                m.d.sync += self.leds.eq(0b000011)
            with m.Elif(self.output > self.logscale(0)):
                m.d.sync += self.leds.eq(0b000001)
            with m.Else():
                m.d.sync += self.leds.eq(0b000000)

        return m
