import logging

from amaranth             import *
from amaranth.lib         import fifo, stream, wiring
from amaranth.lib.wiring  import In, Out

from .clockgen            import ClockGen

class Channel(wiring.Component):
    def __init__(self, bit_depth=16, signed=False):
        super().__init__({
            "input"  : In  (bit_depth),
            "output" : Out (1),
        })

        self.bit_depth   = bit_depth
        self.signed = signed

        self.stb    = Signal()
        self.update = Signal()

    def elaborate(self, platform):
        m = Module()

        input_u = Signal(self.bit_depth)
        if self.signed:
            m.d.comb += input_u.eq(self.input - (1 << (self.bit_depth - 1)))
        else:
            m.d.comb += input_u.eq(self.input)

        accum   = Signal(self.bit_depth)
        input_r = Signal(self.bit_depth)

        with m.If(self.stb):
            m.d.sync += Cat(accum, self.output).eq(accum + input_r)
        with m.If(self.update):
            m.d.sync += input_r.eq(input_u)

        return m


class DAC(wiring.Component):
    def __init__(self, sample_rate, bit_depth, channels, clock_frequency, signed=False):
        super().__init__({
            "inputs"  : In  (stream.Signature(bit_depth)).array(channels),
            "outputs" : Out (channels),
            "latch"   : Out (1),
        })

        modulation_freq    = 30e6 # pulse  cycles

        self.bit_depth     = bit_depth
        self.signed        = signed

        self.pulse_cycles  = ClockGen.derive(
            clock_name = "modulation",
            input_hz   = clock_frequency,
            output_hz  = modulation_freq,
            logger     = logging,
        )
        self.sample_cycles = ClockGen.derive(
            clock_name = "sampling",
            input_hz   = clock_frequency,
            output_hz  = sample_rate,
            logger     = logging,
            max_deviation_ppm = 0,
        )

        self.clock         = ClockGen(self.pulse_cycles)
        self.fifo_0        = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=16)
        self.fifo_1        = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=16)


    def elaborate(self, platform):
        m = Module()

        print(f"pulse_cycles:  {self.pulse_cycles}")
        print(f"sample_cycles: {self.sample_cycles}")

        m.submodules.clock  = clock   = self.clock
        m.submodules.fifo_0 = fifo_0  = self.fifo_0
        m.submodules.fifo_1 = fifo_1  = self.fifo_1

        m.submodules.channel_0 = channel_0 = Channel(bit_depth=self.bit_depth, signed=self.signed)
        m.submodules.channel_1 = channel_1 = Channel(bit_depth=self.bit_depth, signed=self.signed)
        m.d.comb += channel_0.stb.eq(clock.stb_r)
        m.d.comb += channel_1.stb.eq(clock.stb_r)

        timer = Signal(range(self.sample_cycles))

        sample_0 = Signal(self.bit_depth)
        sample_1 = Signal(self.bit_depth)
        len_channels = 1

        with m.FSM():
            with m.State("STANDBY"):
                m.next = "WAIT"

            with m.State("WAIT"):
                with m.If(timer == 0):
                    m.d.sync += timer.eq(self.sample_cycles - len_channels * (self.bit_depth // 8) - 1)
                    m.next = "CHANNEL-0-READ-1"
                with m.Else():
                    m.d.sync += timer.eq(timer - 1)

            with m.State("CHANNEL-0-READ-1"):
                m.d.sync += channel_0.input.eq(sample_0)
                m.d.sync += channel_1.input.eq(sample_1)
                m.next = "LATCH"

            with m.State("LATCH"):
                m.d.comb += self.latch.eq(1)
                m.d.comb += channel_0.update.eq(1)
                m.d.comb += channel_1.update.eq(1)
                m.next = "WAIT"

        # connect input streams to fifo & fifo to channels
        wiring.connect(m, wiring.flipped(self.inputs[0]), fifo_0.w_stream)
        wiring.connect(m, wiring.flipped(self.inputs[1]), fifo_1.w_stream)
        m.d.comb += [
            fifo_0.r_en.eq(self.latch & fifo_0.r_rdy),
            sample_0.eq(fifo_0.r_data),
            fifo_1.r_en.eq(self.latch & fifo_1.r_rdy),
            sample_1.eq(fifo_1.r_data),
        ]

        # connect channel outputs to dac output
        m.d.comb += self.outputs[0].eq(channel_0.output)
        m.d.comb += self.outputs[1].eq(channel_1.output)

        return m
