import logging

from amaranth             import *
from amaranth.lib         import fifo, stream, wiring
from amaranth.lib.wiring  import In, Out

from .clockgen            import ClockGen

class Channel(wiring.Component):
    def __init__(self, bit_depth=16, signed=False):
        super().__init__({
            "input":  In(bit_depth),
            "output": Out(1),
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
        assert (bit_depth // 8) in (1, 2, 3, 4)

        super().__init__({
            "inputs"  : In  (stream.Signature(bit_depth)).array(channels),
            "outputs" : Out (channels),

            "input": In(bit_depth),
            "output": Out(1),
            "latch": Out(1),
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
        self.fifo          = fifo.SyncFIFOBuffered(width=self.bit_depth, depth=128)

        self.debug         = Signal(8)


    def elaborate(self, platform):
        m = Module()

        print(f"pulse_cycles:  {self.pulse_cycles}")
        print(f"sample_cycles: {self.sample_cycles}")

        m.submodules.clock = clock = self.clock
        m.submodules.fifo  = fifo  = self.fifo

        m.submodules.channel_0 = channel_0 = Channel(bit_depth=self.bit_depth, signed=self.signed)
        m.d.comb += self.output.eq(channel_0.output)
        m.d.comb += channel_0.stb.eq(clock.stb_r)

        timer = Signal(range(self.sample_cycles))

        sample = Signal(self.bit_depth)
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
                #m.d.sync += channel_0.input.eq(self.input)
                m.d.sync += channel_0.input.eq(sample)
                m.next = "LATCH"

            with m.State("LATCH"):
                m.d.comb += self.latch.eq(1)
                m.d.comb += channel_0.update.eq(1)
                m.next = "WAIT"

        # connect input streams to fifo
        m.d.comb += [
            fifo.w_en   .eq(self.inputs[0].valid & fifo.w_rdy),
            fifo.w_data .eq(self.inputs[0].payload),
            fifo.r_en.eq(self.latch & fifo.r_rdy),
            sample.eq(fifo.r_data),
        ]

        # debug
        m.d.comb += [
            self.debug[0].eq(ClockSignal("sync")),
            self.debug[1].eq(timer == 0),
            self.debug[2].eq(channel_0.update),
        ]


        return m
