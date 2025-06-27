from amaranth                             import *
from amaranth.lib                         import data, stream, wiring
from amaranth.lib.wiring                  import In, Out

from luna.gateware.stream.future          import Packet

class UAC2StreamToSamples(wiring.Component):
    """ Serialize an UAC 2.0 Audio Stream to Samples """

    def __init__(self, bit_depth, channels, subslot_size):
        self.bit_depth    = bit_depth
        self.channels     = channels
        self.subslot_size = subslot_size

        super().__init__({
            "input"   : In  (stream.Signature(Packet(unsigned(8)))),
            "outputs" : Out (stream.Signature(signed(self.bit_depth))).array(channels),
        })


    def elaborate(self, platform):
        m = Module()

        input_stream   = self.input
        output_streams = self.outputs

        first      = input_stream.payload.first
        channel    = Signal(range(0, self.channels))
        subslot    = Signal(self.subslot_size * 8)
        sample     = Signal(self.bit_depth)
        got_sample = Signal()
        error      = Signal()

        # always receive audio from host
        m.d.comb += input_stream.ready .eq(1) # stream.ready driven by the consumer

        out_ready = output_streams[0].ready | output_streams[1].ready

        # De-serialize byte stream to samples
        with m.If(input_stream.valid & out_ready):
            with m.FSM(domain="usb") as fsm:
                with m.State("B0"):
                    with m.If(first):
                        m.d.usb += channel.eq(0)
                    with m.Else():
                        m.d.usb += channel.eq(channel + 1)

                    m.d.usb += subslot[0:8].eq(input_stream.payload.data)
                    m.next = "B1"

                with m.State("B1"):
                    with m.If(first):
                        m.next = "ERROR"

                    with m.Else():
                        m.d.usb += subslot[8:16].eq(input_stream.payload.data)
                        m.next = "B2"

                with m.State("B2"):
                    with m.If(first):
                        m.next = "ERROR"

                    with m.Else():
                        m.d.usb += subslot[16:24].eq(input_stream.payload.data)
                        m.next = "B3"

                with m.State("B3"):
                    with m.If(first):
                        m.next = "ERROR"

                    with m.Else():
                        m.d.usb += sample.eq(Cat(subslot[8:24], input_stream.payload.data))
                        m.d.comb += got_sample.eq(1)
                        m.next = "B0"

                with m.State("ERROR"):
                    m.d.comb += error.eq(1)
                    m.d.usb += channel.eq(0)
                    m.d.usb += subslot[0:8].eq(input_stream.payload.data)
                    m.next = "B1"

        with m.Else():
            m.d.usb += channel.eq(0)
            m.d.usb += subslot.eq(0)

        # dump samples to output streams
        for n in range(self.channels):
            m.d.comb += [
                # stream.valid is driven by the producer
                output_streams[n].valid   .eq(got_sample & (channel == n)),
                output_streams[n].payload .eq(sample),
            ]

        return m




class SamplesToUAC2Stream(wiring.Component):
    """ Serialize Samples to an UAC 2.0 Audio Stream """

    def __init__(self, bit_depth, channels, subslot_size):
        self.bit_depth    = bit_depth
        self.channels     = channels
        self.subslot_size = subslot_size

        super().__init__({
            "inputs" : In  (stream.Signature(signed(self.bit_depth))).array(channels),
            "output" : Out (stream.Signature(unsigned(8))),
        })


    def elaborate(self, platform):
        m = Module()

        input_streams = self.inputs
        output_stream = self.output

        # frame counters
        next_channel = Signal(1)
        next_byte = Signal(2)

        # Subslot Frame Format for 24-bit int with subslot_size=4 is:
        #    00:08  - lsb
        #    08:15  -
        #    16:23  - msb
        #    24:31  - padding
        subslot = Signal(32)

        with m.If(next_channel == 0):
            with m.If(next_byte == 3):
                m.d.comb += input_streams[0].ready.eq(1)
            m.d.comb += [
                subslot[8:].eq(input_streams[0].payload)
            ]
        with m.Else():
            with m.If(next_byte == 3):
                m.d.comb += input_streams[1].ready.eq(1)
            m.d.comb += [
                subslot[8:].eq(input_streams[1].payload)
            ]

        m.d.comb += [
            output_stream.valid.eq(1), # driven by producer (moi-même)
            output_stream.payload.eq(subslot.word_select(next_byte, 8)),
        ]
        with m.If(output_stream.ready):
            m.d.usb += next_byte.eq(next_byte + 1)
            with m.If(next_byte == 3):
                m.d.usb += next_channel.eq(~next_channel)


        return m
