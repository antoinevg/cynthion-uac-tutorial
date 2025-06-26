from amaranth                             import *

from usb_protocol.emitters.descriptors    import uac2, standard
from usb_protocol.types                   import (
    USBDirection,
    USBRequestRecipient,
    USBRequestType,
    USBStandardRequests,
    USBSynchronizationType,
    USBTransferType,
    USBUsageType,
)
from usb_protocol.types.descriptors.uac2  import AudioClassSpecificRequestCodes

from luna.gateware.stream.generator       import StreamSerializer
from luna.gateware.usb.stream             import USBInStreamInterface
from luna.gateware.usb.usb2.request       import USBRequestHandler


class UAC2RequestHandler(USBRequestHandler):
    """ USB Audio Class Request Handler """

    def __init__(self, sample_rate):
        super().__init__()

        self.sample_rate = int(sample_rate)

    def elaborate(self, platform):
        m = Module()

        interface         = self.interface
        setup             = self.interface.setup

        m.submodules.transmitter = transmitter = StreamSerializer(
            data_length=14,     # The maximum length of data to be sent.
            stream_type=USBInStreamInterface,
            max_length_width=4, # Provides a `max_length` signal to limit total length transmitted
            domain="usb",
        )

        # The requests we'll be handling.
        standard_set_interface = (setup.type == USBRequestType.STANDARD) & \
                                 (setup.recipient == USBRequestRecipient.INTERFACE) & \
                                 (setup.request == USBStandardRequests.SET_INTERFACE)
        uac2_request_range = (setup.type == USBRequestType.CLASS) & \
                             (setup.request == AudioClassSpecificRequestCodes.RANGE)
        uac2_request_cur   = (setup.type == USBRequestType.CLASS) & \
                             (setup.request == AudioClassSpecificRequestCodes.CUR)
        request_clock_freq = (setup.value == 0x100) & (setup.index == 0x0100)

        with m.If(standard_set_interface):
            # Because we have multiple interfaces ('quiet' and 'active' we need
            # to handle SET_INTERFACE ourselves.
            #
            # On a more complex interface we could use this as an opportunity to
            # control pre-amp power or other functionality.

            # claim interface
            if hasattr(interface, "claim"):
                m.d.comb += interface.claim.eq(1)

            # Always ACK the data out...
            with m.If(interface.rx_ready_for_response):
                m.d.comb += interface.handshakes_out.ack.eq(1)

            # ... and accept whatever the request was.
            with m.If(interface.status_requested):
                m.d.comb += self.send_zlp()

        with m.Elif(uac2_request_range & request_clock_freq):
            # Return the valid values for the interface's clock.

            # claim interface
            if hasattr(interface, "claim"):
                m.d.comb += interface.claim.eq(1)

            m.d.comb += transmitter.stream.attach(self.interface.tx)
            m.d.comb += [
                Cat(transmitter.data)   .eq(
                    Cat(
                       Const(0x1, 16),              # num subranges
                       Const(self.sample_rate, 32), # MIN
                       Const(self.sample_rate, 32), # MAX
                       Const(0, 32),                # RES
                    )
                ),
                transmitter.max_length  .eq(setup.length)
            ]

            # ... trigger it to respond when data's requested...
            with m.If(interface.data_requested):
                m.d.comb += transmitter.start.eq(1)

            # ... and ACK our status stage.
            with m.If(interface.status_requested):
                m.d.comb += interface.handshakes_out.ack.eq(1)

        with m.Elif(uac2_request_cur & request_clock_freq):
            # Return the current value of the interface's clock

            # claim interface
            if hasattr(interface, "claim"):
                m.d.comb += interface.claim.eq(1)

            m.d.comb += transmitter.stream.attach(self.interface.tx)
            m.d.comb += [
                Cat(transmitter.data[0:4]).eq(
                    Const(self.sample_rate, 32)
                ),
                transmitter.max_length.eq(4)
            ]

            # ... trigger it to respond when data's requested...
            with m.If(interface.data_requested):
                m.d.comb += transmitter.start.eq(1)

            # ... and ACK our status stage.
            with m.If(interface.status_requested):
                m.d.comb += interface.handshakes_out.ack.eq(1)

        # Stall any unsupported requests.
        with m.Else():
            with m.If(interface.status_requested | interface.data_requested):
                m.d.comb += interface.handshakes_out.stall.eq(1)

        return m
