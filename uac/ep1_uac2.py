import logging
import os
import sys

from amaranth                             import *
from amaranth.lib                         import data, stream, wiring
from amaranth.lib.wiring                  import In, Out

from usb_protocol.emitters                import DeviceDescriptorCollection
from usb_protocol.emitters.descriptors    import uac2, standard
from usb_protocol.types                   import (
    USBDirection,
    USBRequestType,
    USBStandardRequests,
    USBSynchronizationType,
    USBTransferType,
    USBUsageType,
)

from luna.usb2                            import (
    USBDevice,
    USBIsochronousInEndpoint,
    USBIsochronousStreamInEndpoint,
    USBIsochronousStreamOutEndpoint,
)
from luna.gateware.usb.usb2.request       import StallOnlyRequestHandler

from .ep1_stream  import UAC2StreamToSamples, UAC2SamplesToStream
from .ep1_request import UAC2RequestHandler


class USBAudioClass2Device(wiring.Component):
    """ USB Audio Class 2 Audio Interface Device """

    def __init__(self, sample_rate, bit_depth, channels, bus):
        self.sample_rate = sample_rate
        self.bit_depth   = bit_depth
        self.channels    = channels
        self.bus         = bus

        if self.bit_depth == 24:
            self.subslot_size = 4
        elif bit_depth in [8, 16, 32]:
            self.subslot_size = bit_depth / 8
        else:
            logging.error(f"Invalid bit_depth '{bit_depth}'. Supported values are 8, 16, 24, 32")
            sys.exit(1)

        samples_per_microframe = self.sample_rate / 8000
        bytes_per_microframe   = samples_per_microframe * self.subslot_size * self.channels
        logging.info(f"bytes_per_microframe: {bytes_per_microframe}")

        self.bytes_per_microframe = int(bytes_per_microframe)
        if self.bytes_per_microframe > 1024:
            logging.error(f"Configuration requires > 1024 bytes per microframe: {self.bytes_per_microframe}")
            sys.exit(1)

        super().__init__({
            "inputs"  : In  (stream.Signature(signed(self.bit_depth))).array(channels),
            "outputs" : Out (stream.Signature(signed(self.bit_depth))).array(channels),
        })


    def elaborate(self, platform):
        m = Module()

        # Create our USB device interface.
        m.submodules.usb = usb = USBDevice(bus=self.bus)

        # Connect our device as a high speed device.
        m.d.comb += [
            usb.connect          .eq(1),
            usb.full_speed_only  .eq(0),
        ]

        # Add our standard control endpoint to the device.
        descriptors = self.create_descriptors()
        ep_control = usb.add_control_endpoint()
        ep_control.add_standard_request_handlers(descriptors, blacklist=[
            # We have multiple interfaces so we will need to handle
            # SET_INTERFACE ourselves.
            lambda setup: (setup.type == USBRequestType.STANDARD) &
                          (setup.request == USBStandardRequests.SET_INTERFACE)
        ])

        # Attach our class request handlers.
        ep_control.add_request_handler(UAC2RequestHandler(sample_rate=self.sample_rate))

        # Attach class-request handlers that stall any vendor or reserved requests,
        # as we don't have or need any.
        stall_condition = lambda setup : \
            (setup.type == USBRequestType.VENDOR) | \
            (setup.type == USBRequestType.RESERVED)
        ep_control.add_request_handler(StallOnlyRequestHandler(stall_condition))

        # Add endpoints.
        # EP 0x01 OUT - audio from host
        ep1_out = USBIsochronousStreamOutEndpoint(
            endpoint_number=1,
            max_packet_size=self.bytes_per_microframe + 1,
        )
        usb.add_endpoint(ep1_out)

        # EP 0x82 IN - feedback to host
        ep2_in = USBIsochronousInEndpoint(
            endpoint_number=2,
            max_packet_size=4,
        )
        usb.add_endpoint(ep2_in)

        # EP 0x83 IN - audio to host
        ep3_in = USBIsochronousStreamInEndpoint(
            endpoint_number=3,
            max_packet_size=self.bytes_per_microframe + 1,
        )
        usb.add_endpoint(ep3_in)

        # Configure endpoints.
        m.d.comb += [
            ep2_in.bytes_in_frame.eq(4),  # feedback is 32 bits = 4 bytes
            ep3_in.bytes_in_frame.eq(self.bytes_per_microframe), # fs / 8000 * subslot_size * channels
        ]


        # - ep1_out - audio from host -----------------------------------------

        first      = ep1_out.stream.payload.first
        channel    = Signal(range(0, self.channels))
        subslot    = Signal(32)
        sample     = Signal(self.bit_depth)
        got_sample = Signal()
        error      = Signal()

        # always receive audio from host
        m.d.comb += ep1_out.stream.ready .eq(1) # driven by me, the ep1_out consumer

        out_ready = self.outputs[0].ready | self.outputs[1].ready

        # state machine for receiving host audio
        with m.If(ep1_out.stream.valid & out_ready):
            with m.FSM(domain="usb") as fsm:
                with m.State("B0"):
                    with m.If(first):
                        m.d.usb += channel.eq(0)
                    with m.Else():
                        m.d.usb += channel.eq(channel + 1)

                    m.d.usb += subslot[0:8].eq(ep1_out.stream.payload.data)      # byte0  TODO use word_select
                    m.next = "B1"

                with m.State("B1"): # ...
                    with m.If(first):
                        m.next = "ERROR"

                    with m.Else():
                        m.d.usb += subslot[8:16].eq(ep1_out.stream.payload.data) # byte1
                        m.next = "B2"

                with m.State("B2"): # ...
                    with m.If(first):
                        m.next = "ERROR"

                    with m.Else():
                        m.d.usb += subslot[16:24].eq(ep1_out.stream.payload.data) # byte2
                        m.next = "B3"

                with m.State("B3"):
                    with m.If(first):
                        m.next = "ERROR"

                    with m.Else():
                        m.d.usb += sample.eq(Cat(subslot[8:24], ep1_out.stream.payload.data)) # byte3
                        m.d.comb += got_sample.eq(1)
                        m.next = "B0"

                with m.State("ERROR"):
                    m.d.comb += error.eq(1)
                    m.d.usb += channel.eq(0)
                    m.d.usb += subslot[0:8].eq(ep1_out.stream.payload.data)       # byte0
                    m.next = "B1"

        with m.Else():
            m.d.usb += channel.eq(0)
            m.d.usb += subslot.eq(0)

        # dump samples to output streams
        output_streams = self.outputs
        m.d.comb += [
            output_streams[0].valid   .eq(got_sample & (channel == 0)), # driven by producer (that would be me)
            output_streams[0].payload .eq(sample),
            output_streams[1].valid   .eq(got_sample & (channel == 1)), # driven by producer (that would be me)
            output_streams[1].payload .eq(sample),
        ]


        # - ep2_in - feedback to host -----------------------------------------

        # USB2.0 Section 5.12.4.2, Feedback
        #
        # "For high-speed endpoints, the Ff value shall be encoded in
        #  an unsigned 12.13 (K=13) format which fits into four
        #  bytes. The value shall be aligned into these four bytes so
        #  that the binary point is located between the second and the
        #  third byte so that it has a 16.16 format. The most
        #  significant four bits shall be reported zero. Only the first
        #  13 bits behind the binary point are required. The lower
        #  three bits may be optionally used to extend the precision of
        #  Ff, otherwise, they shall be reported as zero."
        #
        # Effectively that works out to: Q12.16

        # 44.1k = 5.5125 samples / microframe
        # 5.5125 * 2^14 = ??
        # Presonus is: 40, 83, 05, 00  aka 0x00 05 8340
        # hex(int(5.5125 * (2 ** 16))) = 0x05_8333

        # 48k = 6 samples / microframe
        # hex(int(6 * (2 ** 16))) = 0x06_0000
        # Presonus is: 10, 00, 06, 00  aka 0x00 06 0010
        # Mine     is: 00, 00, 06, 00  aka 0x00 06 0000

        microframes_per_second = 8000 # 1 000 000 / 125
        samples_per_microframe = self.sample_rate / microframes_per_second
        feedback = round(samples_per_microframe * (2**16))
        logging.info(f"samples_per_microframe: {samples_per_microframe}")
        logging.info(f"feedback_value: {hex(feedback)}")

        feedbackValue = Signal(32)
        bitPos        = Signal(5)

        m.d.comb += feedbackValue.eq(int(samples_per_microframe * (2 << 16)))

        m.d.comb += [
            bitPos.eq(ep2_in.address << 3),
            ep2_in.value.eq(0xff & (feedbackValue >> bitPos)),
        ]


        # - ep3_in - audio to host --------------------------------------------

        # input streams
        input_streams = self.inputs

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
            ep3_in.stream.valid.eq(1), # driven by producer (moi-même)
            ep3_in.stream.payload.eq(subslot.word_select(next_byte, 8)),
        ]
        with m.If(ep3_in.stream.ready):
            m.d.usb += next_byte.eq(next_byte + 1)
            with m.If(next_byte == 3):
                m.d.usb += next_channel.eq(~next_channel)

        return m


    def create_descriptors(self):
        """ Create the descriptors we want to use for our device. """

        descriptors = DeviceDescriptorCollection()

        with descriptors.DeviceDescriptor() as d:
            d.idVendor           = 0x1209 # https://pid.codes/1209/
            d.idProduct          = 0x0001 # pid.codes Test PID 1

            d.iManufacturer      = "LUNA"
            d.iProduct           = "USB Audio Class 2 Device Tutorial"
            d.iSerialNumber      = "no serial"

            d.bDeviceClass       = 0xef # Miscellaneous
            d.bDeviceSubclass    = 0x02 # Use Interface Association Descriptor
            d.bDeviceProtocol    = 0x01 # Use Interface Association Descriptor

            d.bNumConfigurations = 1

        with descriptors.ConfigurationDescriptor() as configuration:

            # Interface association descriptor
            configuration.add_subordinate_descriptor(uac2.InterfaceAssociationDescriptor.build({
                    "bInterfaceCount" : 3, # audio control, audio from host, audio to host
                })
            )


            # - Interface #0: Standard audio control interface descriptor --

            configuration.add_subordinate_descriptor(
                uac2.StandardAudioControlInterfaceDescriptor.build({
                    "bInterfaceNumber" : 0,
                })
            )

            # Class-specific audio control interface descriptor
            interface = uac2.ClassSpecificAudioControlInterfaceDescriptorEmitter()

            # 1: CS clock source
            interface.add_subordinate_descriptor(uac2.ClockSourceDescriptor.build({
                "bClockID"     : 1,
                "bmAttributes" : uac2.ClockAttributes.INTERNAL_FIXED_CLOCK,
                "bmControls"   : uac2.ClockFrequencyControl.HOST_READ_ONLY,
            }))

            # 2: IT streaming input terminal from the host to the USB device
            interface.add_subordinate_descriptor(uac2.InputTerminalDescriptor.build({
                "bTerminalID"   : 2,
                "wTerminalType" : uac2.USBTerminalTypes.USB_STREAMING,
                "bNrChannels"   : self.channels,
                "bCSourceID"    : 1,
            }))

            # 3: OT audio output terminal to the USB device's speaker output
            interface.add_subordinate_descriptor(uac2.OutputTerminalDescriptor.build({
                "bTerminalID"   : 3,
                "wTerminalType" : uac2.OutputTerminalTypes.SPEAKER,
                "bSourceID"     : 2,
                "bCSourceID"    : 1,
            }))

            # 4: IT audio input terminal from the USB device's microphone input
            interface.add_subordinate_descriptor(uac2.InputTerminalDescriptor.build({
                "bTerminalID"   : 4,
                "wTerminalType" : uac2.InputTerminalTypes.MICROPHONE,
                "bNrChannels"   : self.channels,
                "bCSourceID"    : 1,
            }))

            # 5: OT streaming output terminal to the host from the USB device
            interface.add_subordinate_descriptor(uac2.OutputTerminalDescriptor.build({
                "bTerminalID"   : 5,
                "wTerminalType" : uac2.USBTerminalTypes.USB_STREAMING,
                "bSourceID"     : 4,
                "bCSourceID"    : 1,
            }))
            configuration.add_subordinate_descriptor(interface)


            # - Interface #1: Audio output from the host to the USB device --

            # Audio Streaming Interface Descriptor (Audio Streaming OUT, alt 0 - quiet setting)
            configuration.add_subordinate_descriptor(
                uac2.AudioStreamingInterfaceDescriptor.build({
                    "bInterfaceNumber" : 1,
                    "bAlternateSetting" : 0,
                })
            )

            # Audio Streaming Interface Descriptor (Audio Streaming OUT, alt 1 - active setting)
            configuration.add_subordinate_descriptor(
                uac2.AudioStreamingInterfaceDescriptor.build({
                    "bInterfaceNumber"  : 1,
                    "bAlternateSetting" : 1,
                    "bNumEndpoints"     : 2,
                })
            )

            # Class Specific Audio Streaming Interface Descriptor
            configuration.add_subordinate_descriptor(
                uac2.ClassSpecificAudioStreamingInterfaceDescriptor.build({
                    "bTerminalLink" : 2,
                    "bFormatType"   : uac2.FormatTypes.FORMAT_TYPE_I,
                    "bmFormats"     : uac2.TypeIFormats.PCM,
                    "bNrChannels"   : self.channels,
                })
            )

            # Type I Format Type Descriptor
            configuration.add_subordinate_descriptor(uac2.TypeIFormatTypeDescriptor.build({
                "bSubslotSize"   : self.subslot_size,
                "bBitResolution" : self.bit_depth,
            }))

            # Endpoint Descriptor (Audio OUT from the host)
            configuration.add_subordinate_descriptor(standard.EndpointDescriptor.build({
                "bEndpointAddress" : USBDirection.OUT.to_endpoint_address(1), # EP 0x01 OUT
                "bmAttributes"     : USBTransferType.ISOCHRONOUS \
                                   | (USBSynchronizationType.ASYNC << 2) \
                                   | (USBUsageType.DATA << 4),
                "wMaxPacketSize"   : self.bytes_per_microframe + 1,
                "bInterval"        : 1,
            }))

            # Isochronous Audio Data Endpoint Descriptor
            configuration.add_subordinate_descriptor(
                uac2.ClassSpecificAudioStreamingIsochronousAudioDataEndpointDescriptor.build({})
            )

            # Endpoint Descriptor (Feedback IN to the host)
            configuration.add_subordinate_descriptor(standard.EndpointDescriptor.build({
                "bEndpointAddress" : USBDirection.IN.to_endpoint_address(2),  # EP 0x82 IN
                "bmAttributes"     : USBTransferType.ISOCHRONOUS \
                                   | (USBSynchronizationType.NONE << 2)  \
                                   | (USBUsageType.FEEDBACK << 4),
                "wMaxPacketSize"   : 4,
                "bInterval"        : 4,
            }))


            # - Interface #2: Audio input to the host from the USB device --

            # Audio Streaming Interface Descriptor (Audio Streaming IN, alt 0 - quiet setting)
            configuration.add_subordinate_descriptor(
                uac2.AudioStreamingInterfaceDescriptor.build({
                    "bInterfaceNumber" : 2,
                    "bAlternateSetting" : 0,
                })
            )

            # Audio Streaming Interface Descriptor (Audio Streaming IN, alt 1 - active setting)
            configuration.add_subordinate_descriptor(
                uac2.AudioStreamingInterfaceDescriptor.build({
                    "bInterfaceNumber"  : 2,
                    "bAlternateSetting" : 1,
                    "bNumEndpoints"     : 1,
                })
            )

            # Class Specific Audio Streaming Interface Descriptor
            configuration.add_subordinate_descriptor(
                uac2.ClassSpecificAudioStreamingInterfaceDescriptor.build({
                    "bTerminalLink" : 5,
                    "bFormatType"   : uac2.FormatTypes.FORMAT_TYPE_I,
                    "bmFormats"     : uac2.TypeIFormats.PCM,
                    "bNrChannels"   : self.channels,
                })
            )

            # Type I Format Type Descriptor
            configuration.add_subordinate_descriptor(uac2.TypeIFormatTypeDescriptor.build({
                "bSubslotSize"   : self.subslot_size,
                "bBitResolution" : self.bit_depth,
            }))

            # Endpoint Descriptor (Audio IN to the host)
            configuration.add_subordinate_descriptor(standard.EndpointDescriptor.build({
                "bEndpointAddress" : USBDirection.IN.to_endpoint_address(3), # EP 0x83 IN
                "bmAttributes"     : USBTransferType.ISOCHRONOUS  \
                                   | (USBSynchronizationType.ASYNC << 2) \
                                   | (USBUsageType.DATA << 4),
                "wMaxPacketSize"   : self.bytes_per_microframe + 1,
                "bInterval"        : 1,
            }))

            # Isochronous Audio Data Endpoint Descriptor
            configuration.add_subordinate_descriptor(
                uac2.ClassSpecificAudioStreamingIsochronousAudioDataEndpointDescriptor.build({})
            )

        return descriptors
