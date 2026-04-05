/**
 * @file minidsp_2x4hd_descriptors.h
 * @brief Raw USB descriptor bytes for the miniDSP 2x4 HD (VID 0x2752, PID 0x0011)
 *
 * Source: Arduino USB Host Shield 2.0 USB_desc.ino output
 * https://github.com/felis/USB_Host_Shield_2.0/issues/594
 * Posted by dennisfrett, 2021-01-24
 *
 * Reconstruction notes:
 * ---------------------
 * The Arduino sketch prints standard descriptors (device, config, interface, endpoint)
 * as named fields, and class-specific descriptors as "Unknown descriptor" with a
 * "Contents:" hex string. The printunkdescr() function has a bug: it prints bLength
 * bytes starting from offset 2 of the descriptor, which means it overruns by exactly
 * 2 bytes into the NEXT descriptor. Every Contents hex string therefore has 2 extra
 * garbage bytes at the end, which have been stripped during reconstruction.
 *
 * The sketch uses a 256-byte buffer (BUFSIZE=256). The config descriptor has
 * wTotalLength=373 (0x0175), so only the first 256 bytes were captured. The last
 * descriptor that begins within the buffer is the AS Isochronous EP descriptor for
 * Interface 1 Alt 2, which starts at byte 253. Only its first 4 bytes (08 25 01 00)
 * are within the buffer; the remaining 4 bytes are truncated.
 *
 * What's captured (bytes 0-255 of config 1):
 *   - Configuration descriptor (9 bytes)
 *   - IAD for Audio function (8 bytes)
 *   - Interface 0 Alt 0: AudioControl (9 bytes)
 *   - Full AC entity descriptors: Header, Clock Source, Clock Selector,
 *     2x Input Terminal, 2x Feature Unit, 2x Output Terminal (127 bytes total)
 *   - Interface 1 Alt 0: AudioStreaming zero-bandwidth (9 bytes)
 *   - Interface 1 Alt 1: AudioStreaming 24-bit/2ch + endpoints (53 bytes)
 *   - Interface 1 Alt 2: AudioStreaming 16-bit/2ch (partial - 38 of ~53 bytes)
 *
 * What's missing (bytes 256-372, ~117 bytes):
 *   - Remainder of Interface 1 Alt 2 endpoint descriptors
 *   - Interface 2: AudioStreaming (capture direction) with all alternates
 *   - Interface 3: HID (miniDSP control interface)
 *   - Interface 4: (likely DFU or vendor-specific)
 *
 * The device has bNumConfigurations=2. Both configurations appear to have the same
 * wTotalLength (373) and identical structure within the captured 256 bytes.
 *
 * Clock topology:
 *   Clock Source ID=41 (0x29) -> Clock Selector ID=40 (0x28)
 *   Playback: IT2 (USB_STREAMING) -> FU10 -> OT20 (SPEAKER)
 *   Capture:  IT1 (MICROPHONE)    -> FU11 -> OT22 (USB_STREAMING)
 *   Both paths use Clock Selector ID=40.
 *
 * XMOS XU216 processor. UAC2 at Full Speed (bInterfaceProtocol=0x20).
 */

#pragma once

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  Device Descriptor (18 bytes)                                              */
/* -------------------------------------------------------------------------- */

static const uint8_t minidsp_2x4hd_device_desc[] = {
    0x12,                   // bLength = 18
    0x01,                   // bDescriptorType = DEVICE
    0x00, 0x02,             // bcdUSB = 2.00
    0xEF,                   // bDeviceClass = Miscellaneous (0xEF)
    0x02,                   // bDeviceSubClass = Common Class (0x02)
    0x01,                   // bDeviceProtocol = IAD (0x01)
    0x40,                   // bMaxPacketSize0 = 64
    0x52, 0x27,             // idVendor = 0x2752 (miniDSP)
    0x11, 0x00,             // idProduct = 0x0011 (2x4 HD)
    0xF2, 0x06,             // bcdDevice = 6.F2 (firmware version)
    0x01,                   // iManufacturer = 1
    0x03,                   // iProduct = 3
    0x00,                   // iSerialNumber = 0 (none)
    0x02,                   // bNumConfigurations = 2
};

/* -------------------------------------------------------------------------- */
/*  Configuration 1 Descriptor (PARTIAL - 256 of 373 bytes captured)          */
/*                                                                            */
/*  Truncation occurs at byte 256. The last complete descriptor is the AS     */
/*  Format Type I for Alt 2 (ending at byte 246). The OUT endpoint at byte    */
/*  246 is complete (7 bytes, ending at byte 253). The AS EP descriptor       */
/*  starting at byte 253 is truncated after 4 bytes (only 08 25 01 00).      */
/* -------------------------------------------------------------------------- */

static const uint8_t minidsp_2x4hd_config1_desc[] = {

    /* ====================================================================== */
    /*  Configuration Descriptor (9 bytes) [offset 0]                         */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x02,                   // bDescriptorType = CONFIGURATION
    0x75, 0x01,             // wTotalLength = 373 (0x0175) -- full config is 373 bytes
    0x05,                   // bNumInterfaces = 5
    0x01,                   // bConfigurationValue = 1
    0x00,                   // iConfiguration = 0
    0xC0,                   // bmAttributes = Self-powered
    0x00,                   // bMaxPower = 0 (self-powered, no bus current)

    /* ====================================================================== */
    /*  IAD - Interface Association Descriptor (8 bytes) [offset 9]           */
    /* ====================================================================== */
    0x08,                   // bLength = 8
    0x0B,                   // bDescriptorType = INTERFACE_ASSOCIATION
    0x00,                   // bFirstInterface = 0
    0x03,                   // bInterfaceCount = 3  (AC + 2x AS)
    0x01,                   // bFunctionClass = Audio
    0x00,                   // bFunctionSubClass = 0
    0x20,                   // bFunctionProtocol = 0x20 (UAC2) *** KEY: proves UAC2 ***
    0x00,                   // iFunction = 0

    /* ====================================================================== */
    /*  Interface 0, Alt 0 - AudioControl (9 bytes) [offset 17]               */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x00,                   // bInterfaceNumber = 0
    0x00,                   // bAlternateSetting = 0
    0x00,                   // bNumEndpoints = 0
    0x01,                   // bInterfaceClass = Audio
    0x01,                   // bInterfaceSubClass = AudioControl
    0x20,                   // bInterfaceProtocol = 0x20 (UAC2)
    0x03,                   // iInterface = 3

    /* ---------------------------------------------------------------------- */
    /*  AC Interface Header Descriptor (9 bytes) [offset 26]                  */
    /*  UAC2: bcdADC=0x0200, wTotalLength covers all AC descriptors           */
    /* ---------------------------------------------------------------------- */
    0x09,                   // bLength = 9
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x01,                   // bDescriptorSubtype = HEADER
    0x00, 0x02,             // bcdADC = 2.00 (UAC2)
    0x08,                   // bCategory = IO_BOX (0x08)
    0x7F, 0x00,             // wTotalLength = 127 (0x007F) -- all AC descs
    0x00,                   // bmControls = 0

    /* ---------------------------------------------------------------------- */
    /*  Clock Source ID=41 (0x29) (8 bytes) [offset 35]                       */
    /*  Internal clock, frequency control r/w, validity read-only             */
    /* ---------------------------------------------------------------------- */
    0x08,                   // bLength = 8
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x0A,                   // bDescriptorSubtype = CLOCK_SOURCE
    0x29,                   // bClockID = 41
    0x03,                   // bmAttributes = 0x03 (internal, non-fixed freq)
    0x07,                   // bmControls = 0x07 (freq: r/w, validity: read)
    0x00,                   // bAssocTerminal = 0
    0x09,                   // iClockSource = 9

    /* ---------------------------------------------------------------------- */
    /*  Clock Selector ID=40 (0x28) (8 bytes) [offset 43]                     */
    /*  Single input: Clock Source ID=41                                      */
    /* ---------------------------------------------------------------------- */
    0x08,                   // bLength = 8
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x0B,                   // bDescriptorSubtype = CLOCK_SELECTOR
    0x28,                   // bClockID = 40
    0x01,                   // bNrInPins = 1
    0x29,                   // baCSourceID[0] = 41 (Clock Source)
    0x03,                   // bmControls = 0x03 (selector: r/w)
    0x08,                   // iClockSelector = 8

    /* ---------------------------------------------------------------------- */
    /*  Input Terminal ID=2 (17 bytes) [offset 51]                            */
    /*  USB Streaming (host playback -> device)                               */
    /* ---------------------------------------------------------------------- */
    0x11,                   // bLength = 17
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x02,                   // bDescriptorSubtype = INPUT_TERMINAL
    0x02,                   // bTerminalID = 2
    0x01, 0x01,             // wTerminalType = 0x0101 (USB_STREAMING)
    0x00,                   // bAssocTerminal = 0
    0x28,                   // bCSourceID = 40 (Clock Selector)
    0x02,                   // bNrChannels = 2
    0x00, 0x00, 0x00, 0x00, // bmChannelConfig = 0 (unspecified)
    0x0B,                   // iChannelNames = 11
    0x00, 0x00,             // bmControls = 0x0000
    0x06,                   // iTerminal = 6

    /* ---------------------------------------------------------------------- */
    /*  Feature Unit ID=10 (0x0A) (18 bytes) [offset 68]                      */
    /*  Playback path: source=IT2, master+2ch, vol/mute/bass/mid              */
    /* ---------------------------------------------------------------------- */
    0x12,                   // bLength = 18
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x06,                   // bDescriptorSubtype = FEATURE_UNIT
    0x0A,                   // bUnitID = 10
    0x02,                   // bSourceID = 2 (Input Terminal 2)
    0x0F, 0x00, 0x00, 0x00, // bmaControls(0) master = 0x0F (mute,vol,bass,mid r/w)
    0x0F, 0x00, 0x00, 0x00, // bmaControls(1) ch1
    0x0F, 0x00, 0x00, 0x00, // bmaControls(2) ch2
    0x00,                   // iFeature = 0

    /* ---------------------------------------------------------------------- */
    /*  Output Terminal ID=20 (0x14) (12 bytes) [offset 86]                   */
    /*  Speaker (device playback output)                                      */
    /* ---------------------------------------------------------------------- */
    0x0C,                   // bLength = 12
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x03,                   // bDescriptorSubtype = OUTPUT_TERMINAL
    0x14,                   // bTerminalID = 20
    0x01, 0x03,             // wTerminalType = 0x0301 (SPEAKER)
    0x00,                   // bAssocTerminal = 0
    0x0A,                   // bSourceID = 10 (Feature Unit 10)
    0x28,                   // bCSourceID = 40 (Clock Selector)
    0x00, 0x00,             // bmControls = 0x0000
    0x00,                   // iTerminal = 0

    /* ---------------------------------------------------------------------- */
    /*  Input Terminal ID=1 (17 bytes) [offset 98]                            */
    /*  Microphone (device capture input -> host)                             */
    /* ---------------------------------------------------------------------- */
    0x11,                   // bLength = 17
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x02,                   // bDescriptorSubtype = INPUT_TERMINAL
    0x01,                   // bTerminalID = 1
    0x01, 0x02,             // wTerminalType = 0x0201 (MICROPHONE)
    0x00,                   // bAssocTerminal = 0
    0x28,                   // bCSourceID = 40 (Clock Selector)
    0x02,                   // bNrChannels = 2
    0x00, 0x00, 0x00, 0x00, // bmChannelConfig = 0 (unspecified)
    0x0D,                   // iChannelNames = 13
    0x00, 0x00,             // bmControls = 0x0000
    0x00,                   // iTerminal = 0

    /* ---------------------------------------------------------------------- */
    /*  Feature Unit ID=11 (0x0B) (26 bytes) [offset 115]                     */
    /*  Capture path: source=IT1, master+4ch, vol/mute/bass/mid               */
    /*  Note: IT1 declares 2 channels but FU has 4ch controls (XMOS default)  */
    /* ---------------------------------------------------------------------- */
    0x1A,                   // bLength = 26
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x06,                   // bDescriptorSubtype = FEATURE_UNIT
    0x0B,                   // bUnitID = 11
    0x01,                   // bSourceID = 1 (Input Terminal 1)
    0x0F, 0x00, 0x00, 0x00, // bmaControls(0) master
    0x0F, 0x00, 0x00, 0x00, // bmaControls(1) ch1
    0x0F, 0x00, 0x00, 0x00, // bmaControls(2) ch2
    0x0F, 0x00, 0x00, 0x00, // bmaControls(3) ch3
    0x0F, 0x00, 0x00, 0x00, // bmaControls(4) ch4
    0x00,                   // iFeature = 0

    /* ---------------------------------------------------------------------- */
    /*  Output Terminal ID=22 (0x16) (12 bytes) [offset 141]                  */
    /*  USB Streaming (device capture -> host)                                */
    /* ---------------------------------------------------------------------- */
    0x0C,                   // bLength = 12
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x03,                   // bDescriptorSubtype = OUTPUT_TERMINAL
    0x16,                   // bTerminalID = 22
    0x01, 0x01,             // wTerminalType = 0x0101 (USB_STREAMING)
    0x00,                   // bAssocTerminal = 0
    0x0B,                   // bSourceID = 11 (Feature Unit 11)
    0x28,                   // bCSourceID = 40 (Clock Selector)
    0x00, 0x00,             // bmControls = 0x0000
    0x07,                   // iTerminal = 7

    /* ====================================================================== */
    /*  Interface 1, Alt 0 - AudioStreaming (zero-bandwidth) (9 bytes) [153]  */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x01,                   // bInterfaceNumber = 1
    0x00,                   // bAlternateSetting = 0
    0x00,                   // bNumEndpoints = 0
    0x01,                   // bInterfaceClass = Audio
    0x02,                   // bInterfaceSubClass = AudioStreaming
    0x20,                   // bInterfaceProtocol = 0x20 (UAC2)
    0x04,                   // iInterface = 4

    /* ====================================================================== */
    /*  Interface 1, Alt 1 - AudioStreaming 24-bit (9 bytes) [offset 162]     */
    /*  Playback: 2ch, 24-bit, 48kHz, async isochronous OUT EP 0x01          */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x01,                   // bInterfaceNumber = 1
    0x01,                   // bAlternateSetting = 1
    0x02,                   // bNumEndpoints = 2  (data + feedback)
    0x01,                   // bInterfaceClass = Audio
    0x02,                   // bInterfaceSubClass = AudioStreaming
    0x20,                   // bInterfaceProtocol = 0x20 (UAC2)
    0x04,                   // iInterface = 4

    /* ---------------------------------------------------------------------- */
    /*  AS Interface Descriptor - Alt 1 (16 bytes) [offset 171]               */
    /* ---------------------------------------------------------------------- */
    0x10,                   // bLength = 16
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x01,                   // bDescriptorSubtype = AS_GENERAL
    0x02,                   // bTerminalLink = 2 (Input Terminal 2 = playback)
    0x00,                   // bmControls = 0
    0x01,                   // bFormatType = FORMAT_TYPE_I
    0x01, 0x00, 0x00, 0x00, // bmFormats = 0x00000001 (PCM)
    0x02,                   // bNrChannels = 2
    0x00, 0x00, 0x00, 0x00, // bmChannelConfig = 0 (unspecified)
    0x0B,                   // iChannelNames = 11

    /* ---------------------------------------------------------------------- */
    /*  AS Format Type I Descriptor - Alt 1 (6 bytes) [offset 187]            */
    /* ---------------------------------------------------------------------- */
    0x06,                   // bLength = 6
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x02,                   // bDescriptorSubtype = FORMAT_TYPE
    0x01,                   // bFormatType = FORMAT_TYPE_I
    0x03,                   // bSubSlotSize = 3 (3 bytes = 24 bits)
    0x18,                   // bBitResolution = 24

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x01 OUT - Isochronous Async (7 bytes) [offset 193]          */
    /*  wMaxPacketSize=294: 2ch * 3bytes * 49frames = 294 (48kHz + 1 extra)   */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x01,                   // bEndpointAddress = 0x01 (OUT)
    0x05,                   // bmAttributes = 0x05 (Isochronous, Async)
    0x26, 0x01,             // wMaxPacketSize = 294 (0x0126)
    0x01,                   // bInterval = 1 (every frame = 1ms at Full Speed)

    /* ---------------------------------------------------------------------- */
    /*  AS Isochronous EP Descriptor - Alt 1 (8 bytes) [offset 200]           */
    /* ---------------------------------------------------------------------- */
    0x08,                   // bLength = 8
    0x25,                   // bDescriptorType = CS_ENDPOINT
    0x01,                   // bDescriptorSubtype = EP_GENERAL
    0x00,                   // bmAttributes = 0 (no pitch control, no overrun/underrun)
    0x00,                   // bmControls = 0
    0x02,                   // bLockDelayUnits = 2 (milliseconds)
    0x08, 0x00,             // wLockDelay = 8 ms

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x81 IN - Isochronous Feedback (7 bytes) [offset 208]        */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x81,                   // bEndpointAddress = 0x81 (IN)
    0x11,                   // bmAttributes = 0x11 (Isochronous, Feedback)
    0x04, 0x00,             // wMaxPacketSize = 4
    0x04,                   // bInterval = 4 (every 8 frames at FS = 8ms feedback rate)

    /* ====================================================================== */
    /*  Interface 1, Alt 2 - AudioStreaming 16-bit (9 bytes) [offset 215]     */
    /*  Playback: 2ch, 16-bit, 48kHz, async isochronous OUT EP 0x01          */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x01,                   // bInterfaceNumber = 1
    0x02,                   // bAlternateSetting = 2
    0x02,                   // bNumEndpoints = 2
    0x01,                   // bInterfaceClass = Audio
    0x02,                   // bInterfaceSubClass = AudioStreaming
    0x20,                   // bInterfaceProtocol = 0x20 (UAC2)
    0x04,                   // iInterface = 4

    /* ---------------------------------------------------------------------- */
    /*  AS Interface Descriptor - Alt 2 (16 bytes) [offset 224]               */
    /* ---------------------------------------------------------------------- */
    0x10,                   // bLength = 16
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x01,                   // bDescriptorSubtype = AS_GENERAL
    0x02,                   // bTerminalLink = 2 (Input Terminal 2 = playback)
    0x00,                   // bmControls = 0
    0x01,                   // bFormatType = FORMAT_TYPE_I
    0x01, 0x00, 0x00, 0x00, // bmFormats = 0x00000001 (PCM)
    0x02,                   // bNrChannels = 2
    0x00, 0x00, 0x00, 0x00, // bmChannelConfig = 0 (unspecified)
    0x0B,                   // iChannelNames = 11

    /* ---------------------------------------------------------------------- */
    /*  AS Format Type I Descriptor - Alt 2 (6 bytes) [offset 240]            */
    /* ---------------------------------------------------------------------- */
    0x06,                   // bLength = 6
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x02,                   // bDescriptorSubtype = FORMAT_TYPE
    0x01,                   // bFormatType = FORMAT_TYPE_I
    0x02,                   // bSubSlotSize = 2 (2 bytes = 16 bits)
    0x10,                   // bBitResolution = 16

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x01 OUT - Isochronous Async (7 bytes) [offset 246]          */
    /*  wMaxPacketSize=196: 2ch * 2bytes * 49frames = 196 (48kHz + 1 extra)   */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x01,                   // bEndpointAddress = 0x01 (OUT)
    0x05,                   // bmAttributes = 0x05 (Isochronous, Async)
    0xC4, 0x00,             // wMaxPacketSize = 196 (0x00C4)
    0x01,                   // bInterval = 1

    /* ---------------------------------------------------------------------- */
    /*  AS Isochronous EP Descriptor - Alt 2 (TRUNCATED) [offset 253]         */
    /*  Only first 4 bytes captured (256-byte buffer limit).                  */
    /*  Remaining 4 bytes (bmControls, bLockDelayUnits, wLockDelay) unknown.  */
    /*  By analogy with Alt 1, likely: 00 02 08 00                            */
    /* ---------------------------------------------------------------------- */
    0x08,                   // bLength = 8
    0x25,                   // bDescriptorType = CS_ENDPOINT
    0x01,                   // bDescriptorSubtype = EP_GENERAL
    0x00,                   // bmAttributes = 0

    /* ====================================================================== */
    /*  TRUNCATION: bytes 257-372 not captured (256-byte Arduino buffer)      */
    /*                                                                        */
    /*  Missing content (estimated ~117 bytes):                               */
    /*    - 4 bytes: remainder of AS EP descriptor for Alt 2                  */
    /*    - 7 bytes: Feedback endpoint 0x81 for Alt 2                         */
    /*    - ~53 bytes: Interface 2 (AudioStreaming capture) with alternates    */
    /*    - ~50+ bytes: Interfaces 3-4 (HID control, possibly DFU)           */
    /* ====================================================================== */
};

/*
 * Summary of key values:
 *
 * Device:
 *   VID = 0x2752 (miniDSP)
 *   PID = 0x0011 (2x4 HD)
 *   bcdUSB = 0x0200
 *   bDeviceClass = 0xEF (Miscellaneous, IAD)
 *   bMaxPacketSize0 = 64
 *   bNumConfigurations = 2
 *
 * AudioControl (Interface 0):
 *   UAC version = 2.00 (bcdADC=0x0200, bInterfaceProtocol=0x20)
 *   AC wTotalLength = 127 bytes (9 descriptors)
 *   Clock: Source ID=41 (internal, variable freq) -> Selector ID=40
 *
 * Playback path (host -> device):
 *   IT ID=2 (USB_STREAMING, 2ch) -> FU ID=10 -> OT ID=20 (SPEAKER)
 *   Interface 1, Alt 1: 24-bit/2ch, EP 0x01 OUT async iso, 294 bytes/frame
 *   Interface 1, Alt 2: 16-bit/2ch, EP 0x01 OUT async iso, 196 bytes/frame
 *   Feedback: EP 0x81 IN, 4 bytes, interval=4 (8ms at FS)
 *
 * Capture path (device -> host):
 *   IT ID=1 (MICROPHONE, 2ch) -> FU ID=11 -> OT ID=22 (USB_STREAMING)
 *   Interface 2: NOT CAPTURED (truncated)
 *
 * Audio formats at Full Speed:
 *   Alt 1: PCM, 2ch, 24-bit, 3 bytes/subslot -> 294 bytes/frame max
 *          (48kHz * 2ch * 3B = 288B/frame nominal + 6B headroom for async)
 *   Alt 2: PCM, 2ch, 16-bit, 2 bytes/subslot -> 196 bytes/frame max
 *          (48kHz * 2ch * 2B = 192B/frame nominal + 4B headroom for async)
 */
