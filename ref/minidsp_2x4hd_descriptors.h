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
 * wTotalLength=373 (0x0175), so only the first 257 bytes were captured. The last
 * descriptor that begins within the buffer is the AS Isochronous EP descriptor for
 * Interface 1 Alt 2, which starts at byte 253. Only its first 4 bytes (08 25 01 00)
 * are within the buffer; the remaining 4 bytes are truncated.
 *
 * The remaining 116 bytes (offsets 257-372) have been RECONSTRUCTED from the XMOS
 * reference firmware, ALSA driver output, and the minidsp-rs control protocol.
 * See "Sources for reconstruction" at bottom of file for details.
 *
 * What's captured (bytes 0-256 of config 1, 257 bytes):
 *   - Configuration descriptor (9 bytes)
 *   - IAD for Audio function (8 bytes)
 *   - Interface 0 Alt 0: AudioControl (9 bytes)
 *   - Full AC entity descriptors: Header, Clock Source, Clock Selector,
 *     2x Input Terminal, 2x Feature Unit, 2x Output Terminal (127 bytes total)
 *   - Interface 1 Alt 0: AudioStreaming zero-bandwidth (9 bytes)
 *   - Interface 1 Alt 1: AudioStreaming 24-bit/2ch + endpoints (53 bytes)
 *   - Interface 1 Alt 2: AudioStreaming 16-bit/2ch (partial - 42 of 53 bytes)
 *
 * What's reconstructed (bytes 257-372, 116 bytes):
 *   - 4 bytes: Remainder of AS EP descriptor for Alt 2 (by analogy with Alt 1)
 *   - 7 bytes: Feedback endpoint 0x81 for Alt 2 (identical to Alt 1)
 *   - 55 bytes: Interface 2 AudioStreaming capture (Alt 0 + Alt 1, 24-bit/2ch)
 *   - 32 bytes: Interface 3 HID (miniDSP control, bidirectional 64-byte)
 *   - 18 bytes: Interface 4 DFU (XMOS standard firmware update)
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
/*  Configuration 1 Descriptor (373 bytes total)                              */
/*                                                                            */
/*  Bytes 0-256 (257 bytes): Captured from Arduino USB_desc.ino output.       */
/*  Bytes 257-372 (116 bytes): RECONSTRUCTED from XMOS reference firmware,    */
/*  Linux ALSA dumps, and minidsp-rs protocol analysis. See comments inline.  */
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
    /*  AS Isochronous EP Descriptor - Alt 2 (8 bytes) [offset 253]           */
    /*  First 4 bytes captured. Remaining 4 reconstructed by analogy w/ Alt 1 */
    /* ---------------------------------------------------------------------- */
    0x08,                   // bLength = 8
    0x25,                   // bDescriptorType = CS_ENDPOINT
    0x01,                   // bDescriptorSubtype = EP_GENERAL
    0x00,                   // bmAttributes = 0

    /* ====================================================================== */
    /*  RECONSTRUCTION: bytes 257-372 (116 bytes)                             */
    /*                                                                        */
    /*  Not captured by original Arduino dump (256-byte buffer limit).         */
    /*  Reconstructed from:                                                   */
    /*    - XMOS XU216 reference firmware (sw_usb_audio descriptors_2.h)      */
    /*    - Linux ALSA stream0 dump of DDRC-24 (same XMOS hardware as 2x4HD) */
    /*      posted by mdsimon2 on AudioScienceReview, July 2021               */
    /*    - XMOS lib_xua defaults for Full Speed channel counts               */
    /*    - miniDSP HID protocol (mrene/minidsp-rs, bidirectional 64-byte)    */
    /*    - Byte budget: 373 total - 257 captured = 116 missing, all accounted*/
    /*                                                                        */
    /*  Confidence levels:                                                    */
    /*    - Remaining AS EP + FB EP: HIGH (identical pattern to Alt 1)         */
    /*    - Interface 2 capture structure: HIGH (XMOS FS defaults + ALSA data)*/
    /*    - Capture EP 0x82 address: CONFIRMED (from ALSA stream0 dump)       */
    /*    - Capture channels=2 at FS: HIGH (XMOS default NUM_USB_CHAN_IN_FS)  */
    /*    - HID bidirectional: HIGH (minidsp-rs uses IN+OUT HID endpoints)    */
    /*    - HID EP addresses (0x83/0x03): ESTIMATED (XMOS convention)         */
    /*    - HID report descriptor length: ESTIMATED (34 bytes placeholder)    */
    /*    - DFU interface: HIGH (XMOS default, standard structure)             */
    /*    - DFU wTransferSize/bcdVersion: ESTIMATED (common XMOS values)      */
    /* ====================================================================== */

    // --- Reconstructed from here (offset 257) ---

    0x00,                   // bmControls = 0
    0x02,                   // bLockDelayUnits = 2 (milliseconds)
    0x08, 0x00,             // wLockDelay = 8 ms

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x81 IN - Isochronous Feedback (7 bytes) [offset 261]        */
    /*  Identical to Alt 1 feedback endpoint                                  */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x81,                   // bEndpointAddress = 0x81 (IN)
    0x11,                   // bmAttributes = 0x11 (Isochronous, Feedback)
    0x04, 0x00,             // wMaxPacketSize = 4
    0x04,                   // bInterval = 4 (every 8 frames at FS = 8ms)

    /* ====================================================================== */
    /*  Interface 2, Alt 0 - AudioStreaming capture (zero-bw) [offset 268]    */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x02,                   // bInterfaceNumber = 2
    0x00,                   // bAlternateSetting = 0
    0x00,                   // bNumEndpoints = 0
    0x01,                   // bInterfaceClass = Audio
    0x02,                   // bInterfaceSubClass = AudioStreaming
    0x20,                   // bInterfaceProtocol = 0x20 (UAC2)
    0x05,                   // iInterface = 5

    /* ====================================================================== */
    /*  Interface 2, Alt 1 - AudioStreaming capture 24-bit [offset 277]       */
    /*  Capture: 2ch, 24-bit, async isochronous IN EP 0x82                   */
    /*  At HS this is 4ch (confirmed by ALSA); at FS XMOS defaults to 2ch    */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x02,                   // bInterfaceNumber = 2
    0x01,                   // bAlternateSetting = 1
    0x01,                   // bNumEndpoints = 1 (data EP only, no feedback for IN)
    0x01,                   // bInterfaceClass = Audio
    0x02,                   // bInterfaceSubClass = AudioStreaming
    0x20,                   // bInterfaceProtocol = 0x20 (UAC2)
    0x05,                   // iInterface = 5

    /* ---------------------------------------------------------------------- */
    /*  AS Interface Descriptor - capture (16 bytes) [offset 286]             */
    /* ---------------------------------------------------------------------- */
    0x10,                   // bLength = 16
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x01,                   // bDescriptorSubtype = AS_GENERAL
    0x16,                   // bTerminalLink = 22 (OT22 = USB_STREAMING capture)
    0x00,                   // bmControls = 0
    0x01,                   // bFormatType = FORMAT_TYPE_I
    0x01, 0x00, 0x00, 0x00, // bmFormats = 0x00000001 (PCM)
    0x02,                   // bNrChannels = 2 (FS: 2ch; HS: 4ch)
    0x00, 0x00, 0x00, 0x00, // bmChannelConfig = 0 (unspecified)
    0x0D,                   // iChannelNames = 13

    /* ---------------------------------------------------------------------- */
    /*  AS Format Type I Descriptor - capture (6 bytes) [offset 302]          */
    /* ---------------------------------------------------------------------- */
    0x06,                   // bLength = 6
    0x24,                   // bDescriptorType = CS_INTERFACE
    0x02,                   // bDescriptorSubtype = FORMAT_TYPE
    0x01,                   // bFormatType = FORMAT_TYPE_I
    0x03,                   // bSubSlotSize = 3 (3 bytes = 24 bits)
    0x18,                   // bBitResolution = 24

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x82 IN - Isochronous Async (7 bytes) [offset 308]           */
    /*  Capture data endpoint. EP address 0x82 CONFIRMED by ALSA stream0.     */
    /*  wMaxPacketSize=294: 2ch * 3bytes * 49frames (48kHz + async headroom)  */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x82,                   // bEndpointAddress = 0x82 (IN) *** CONFIRMED ***
    0x05,                   // bmAttributes = 0x05 (Isochronous, Async)
    0x26, 0x01,             // wMaxPacketSize = 294 (0x0126)
    0x01,                   // bInterval = 1 (every frame = 1ms at FS)

    /* ---------------------------------------------------------------------- */
    /*  AS Isochronous EP Descriptor - capture (8 bytes) [offset 315]         */
    /* ---------------------------------------------------------------------- */
    0x08,                   // bLength = 8
    0x25,                   // bDescriptorType = CS_ENDPOINT
    0x01,                   // bDescriptorSubtype = EP_GENERAL
    0x00,                   // bmAttributes = 0
    0x00,                   // bmControls = 0
    0x02,                   // bLockDelayUnits = 2 (milliseconds)
    0x08, 0x00,             // wLockDelay = 8 ms

    /* ====================================================================== */
    /*  Interface 3, Alt 0 - HID (miniDSP control) [offset 323]              */
    /*  Used by minidsp-rs / miniDSP plugin for gain/mute/source/DSP control  */
    /*  Bidirectional: IN endpoint for status, OUT for commands               */
    /*  64-byte reports match minidsp-rs protocol (mrene/minidsp-rs)          */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x03,                   // bInterfaceNumber = 3
    0x00,                   // bAlternateSetting = 0
    0x02,                   // bNumEndpoints = 2 (IN + OUT)
    0x03,                   // bInterfaceClass = HID
    0x00,                   // bInterfaceSubClass = 0 (no boot)
    0x00,                   // bInterfaceProtocol = 0
    0x00,                   // iInterface = 0

    /* ---------------------------------------------------------------------- */
    /*  HID Descriptor (9 bytes) [offset 332]                                 */
    /* ---------------------------------------------------------------------- */
    0x09,                   // bLength = 9
    0x21,                   // bDescriptorType = HID
    0x10, 0x01,             // bcdHID = 1.10
    0x00,                   // bCountryCode = 0
    0x01,                   // bNumDescriptors = 1
    0x22,                   // bDescriptorType[0] = Report
    0x22, 0x00,             // wDescriptorLength[0] = 34 (ESTIMATED)

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x83 IN - Interrupt (HID status) (7 bytes) [offset 341]      */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x83,                   // bEndpointAddress = 0x83 (IN)
    0x03,                   // bmAttributes = 0x03 (Interrupt)
    0x40, 0x00,             // wMaxPacketSize = 64
    0x01,                   // bInterval = 1 (1ms)

    /* ---------------------------------------------------------------------- */
    /*  Endpoint 0x03 OUT - Interrupt (HID commands) (7 bytes) [offset 348]   */
    /* ---------------------------------------------------------------------- */
    0x07,                   // bLength = 7
    0x05,                   // bDescriptorType = ENDPOINT
    0x03,                   // bEndpointAddress = 0x03 (OUT)
    0x03,                   // bmAttributes = 0x03 (Interrupt)
    0x40, 0x00,             // wMaxPacketSize = 64
    0x01,                   // bInterval = 1 (1ms)

    /* ====================================================================== */
    /*  Interface 4, Alt 0 - DFU Runtime (9 bytes) [offset 355]               */
    /*  XMOS standard DFU interface for firmware updates                      */
    /* ====================================================================== */
    0x09,                   // bLength = 9
    0x04,                   // bDescriptorType = INTERFACE
    0x04,                   // bInterfaceNumber = 4
    0x00,                   // bAlternateSetting = 0
    0x00,                   // bNumEndpoints = 0
    0xFE,                   // bInterfaceClass = Application Specific (0xFE)
    0x01,                   // bInterfaceSubClass = DFU (0x01)
    0x01,                   // bInterfaceProtocol = Runtime (0x01)
    0x00,                   // iInterface = 0

    /* ---------------------------------------------------------------------- */
    /*  DFU Functional Descriptor (9 bytes) [offset 364]                      */
    /* ---------------------------------------------------------------------- */
    0x09,                   // bLength = 9
    0x21,                   // bDescriptorType = DFU_FUNCTIONAL
    0x07,                   // bmAttributes = 0x07 (download, upload, manifestation tolerant)
    0xFA, 0x00,             // wDetachTimeOut = 250 ms
    0x40, 0x00,             // wTransferSize = 64
    0x01, 0x10,             // bcdDFUVersion = 1.10 (ESTIMATED)
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
 *   Interface 1, Alt 1: 24-bit/2ch, EP 0x01 OUT async iso, MPS=294
 *   Interface 1, Alt 2: 16-bit/2ch, EP 0x01 OUT async iso, MPS=196
 *   Feedback: EP 0x81 IN, 4 bytes, interval=4 (8ms at FS)
 *
 * Capture path (device -> host):  [RECONSTRUCTED]
 *   IT ID=1 (MICROPHONE, 2ch) -> FU ID=11 -> OT ID=22 (USB_STREAMING)
 *   Interface 2, Alt 1: 24-bit/2ch, EP 0x82 IN async iso, MPS=294
 *   No feedback endpoint needed for IN direction
 *   At High Speed: 4 channels (confirmed by ALSA dump of DDRC-24)
 *   At Full Speed: 2 channels (XMOS default NUM_USB_CHAN_IN_FS)
 *
 * HID control interface (Interface 3):  [RECONSTRUCTED]
 *   Bidirectional: EP 0x83 IN + EP 0x03 OUT, interrupt, 64-byte MPS
 *   Used by miniDSP plugin / minidsp-rs for gain/mute/source/DSP control
 *
 * DFU interface (Interface 4):  [RECONSTRUCTED]
 *   Standard XMOS DFU runtime interface for firmware updates
 *
 * Endpoint summary:
 *   EP 0x01 OUT  - Isochronous Async  - Audio playback data
 *   EP 0x81 IN   - Isochronous Feedback - Playback rate feedback
 *   EP 0x82 IN   - Isochronous Async  - Audio capture data (CONFIRMED)
 *   EP 0x83 IN   - Interrupt           - HID status/responses (ESTIMATED)
 *   EP 0x03 OUT  - Interrupt           - HID commands (ESTIMATED)
 *
 * Audio formats at Full Speed:
 *   Playback Alt 1: PCM, 2ch, 24-bit, 3 bytes/subslot -> 294 bytes/frame max
 *          (48kHz * 2ch * 3B = 288B/frame nominal + 6B headroom for async)
 *   Playback Alt 2: PCM, 2ch, 16-bit, 2 bytes/subslot -> 196 bytes/frame max
 *          (48kHz * 2ch * 2B = 192B/frame nominal + 4B headroom for async)
 *   Capture Alt 1:  PCM, 2ch, 24-bit, 3 bytes/subslot -> 294 bytes/frame max
 *
 * Sources for reconstruction:
 *   - XMOS sw_usb_audio v6.1 descriptors_2.h (GitHub: itdaniher/USB-Audio-2.0-Software-v6.1)
 *   - XMOS lib_xua xua_conf_default.h (NUM_USB_CHAN_IN_FS defaults to min(N,2))
 *   - ALSA stream0 dump of DDRC-24 (same XMOS XU216 hardware as 2x4 HD)
 *     confirming EP 0x82 for capture, 4ch at HS, S32_LE/24-bit, async mode
 *     (mdsimon2 on AudioScienceReview, July 2021)
 *   - minidsp-rs (mrene/minidsp-rs) confirming bidirectional HID with 64-byte reports
 *   - linux-hardware.org usb:2752-0011 confirming Class 01-01-20 (Audio UAC2)
 */
