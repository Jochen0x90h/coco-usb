#pragma once

#include <coco/enum.hpp>
#include <cstdint>

#ifdef _MSC_VER
#define PACK(Declaration) __pragma(pack(push, 1)) Declaration __pragma(pack(pop))
#else
#define PACK(Declaration) Declaration __attribute__((__packed__))
#endif


namespace coco {

/**
 * Helper enums and structs for USB
 */
namespace usb {

/**
 * Descriptor type
 */
enum class DescriptorType : uint8_t {
    DEVICE = 1,
    CONFIGURATION = 2,
    STRING = 3,
    INTERFACE = 4,
    ENDPOINT = 5,

    // interface association descriptor type,
    // see https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
    INTERFACE_ASSOCIATION = 11,

    // CDC functional descriptor type "Interface"
    CDC_INTERFACE = 0x24,

    // CDC functional descriptor type "Endpoint"
    CDC_ENDPOINT = 0x25,
};

/**
 * Endpoint type
 */
enum class EndpointType : uint8_t {
    CONTROL = 0,
    ISOCHRONOUS = 1,
    BULK = 2,
    INTERRUPT = 3
};

/**
 * Endpoint transfer direction
 */
enum Direction : uint8_t {
    OUT = 0, // to device
    IN = 0x80 // to host
};

/**
 * Control request type
 */
enum class RequestType : uint8_t {
    // request type
    STANDARD = 0x0 << 5,
    CLASS = 0x1 << 5,
    VENDOR = 0x2 << 5,
    TYPE_MASK = 0x3 << 5,

    // request recipient
    DEVICE = 0x00,
    INTERFACE = 0x01,
    ENDPOINT = 0x02,
    OTHER = 0x03,
    RECIPIENT_MASK = 0x1f,

    // request direction
    OUT = 0, // to device
    IN = 0x80, // to host
    DIRECTION_MASK = 0x80,

    // combinations
    STANDARD_DEVICE_OUT = STANDARD | DEVICE | OUT,
    STANDARD_DEVICE_IN = STANDARD | DEVICE | IN,
    STANDARD_INTERFACE_OUT = STANDARD | INTERFACE | OUT,
    STANDARD_INTERFACE_IN = STANDARD | INTERFACE | IN,
    STANDARD_ENDPOINT_OUT = STANDARD | ENDPOINT | OUT,
    STANDARD_ENDPOINT_IN = STANDARD | ENDPOINT | IN,

    CLASS_DEVICE_OUT = CLASS | DEVICE | OUT,
    CLASS_DEVICE_IN = CLASS | DEVICE | IN,
    CLASS_INTERFACE_OUT = CLASS | INTERFACE | OUT,
    CLASS_INTERFACE_IN = CLASS | INTERFACE | IN,

    VENDOR_DEVICE_OUT = VENDOR | DEVICE | OUT,
    VENDOR_DEVICE_IN = VENDOR | DEVICE | IN,
    VENDOR_INTERFACE_OUT = VENDOR | INTERFACE | OUT,
    VENDOR_INTERFACE_IN = VENDOR | INTERFACE | IN,
    VENDOR_ENDPOINT_OUT = VENDOR | ENDPOINT | OUT,
    VENDOR_ENDPOINT_IN = VENDOR | ENDPOINT | IN
};
COCO_ENUM(RequestType)

/**
 * Control request.
 * This is a namespace so that it can be extended in the application code
 */
namespace Request {
    constexpr uint8_t GET_STATUS = 0x00; // device, interface, endpoint
    constexpr uint8_t CLEAR_FEATURE = 0x01; // device, interface, endpoint
    constexpr uint8_t SET_FEATURE = 0x03; // device, interface, endpoint
    constexpr uint8_t SET_ADDRESS = 0x05; // device
    constexpr uint8_t GET_DESCRIPTOR = 0x06; // device
    constexpr uint8_t SET_DESCRIPTOR = 0x07; // device
    constexpr uint8_t GET_CONFIGURATION = 0x08; // device
    constexpr uint8_t SET_CONFIGURATION = 0x09; // device
    constexpr uint8_t GET_INTERFACE = 0x0a; // interface
    constexpr uint8_t SET_INTERFACE = 0x11; // interface
    constexpr uint8_t SYNCH_FRAME = 0x12; // endpoint
}

/**
 * USB device class
 * see https://developerhelp.microchip.com/xwiki/bin/view/applications/usb/how-it-works/device-classes/
 */
enum class DeviceClass : uint8_t {
    // Use class code info from Interface Descriptors
    NONE = 0x00,

    // Communications Device Class (CDC)
    CDC = 0x02,

    // Hub
    HUB = 0x09,

    // Billboard Device Class
    BILLBOARD = 0x11,

    // Diagnostic Device
    DIAGNOSTIC = 0xDC,

    // Miscellaneous
    MISCELLANEOUS = 0xEF,

    // Vendor Specific
    VENDOR = 0xff,
};

/**
 * USB device subclass.
 * This is a namespace so that it can be extended in the application code
 */
namespace DeviceSubClass {
    // CDC Direct Line Control Model (DLCM)
    constexpr uint8_t CDC_DLCM = 0x01;

    // CDC Abstract Control Model (ACM)
    constexpr uint8_t CDC_ACM = 0x02;

    // CDC Telephone Control Model (TCM)
    constexpr uint8_t CDC_TCM = 0x03;

    // Miscellaneous Common
    constexpr uint8_t MISCELLANEOUS_COMMON = 0x02;
};

/**
 * USB device Protocol.
 * This is a namespace so that it can be extended in the application code
 */
namespace DeviceProtocol {
    // device protocol "Interface association descriptor"
    // see https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
    constexpr uint8_t INTERFACE_ASSOCIATION_DESCRIPTOR = 0x02;
};

/**
 * USB interface class
 * see https://developerhelp.microchip.com/xwiki/bin/view/applications/usb/how-it-works/device-classes/
 */
enum class InterfaceClass : uint8_t {
    // Audio
    AUDIO = 0x01,

    // Communications Device Class (CDC) Control
    CDC_CONTROL = 0x02,

    // Human Interface Device (HID)
    HID = 0x03,

    // Physical
    PHYSICAL = 0x05,

    // Image
    IMAGE = 0x06,

    // Printer
    PRINTER = 0x07,

    // Mass Storage (MSD)
    MASS_STORAGE = 0x08,

    // Communications Device Class (CDC) Data
    CDC_DATA = 0x0A,

    // Smart Card
    SMART_CARD = 0x0B,

    // Content Security
    CONTENT_SECURITY = 0x0D,

    // Video
    VIDEO = 0x0E,

    // Personal Healthcare
    PERSONAL_HEALTHCARE = 0x0F,

    // Audio/Video Devices
    AUDIO_VIDEO = 0x10,

    // Diagnostic Device
    DIAGNOSTIC = 0xDC,

    // Miscellaneous
    MISCELLANEOUS = 0xEF,

    // Application Specific
    APPLICATION = 0xFE,

    // Vendor Specific
    VENDOR = 0xFF,
};

/**
 * USB interface subclass.
 * This is a namespace so that it can be extended in the application code
 */
namespace InterfaceSubClass {
    // CDC Direct Line Control Model (DLCM)
    constexpr uint8_t CDC_DLCM = 0x01;

    // CDC Abstract Control Model (ACM)
    constexpr uint8_t CDC_ACM = 0x02;

    // CDC Telephone Control Model (TCM)
    constexpr uint8_t CDC_TCM = 0x03;
};

/**
 * USB interface Protocol.
 * This is a namespace so that it can be extended in the application code
 */
namespace InterfaceProtocol {
    // CDC interface protocol "AT"
    constexpr uint8_t AT = 0x01;
};


/**
 * Setup packet of control request
 */
PACK(struct Setup {
    usb::RequestType bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
});

/**
 * Device descriptor
 */
PACK(struct DeviceDescriptor {
    uint8_t bLength = sizeof(DeviceDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = usb::DescriptorType::DEVICE;

    /// USB version as BCD number
    uint16_t bcdUSB = 0x0200; // USB 2.0

    DeviceClass  bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;

    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
});

/**
 * Configuration descriptor
 */
PACK(struct ConfigurationDescriptor {
    uint8_t bLength = sizeof(ConfigurationDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::CONFIGURATION;

    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue; // value to use as an argument to select this configuration
    uint8_t iConfiguration; // index of String Descriptor describing this configuration
    uint8_t bmAttributes;
    uint8_t bMaxPower;
});

/**
 * Interface descriptor
 */
PACK(struct InterfaceDescriptor {
    uint8_t bLength = sizeof(InterfaceDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::INTERFACE;

    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    InterfaceClass bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
});

/**
 * Endpoint descriptor
 */
PACK(struct EndpointDescriptor {
    uint8_t bLength = sizeof(EndpointDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::ENDPOINT;

    uint8_t bEndpointAddress;
    EndpointType bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
});

/**
 * Interface association descriptor
 * see https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
 */
PACK(struct InterfaceAssiciationDescriptor {
    /// Size of this descriptor in bytes
    uint8_t bLength = sizeof(InterfaceAssiciationDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::INTERFACE_ASSOCIATION;

    /// Interface number of the first interface that is associated with this function
    uint8_t bFirstInterface;

    /// Number of contiguous interfaces that are associated with this function
    uint8_t bInterfaceCount;

    /// Class code (assigned by USB-IF)
    InterfaceClass bFunctionClass;

    /// Subclass code (assigned by USB-IF)
    uint8_t bFunctionSubClass;

    /// Subclass code (assigned by USB-IF)
    uint8_t bFunctionProtocol;

    /// Index of string descriptor describing this function
    uint8_t iFunction;
});


// CDC (communications device class)
// =================================

/**
 * Descriptor subtype
 */
enum class CdcDescriptorSubtype : uint8_t {
    CDC_HEADER = 0x00,
    CDC_PSTN_CALL_MANAGEMENT = 0x01,
    CDC_ACM = 0x02,
    CDC_UNION = 0x06,
};

namespace CdcAcmCapability {
    constexpr uint8_t LINE_CODING = 2;
}


/// USB CDC header functional descriptor
PACK(struct CdcHeaderDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(CdcHeaderDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    CdcDescriptorSubtype bDescriptorSubtype = CdcDescriptorSubtype::CDC_HEADER;

    /// USB CDC specification release number in binary-coded decimal (0x0110)
    uint16_t bcdCDC = 0x0110;
});

/// USB PSTN call management functional descriptor
PACK(struct CdcPstnCallManagementDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(CdcPstnCallManagementDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    CdcDescriptorSubtype bDescriptorSubtype = CdcDescriptorSubtype::CDC_PSTN_CALL_MANAGEMENT;

    /// The capabilities that this configuration supports
    uint8_t bmCapabilities;

    /// Interface number of the Data Class interface optionally used for call management
    uint8_t bDataInterface;
});

/// USB PSTN Abstract control management functional descriptor
PACK(struct CdcAcmDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(CdcAcmDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    CdcDescriptorSubtype bDescriptorSubtype = CdcDescriptorSubtype::CDC_ACM;

    /// The capabilities that this configuration supports (QSB_ACM_CAP_xxx)
    uint8_t bmCapabilities;
});

/// USB CDC union functional descriptor
PACK(struct CdcUnionDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(CdcUnionDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    CdcDescriptorSubtype bDescriptorSubtype = CdcDescriptorSubtype::CDC_UNION;

    /// The interface number of the communications or data class interface
    uint8_t bControlInterface;

    /// The interface number of the first subordinate interface in the union
    uint8_t bSubordinateInterface0;
});

/**
 * Functional descriptors for USB CDC control interface
 */
PACK(struct CdcFunctionalDescriptors {
    CdcHeaderDescriptor header;
    CdcPstnCallManagementDescriptor callManagement;
    CdcAcmDescriptor acm;
    CdcUnionDescriptor cdcUnion;
});


// CDC PSTN subclass (https://www.usb.org/document-library/class-definitions-communication-devices-12)
// ---------------------------------------------------------------------------------------------------

/**
 * PSTN specific request codes (6.3, Table 13)
 */
namespace PstnRequest {
    /// Set line coding
    constexpr uint8_t SET_LINE_CODING = 0x20;

    /// Get line coding
    constexpr uint8_t GET_LINE_CODING = 0x21;

    /// Set the control line state (D0: DTR, D1: RTS)
    constexpr uint8_t SET_CONTROL_LINE_STATE = 0x22;
}

/**
 * Control Line State (6.3.12, Table 18)
 */
enum class PstnControlLineState : uint16_t {
    NONE = 0,

    // Data Terminal Ready
    DTR = 1,

    // Request To Send
    RTS = 1 << 1
};
COCO_ENUM(PstnControlLineState)

/**
 * PSTN line coding structure (6.3.11, Table 17)
 */
PACK(struct PstnLineCoding {
    /// Data terminal rate, in bits per second
    uint32_t dwDTERate;

    /// Stop bits (0: 1 stop bit; 1: 1.5 stop bits; 2: 2 stop bits)
    uint8_t bCharFormat;

    /// Parity (0: none; 1: odd; 2: even; 3: mark; 4: space)
    uint8_t bParityType;

    /// Data bits (5, 6, 7, 8 or 16)
    uint8_t bDataBits;
});

/**
 * PSTN specific notifications (6.5, Table 30)
 */
namespace PstnNotification {
    /// Notify serial state (D0: TxCarrier/DCD, D1: RxCarrier/DSR, D2: Break, D3: RingSignal, D4: Framing, D5: Parity, D6: OverRun)
    constexpr uint8_t SERIAL_STATE = 0x20;
};

/**
 * Serial State (6.5.4, Table 31)
 */
enum class PstnSerialState : uint16_t {
    NONE = 0,

    // Data Carrier Detect
    DCD = 1,

    // Data Set Ready
    DSR = 1 << 1,

    // Ring Indicator
    RI = 1 << 3,

    BREAK = 1 << 2,
    FRAMING_ERROR = 1 << 4,
    PARITY_ERROR = 1 << 5,
    OVERRUN_ERROR = 1 << 6
};
COCO_ENUM(PstnSerialState)

} // namespace usb
} // namespace coco
