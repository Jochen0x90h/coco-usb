#pragma once

#include <coco/enum.hpp>
#include <coco/platform/compiler.hpp>
#include <cstdint>


namespace coco {

/// @brief Helper enums and structs for USB
///
namespace usb {

/// @brief Descriptor type
///
enum class DescriptorType : uint8_t {
    DEVICE = 1,
    CONFIGURATION = 2,
    STRING = 3,
    INTERFACE = 4,
    ENDPOINT = 5,

    // interface association descriptor type
    // see https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
    INTERFACE_ASSOCIATION = 11,

    // binary object store descriptor
    // https://www.usb.org/bos-descriptor-types
    BOS = 0x0f,
    DEVICE_CAPABILITY = 0x10,

    // DFU
    DFU = 0x21,

    // CDC functional descriptor type "Interface"
    CDC_INTERFACE = 0x24,

    // CDC functional descriptor type "Endpoint"
    CDC_ENDPOINT = 0x25,
};

/// @brief Endpoint type
///
enum class EndpointType : uint8_t {
    CONTROL = 0,
    ISOCHRONOUS = 1,
    BULK = 2,
    INTERRUPT = 3
};

/// @brief Endpoint transfer direction
///
enum Direction : uint8_t {
    OUT = 0, // to device
    IN = 0x80 // to host
};

/// @brief Control request type
///
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

    // common combinations
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

/// @brief Control request.
/// This is a namespace so that it can be extended in the application code
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

/// @brief USB device class
/// see https://developerhelp.microchip.com/xwiki/bin/view/applications/usb/how-it-works/device-classes/
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

/// @brief USB device subclass.
/// This is a namespace so that it can be extended in the application code
namespace DeviceSubClass {
    // Use subclass code info from Interface Descriptors
    constexpr uint8_t NONE = 0x00;

    // CDC Direct Line Control Model (DLCM)
    constexpr uint8_t CDC_DLCM = 0x01;

    // CDC Abstract Control Model (ACM)
    constexpr uint8_t CDC_ACM = 0x02;

    // CDC Telephone Control Model (TCM)
    constexpr uint8_t CDC_TCM = 0x03;

    // Miscellaneous Common
    constexpr uint8_t MISCELLANEOUS_COMMON = 0x02;
};

/// @brief USB device Protocol.
/// This is a namespace so that it can be extended in the application code
namespace DeviceProtocol {
    // device protocol "Interface association descriptor"
    // see https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
    constexpr uint8_t INTERFACE_ASSOCIATION_DESCRIPTOR = 0x02;
};

/// @brief USB interface class
/// see https://developerhelp.microchip.com/xwiki/bin/view/applications/usb/how-it-works/device-classes/
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

    // Application Specific (DFU)
    APPLICATION = 0xFE,

    // Vendor Specific
    VENDOR = 0xFF,
};

/// @brief USB interface subclass.
/// This is a namespace so that it can be extended in the application code
namespace InterfaceSubClass {
    // CDC Direct Line Control Model (DLCM)
    constexpr uint8_t CDC_DLCM = 0x01;

    // CDC Abstract Control Model (ACM)
    constexpr uint8_t CDC_ACM = 0x02;

    // CDC Telephone Control Model (TCM)
    constexpr uint8_t CDC_TCM = 0x03;


    // DFU (Device Firmware Update)
    constexpr uint8_t DFU = 0x01;
};

/// @brief USB interface Protocol.
/// This is a namespace so that it can be extended in the application code
namespace InterfaceProtocol {
    // CDC interface protocol "AT"
    constexpr uint8_t AT = 0x01;

    // DFU runtime protocol (device in normal run-time operation)
    constexpr uint8_t DFU_RUNTIME = 0x01;

    // DFU protocol (device in DFU mode)
    constexpr uint8_t DFU_MODE = 0x02;
};

/// @brief Setup packet of control request
///
COCO_PACK_BEGIN struct Setup {
    usb::RequestType bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} COCO_PACK_END


/// @brief Device descriptor
///
COCO_PACK_BEGIN struct DeviceDescriptor {
    uint8_t bLength = sizeof(DeviceDescriptor);

    // Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::DEVICE;

    // USB version as BCD number
    // set to 0x0210 (USB 2.10) for support of BOS descriptor)
    uint16_t bcdUSB = 0x0200;

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
} COCO_PACK_END


/// @brief Attributes of ConfigurationDescriptor (one of SELF_POWERED or BUS_POWERED must be set)
enum class ConfigurationDescriptorAttriubtes : uint8_t {
    // Remote wakeup
    REMOTE_WAKEUP = 0x20,

    // Self-powered (note that D7 must always be 1 for historical reasons)
    SELF_POWERED = 0x40 | 0x80,

    // Bus-powered
    BUS_POWERED = 0x80,
};
COCO_ENUM(ConfigurationDescriptorAttriubtes)

/// @brief Configuration descriptor
///
COCO_PACK_BEGIN struct ConfigurationDescriptor {
    uint8_t bLength = sizeof(ConfigurationDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::CONFIGURATION;

    // Length of configuration descriptor (including all interface and endpoint descriptors)
    uint16_t wTotalLength;

    // Number of interface descriptors following
    uint8_t bNumInterfaces;

    // Value to use as an argument to select this configuration
    uint8_t bConfigurationValue;

    // Index of String Descriptor describing this configuration
    uint8_t iConfiguration;

    // Attributes
    ConfigurationDescriptorAttriubtes bmAttributes;

    // Maximum power in 2mA units
    uint8_t bMaxPower;
} COCO_PACK_END

/// @brief String descriptor
///
template <int N>
COCO_PACK_BEGIN struct StringDescriptor {
    uint8_t bLength = sizeof(StringDescriptor);
    DescriptorType bDescriptorType = DescriptorType::STRING;
    char16_t wString[N];
} COCO_PACK_END

template <int N>
constexpr StringDescriptor<N - 1> makeStringDescriptor(const char16_t (&str)[N]) {
    StringDescriptor<N - 1> desc;
    for (int i = 0; i < N - 1; ++i) {
        desc.wString[i] = str[i];
    }
    return desc;
}


/// @brief Interface descriptor
///
COCO_PACK_BEGIN struct InterfaceDescriptor {
    uint8_t bLength = sizeof(InterfaceDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::INTERFACE;

    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;

    // Number of endpoint descriptors following
    uint8_t bNumEndpoints;

    InterfaceClass bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} COCO_PACK_END

/// @brief Endpoint descriptor
///
COCO_PACK_BEGIN struct EndpointDescriptor {
    uint8_t bLength = sizeof(EndpointDescriptor);

    /// Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::ENDPOINT;

    uint8_t bEndpointAddress;
    EndpointType bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} COCO_PACK_END


/// @brief Interface association descriptor
/// see https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
COCO_PACK_BEGIN struct InterfaceAssiciationDescriptor {
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
} COCO_PACK_END


/// @brief Binary Object Store (BOS) descriptor
/// USB version must be 2.10 so that windows asks for the descriptor
COCO_PACK_BEGIN struct BosDescriptor {
    uint8_t bLength = sizeof(BosDescriptor);

    // Type of this descriptor
    DescriptorType bDescriptorType = DescriptorType::BOS;

    // Length of BOS descriptor (including all capability descriptors)
    uint16_t wTotalLength;

    // Number of capability descriptors following
    uint8_t bNumDeviceCaps;
} COCO_PACK_END


// =====================================================================================================================
// Microsoft OS 2.0 Descriptors Specification
// =====================================================================================================================

// https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/automatic-installation-of-winusb
// https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-os-2-0-descriptors-specification
// Example: https://github.com/pololu/libusbp/blob/master/test/firmware/wixel/main.c

namespace msos20 {

/// @brief Microsoft OS 2.0 platform capability descriptor header.
/// https://github.com/ataradov/free-dap/blob/master/platform/samd11/usb_descriptors.c
/// https://www.mikrocontroller.net/topic/544267
COCO_PACK_BEGIN struct PlatformCapabilityDescriptorHeader {
    // Length of header and following array of descriptor set information structures
    uint8_t bLength;

    // Type of this descriptor
    usb::DescriptorType bDescriptorType = usb::DescriptorType::DEVICE_CAPABILITY;

    // Type (5 = device capability specific to a particular platform/operating system)
    uint8_t bDevCapabilityType = 5;
    uint8_t bReserved = 0;

    // MS_OS_20_Platform_Capability_ID D8DD60DF-4589-4CC7-9CD2-659D9E648A9F
    uint32_t uuid1 = 0xD8DD60DF;
    uint16_t uuid2 = 0x4589;
    uint16_t uuid3 = 0x4CC7;
    uint8_t uuid4[8] = {0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F};
} COCO_PACK_END

COCO_PACK_BEGIN struct DescriptorSetInformation {
    uint32_t dwWindowsVersion = 0x06030000; // Windows 8.1

    // Length of the MS OS 2.0 descriptor set (field wTotalLength of struct MsOs20DescriptorSetHeader)
    uint16_t wMSOSDescriptorSetTotalLength;

    // Vendor request code to use to retrieve the MS OS 2.0 descriptor
    uint8_t bMS_VendorCode;

    uint8_t bAltEnumCode = 0;
} COCO_PACK_END

enum class DescriptorType : uint16_t {
    SET_HEADER_DESCRIPTOR = 0x00,
    SUBSET_HEADER_CONFIGURATION = 0x01,
    SUBSET_HEADER_FUNCTION = 0x02,
    FEATURE_COMPATBLE_ID = 0x03,
    FEATURE_REG_PROPERTY = 0x04,
    FEATURE_MIN_RESUME_TIME = 0x05,
    FEATURE_MODEL_ID = 0x06,
    FEATURE_CCGP_DEVICE = 0x07,
    FEATURE_VENDOR_REVISION = 0x08
};

// Table 10. Microsoft OS 2.0 descriptor set header
COCO_PACK_BEGIN struct DescriptorSetHeader {
    uint16_t wLength = sizeof(DescriptorSetHeader);

    // Type of this descriptor
    DescriptorType wDescriptorType = DescriptorType::SET_HEADER_DESCRIPTOR;

    uint32_t dwWindowsVersion = 0x06030000; // Windows 8.1

    // Length of header and following subsets
    uint16_t wTotalLength;
} COCO_PACK_END

// Table 11. Configuration subset header
COCO_PACK_BEGIN struct ConfigurationSubsetHeader {
    uint16_t wLength = sizeof(ConfigurationSubsetHeader);

    // Type of this descriptor
    DescriptorType wDescriptorType = DescriptorType::SUBSET_HEADER_CONFIGURATION;

    // The configuration value for the USB configuration to which this subset applies (is a zero based index)
    uint8_t bConfigurationValue;
    uint8_t bReserved = 0;

    // Length of header and following features
    uint16_t wTotalLength;
} COCO_PACK_END

// Table 12. Function subset header
COCO_PACK_BEGIN struct FunctionSubsetHeader {
    uint16_t wLength = sizeof(FunctionSubsetHeader);

    // Type of this descriptor
    DescriptorType wDescriptorType = DescriptorType::SUBSET_HEADER_FUNCTION;

    // The interface number for the first interface of the function to which this subset applies
    uint8_t bFirstInterface;
    uint8_t bReserved = 0;

    // Length of header and following features
    uint16_t wSubsetLength;
} COCO_PACK_END

// Table 13. Microsoft OS 2.0 compatible ID descriptor
COCO_PACK_BEGIN struct CompatibleIdDescriptor {
    uint16_t wLength = sizeof(CompatibleIdDescriptor);

    // Type of this descriptor
    DescriptorType wDescriptorType = DescriptorType::FEATURE_COMPATBLE_ID;

    char CompatibleID[8];

    char SubCompatibleID[8] = {};
} COCO_PACK_END

enum class PropertyDatatype : uint16_t {
    // Null terminated string (utf-16))
    SZ = 1,

    // Multiple null terminated strings (utf-16)
    MULTI_SZ = 7,

    // Null terminated string with env variable references (utf-16)
    EXPAND_SZ = 2,

    // Free form binary
    BINARY = 3,

    // 32-bit number
    DWORD_LITTLE_ENDIAN = 4,

    // 32-bit number
    DWORD_BIG_ENDIAN = 5,

    // Symbolic link (utf-16)
    LINK = 6,
};

// Table 14. Microsoft OS 2.0 registry property descriptor
template <int N, int D>
COCO_PACK_BEGIN struct RegistryPropertyDescriptor {
    uint16_t wLength = sizeof(RegistryPropertyDescriptor);

    // Type of this descriptor
    DescriptorType wDescriptorType = DescriptorType::FEATURE_REG_PROPERTY;

    PropertyDatatype wPropertyDataType;

    uint16_t wPropertyNameLength = N * 2;
    char16_t PropertyName[N];

    uint16_t wPropertyDataLength = D * 2;
    char16_t PropertyData[D];
} COCO_PACK_END

} // namespace msos20


// =====================================================================================================================
// CDC (Communications Device Class)
// =====================================================================================================================
namespace cdc {

/// @brief Descriptor subtype
///
enum class DescriptorSubtype : uint8_t {
    CDC_HEADER = 0x00,
    CDC_PSTN_CALL_MANAGEMENT = 0x01,
    CDC_ACM = 0x02,
    CDC_UNION = 0x06,
};

namespace AcmCapability {
    constexpr uint8_t LINE_CODING = 2;
}


/// @brief USB CDC header functional descriptor
///
COCO_PACK_BEGIN struct HeaderDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(HeaderDescriptor);

    /// Type of this descriptor
    usb::DescriptorType bDescriptorType = usb::DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::CDC_HEADER;

    /// USB CDC specification release number in binary-coded decimal (0x0110)
    uint16_t bcdCDC = 0x0110;
} COCO_PACK_END

/// @brief USB PSTN call management functional descriptor
///
COCO_PACK_BEGIN struct PstnCallManagementDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(PstnCallManagementDescriptor);

    /// Type of this descriptor
    usb::DescriptorType bDescriptorType = usb::DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::CDC_PSTN_CALL_MANAGEMENT;

    /// The capabilities that this configuration supports
    uint8_t bmCapabilities;

    /// Interface number of the Data Class interface optionally used for call management
    uint8_t bDataInterface;
} COCO_PACK_END

/// @brief USB PSTN Abstract control management functional descriptor
///
COCO_PACK_BEGIN struct AcmDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(AcmDescriptor);

    /// Type of this descriptor
    usb::DescriptorType bDescriptorType = usb::DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::CDC_ACM;

    /// The capabilities that this configuration supports (QSB_ACM_CAP_xxx)
    uint8_t bmCapabilities;
} COCO_PACK_END

/// @brief USB CDC union functional descriptor
///
COCO_PACK_BEGIN struct UnionDescriptor {
    /// Size of this descriptor, in bytes
    uint8_t bLength = sizeof(UnionDescriptor);

    /// Type of this descriptor
    usb::DescriptorType bDescriptorType = usb::DescriptorType::CDC_INTERFACE;

    /// Subtype of this descriptor
    DescriptorSubtype bDescriptorSubtype = DescriptorSubtype::CDC_UNION;

    /// The interface number of the communications or data class interface
    uint8_t bControlInterface;

    /// The interface number of the first subordinate interface in the union
    uint8_t bSubordinateInterface0;
} COCO_PACK_END

/// @brief Functional descriptors for USB CDC control interface
///
COCO_PACK_BEGIN struct FunctionalDescriptors {
    HeaderDescriptor header;
    PstnCallManagementDescriptor callManagement;
    AcmDescriptor acm;
    UnionDescriptor cdcUnion;
} COCO_PACK_END

// ---------------------------------------------------------------------------------------------------------------------
// CDC PSTN subclass
// https://www.usb.org/document-library/class-definitions-communication-devices-12
// ---------------------------------------------------------------------------------------------------------------------

/// @brief PSTN specific request codes (6.3, Table 13)
///
namespace PstnRequest {
    /// Set line coding
    constexpr uint8_t SET_LINE_CODING = 0x20;

    /// Get line coding
    constexpr uint8_t GET_LINE_CODING = 0x21;

    /// Set the control line state (D0: DTR, D1: RTS)
    constexpr uint8_t SET_CONTROL_LINE_STATE = 0x22;
}

/// @brief Control Line State (6.3.12, Table 18)
///
enum class PstnControlLineState : uint16_t {
    NONE = 0,

    // Data Terminal Ready
    DTR = 1,

    // Request To Send
    RTS = 1 << 1
};
COCO_ENUM(PstnControlLineState)

/// @brief PSTN line coding structure (6.3.11, Table 17)
///
COCO_PACK_BEGIN struct PstnLineCoding {
    /// Data terminal rate, in bits per second
    uint32_t dwDTERate;

    /// Stop bits (0: 1 stop bit; 1: 1.5 stop bits; 2: 2 stop bits)
    uint8_t bCharFormat;

    /// Parity (0: none; 1: odd; 2: even; 3: mark; 4: space)
    uint8_t bParityType;

    /// Data bits (5, 6, 7, 8 or 16)
    uint8_t bDataBits;
} COCO_PACK_END

/// @brief PSTN specific notifications (6.5, Table 30)
///
namespace PstnNotification {
    /// Notify serial state (D0: TxCarrier/DCD, D1: RxCarrier/DSR, D2: Break, D3: RingSignal, D4: Framing, D5: Parity, D6: OverRun)
    constexpr uint8_t SERIAL_STATE = 0x20;
};

/// @brief Serial State (6.5.4, Table 31)
///
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

} // namespace cdc


// =====================================================================================================================
// DFU (Device Firmware Upgrade)
// https://www.usb.org/sites/default/files/DFU_1.1.pdf
// =====================================================================================================================
namespace dfu {

/// @brief DFU specific request codes (3, Table 3.1)
///
namespace Request {
    /// Requests the device to leave DFU mode and enter the application
    constexpr uint8_t DFU_DETACH = 0;

    /// Download firmware from host to device flash memory
    constexpr uint8_t DFU_DNLOAD = 1;

    /// Upload firmware from device flash memory to host
    constexpr uint8_t DFU_UPLOAD = 2;

    /// Request status from the device
    constexpr uint8_t DFU_GETSTATUS = 3;

    /// Requests device to clear error status
    constexpr uint8_t DFU_CLRSTATUS = 4;

    /// Requests the device to send only the state it enters immediately after this request
    constexpr uint8_t DFU_GETSTATE = 5;

    /// Requests device to exit the current state/operation and enter idle state immediately
    constexpr uint8_t DFU_ABORT = 6;
}

/// @brief Attribute flags of DFU functional descriptor
///
enum class Attributes : uint8_t {
    // device generates detach-attach sequence after successful firmware update or DFU_DETACH request in APP_IDLE state
    WILL_DETACH = 1 << 3,
    MANIFESTATION_TOLERANT = 1 << 2,
    CAN_UPLOAD = 1 << 1,
    CAN_DNLOAD = 1 << 0,
};
COCO_ENUM(Attributes)

/// @brief DFU functional descriptor
///
COCO_PACK_BEGIN struct FunctionalDescriptor {
    uint8_t bLength = sizeof(FunctionalDescriptor);

    // Type of this descriptor
    usb::DescriptorType bDescriptorType = usb::DescriptorType::DFU;

    /// Attributes
    Attributes bmAttributes;

    uint16_t wDetachTimeout;

    uint16_t wTransferSize;

    uint16_t bcdDFUVersion = 0x101;
} COCO_PACK_END

enum class Status : uint8_t {
    OK = 0,

    ERR_ERASE = 0x04,

    ERR_VERIFY = 0x07,

    ERR_NOTDONE = 0x09,
};

enum class State : uint8_t {
    APP_IDLE = 0,

    APP_DETACH = 1,

    DFU_IDLE = 2,

    DFU_DNLOAD_SYNC = 3,

    DFU_DNBUSY = 4,

    DFU_DNLOAD_IDLE = 5,

    DFU_MANIFEST_SYNC = 6,

    DFU_MANIFEST = 7,

    DFU_MANIFEST_WAIT_RESET = 8,

    DFU_UPLOAD_IDLE = 9,

    DFU_ERROR = 10,
};

/// @brief DFU status report, reply to DFU_GETSTATUS request
///
COCO_PACK_BEGIN struct StatusReport {
    Status bStatus;

    uint8_t bwPollTimeout;
    uint16_t bwPollTimeout2;

    State bState;

    uint8_t iString;
} COCO_PACK_END

} // namespace dfu

} // namespace usb
} // namespace coco
