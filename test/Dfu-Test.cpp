#include <Dfu-Test.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/StreamOperators.hpp>


/*
    DFU (Device Firmware Upgrade) test for device in DFU mode (see UsbSerialTest for device in run-time mode)
    USB class specification: https://www.usb.org/sites/default/files/DFU_1.1.pdf)
    Windows: Uses BOS descriptor to use WinUSB driver (use Zadig to install WinUSB driver if necessary)

    List devices in DFU mode: $ dfu-util -l
    Download firmware into device: $ dfu-util -d 1209:0003 -D firmware.bin
*/

using namespace coco;
namespace msos20 = usb::msos20;
namespace dfu = usb::dfu;


constexpr int USB_VID = 0x1209; // https://pid.codes/1209/
constexpr int USB_PID = 0x0003; // test PID
constexpr int USB_DEVICE_VERSION = 0x0100; // version 1.00


// string id
enum class StringId : uint8_t {
    LANGUAGES = 0,
    MANUFACTURER = 1,
    PRODUCT = 2,
    SERIAL = 3,
    DFU_INTERFACE = 4,
};


// vendor specific control request
enum class VendorRequest : uint8_t {
    WINUSB = 6,
};


// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
    .bcdUSB = 0x0210, // USB version 2.10 to support BOS descriptor
    .bDeviceClass = usb::DeviceClass::NONE,
    .bDeviceSubClass = usb::DeviceSubClass::NONE,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64, // max packet size for endpoint 0
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_DEVICE_VERSION, // device version
    .iManufacturer = uint8_t(StringId::MANUFACTURER), // index into string table
    .iProduct = uint8_t(StringId::PRODUCT), // index into string table
    .iSerialNumber = uint8_t(StringId::SERIAL), // index into string table
    .bNumConfigurations = 1
};


// configuration descriptor
struct UsbConfiguration {
    usb::ConfigurationDescriptor config;
    usb::InterfaceDescriptor dfuInterface;
    dfu::FunctionalDescriptor dfuDescriptor;
};
static_assert(sizeof(UsbConfiguration) <= 256);

static const UsbConfiguration configurationDescriptor = {
    .config = {
        .wTotalLength = sizeof(UsbConfiguration),
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = usb::ConfigurationDescriptorAttriubtes::BUS_POWERED,
        .bMaxPower = 50 // 100 mA
    },
    .dfuInterface = {
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0, // no endpoints, DFU only uses control endpoint 0
        .bInterfaceClass = usb::InterfaceClass::APPLICATION,
        .bInterfaceSubClass = usb::InterfaceSubClass::DFU,
        .bInterfaceProtocol = usb::InterfaceProtocol::DFU_MODE,
        .iInterface = uint8_t(StringId::DFU_INTERFACE) // index into string table
    },
    .dfuDescriptor = {
        .bmAttributes = dfu::Attributes::CAN_DNLOAD | dfu::Attributes::MANIFESTATION_TOLERANT,
        .wDetachTimeout = 100,
        .wTransferSize = CONTROL_BUFFER_SIZE
    }
};


// string descriptors
const usb::StringDescriptor<1> languages = {
    .wString = {0x0409} // English (United States)
};
const auto manufacturerString = usb::makeStringDescriptor(u"CoCo");
const auto productString = usb::makeStringDescriptor(u"Dfu-Test");
const auto serialString = usb::makeStringDescriptor(u"1337");
const auto dfuInterfaceString = usb::makeStringDescriptor(u"Dfu-Test DFU interface");


// binary object store (BOS) descriptor with MS OS 2.0 platform capability descriptor
struct Bos {
    usb::BosDescriptor bos;
    msos20::PlatformCapabilityDescriptorHeader header;
    msos20::DescriptorSetInformation info;
};
static_assert(sizeof(Bos) <= CONTROL_BUFFER_SIZE);


// descriptor that instructs Windows to use WinUSB driver
// Creates registry entry: HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\VID_1209&PID_0003\12345
// Example: https://github.com/pololu/libusbp/blob/master/test/firmware/wixel/main.c
struct WinUsbDescriptor {
    msos20::DescriptorSetHeader header;
    msos20::CompatibleIdDescriptor compatibleId;
    msos20::RegistryPropertyDescriptor<21, 40> registryProperty; // length of PropertyName and PropertyData
};
static_assert(sizeof(WinUsbDescriptor) <= CONTROL_BUFFER_SIZE);

static const Bos bos {
    .bos = {
        .wTotalLength = sizeof(Bos),
        .bNumDeviceCaps = 1
    },
    .header = {
        .bLength = sizeof(Bos) - sizeof(usb::BosDescriptor),
    },
    .info = {
        .wMSOSDescriptorSetTotalLength = sizeof(WinUsbDescriptor),
        .bMS_VendorCode = uint8_t(VendorRequest::WINUSB)
    }
};

static const WinUsbDescriptor winUsbDescriptor {
    .header = {
        .wTotalLength = sizeof(WinUsbDescriptor),
    },
    .compatibleId = {
        .CompatibleID = "WINUSB"
    },
    .registryProperty = {
        // list of utf-16 strings
        .wPropertyDataType = msos20::PropertyDatatype::MULTI_SZ,

        // property DeviceInterfaceGUIDs
        .PropertyName = u"DeviceInterfaceGUIDs",

        // custom generated UUID for our device (additional zero termination for list of strings)
        .PropertyData = u"{cabf2319-8394-49c1-98c8-12656d393ce0}\0"
    }
};



namespace global {
    uint32_t currentCrc = 0xffffffff;
}

// handle control requests
Coroutine control(UsbDevice &usb, Buffer &buffer) {
    dfu::State dfuState = dfu::State::DFU_IDLE;
    while (true) {
        // wait for a control request (https://www.beyondlogic.org/usbnutshell/usb6.shtml)
        co_await usb.untilRequest();
        auto setup = usb.getSetup();

        // handle request
        switch (setup.bmRequestType) {
        case usb::RequestType::STANDARD_DEVICE_IN:
            switch (setup.bRequest) {
            case usb::Request::GET_DESCRIPTOR:
                {
                    auto descriptorType = usb::DescriptorType(setup.wValue >> 8);
                    //int descriptorIndex = setup.value & 0xff;
                    switch (descriptorType) {
                    case usb::DescriptorType::DEVICE:
                        // send device descriptor to host
                        co_await UsbDevice::controlIn(buffer, setup, deviceDescriptor);
                        break;
                    case usb::DescriptorType::CONFIGURATION:
                        // send configuration descriptor to host
                        co_await UsbDevice::controlIn(buffer, setup, configurationDescriptor);
                        break;
                    case usb::DescriptorType::STRING:
                        switch (StringId(setup.wValue & 0xff)) {
                        case StringId::LANGUAGES:
                            co_await UsbDevice::controlIn(buffer, setup, languages);
                            break;
                        case StringId::MANUFACTURER:
                            co_await UsbDevice::controlIn(buffer, setup, manufacturerString);
                            break;
                        case StringId::PRODUCT:
                            co_await UsbDevice::controlIn(buffer, setup, productString);
                            break;
                        case StringId::SERIAL:
                            co_await UsbDevice::controlIn(buffer, setup, serialString);
                            break;
                        case StringId::DFU_INTERFACE:
                            co_await UsbDevice::controlIn(buffer, setup, dfuInterfaceString);
                            break;
                        default:
                            usb.stall();
                        }
                        break;
                    case usb::DescriptorType::BOS:
                        // send BOS descriptor to host
                        co_await UsbDevice::controlIn(buffer, setup, bos);
                        break;
                    default:
                        usb.stall();
                    }
                }
                break;
            default:
                // unknown request
                usb.stall();
            }
            break;
        case usb::RequestType::CLASS_INTERFACE_IN:
            switch (setup.bRequest) {
            case dfu::Request::DFU_UPLOAD:
                //debug::out << "DFU_UPLOAD wValue " << dec(setup.wValue) << " wLength " << dec(setup.wLength) << '\n';

                // not supported
                usb.stall();
                break;
            case dfu::Request::DFU_GETSTATE:
                buffer.cast<dfu::State &>() = dfuState;
                co_await buffer.write(std::min(int(setup.wLength), 1));
                break;
            case dfu::Request::DFU_GETSTATUS:
                debug::out << "DFU_GETSTATUS wValue " << dec(setup.wValue) << " wLength " << dec(setup.wLength) << '\n';
                {
                    switch (dfuState) {
                    case dfu::State::DFU_IDLE:
                        global::currentCrc = 0xffffffff; // reset CRC
                        break;
                    case dfu::State::DFU_DNLOAD_SYNC:
                        dfuState = dfu::State::DFU_DNLOAD_IDLE;
                        break;
                    case dfu::State::DFU_MANIFEST_SYNC:
                        dfuState = dfu::State::DFU_IDLE;
                        break;
                    case dfu::State::DFU_MANIFEST:
                        //dfuState = dfu::State::DFU_MANIFEST_WAIT_RESET; // manifestation tolerant bit not set
                        dfuState = dfu::State::DFU_MANIFEST_SYNC; // manifestation tolerant bit set
                        break;
                    default:
                        ;
                    }

                    auto &status = buffer.cast<dfu::StatusReport &>();
                    status = {
                        dfu::Status::OK,
                        200, // 200ms
                        dfuState,
                        0}; // status description string index
                    co_await buffer.write(std::min(int(setup.wLength), int(sizeof(dfu::StatusReport))));
                }
                break;
            default:
                // unknown request
                //debug::set(debug::MAGENTA);
                usb.stall();
            }
            break;
        case usb::RequestType::CLASS_INTERFACE_OUT:
            switch (setup.bRequest) {
            case dfu::Request::DFU_CLRSTATUS:
                debug::out << "DFU_CLRSTATUS\n";
                usb.acknowledge();
                break;
            case dfu::Request::DFU_DNLOAD:
                // download firmware
                //debug::out << "DFU_DNLOAD wValue " << dec(setup.wValue) << " wLength " << dec(setup.wLength) << '\n';
                if (dfuState == dfu::State::DFU_IDLE || dfuState == dfu::State::DFU_DNLOAD_IDLE) {
                    if (setup.wLength > 0) {
                        // download section
                        co_await buffer.read(setup.wLength);
                        global::currentCrc = crc::instance().calc(global::currentCrc, buffer.cast<uint32_t *>(), setup.wLength >> 2); // todo: pad to size divisible by 4
                        dfuState = dfu::State::DFU_DNLOAD_SYNC;
                    } else {
                        // download finished
                        debug::out << "download finished, CRC = " << hex(global::currentCrc) << '\n';
                        usb.acknowledge();
                        dfuState = dfu::State::DFU_MANIFEST_SYNC;
                    }
                } else {
                    // not idle for download
                    usb.stall();
                }
                break;
            case dfu::Request::DFU_ABORT:
                debug::out << "DFU_ABORT\n";
                usb.acknowledge();
                switch (dfuState) {
                case dfu::State::DFU_DNLOAD_IDLE:
                    dfuState = dfu::State::DFU_IDLE;
                    break;
                default:
                    ;
                }
                break;
            default:
                // unknown request
                usb.stall();
            }
            break;
        case usb::RequestType::VENDOR_DEVICE_IN:
            switch (VendorRequest(setup.bRequest)) {
            case VendorRequest::WINUSB:
                // send WinUSB descriptor to host
                if (setup.wIndex == 0x07) {
                    //debug::out << "WinUSB\n";
                    co_await UsbDevice::controlIn(buffer, setup, winUsbDescriptor);
                } else {
                    usb.stall();
                }
                break;
            default:
                usb.stall();
            }
            break;
        default:
            // unknown request type
            usb.stall();
        }
    }
}

int main() {
    debug::out << "Dfu-Test\n";

    control(drivers.usb, drivers.controlBuffer);

    drivers.loop.run();
    return 0;
}
