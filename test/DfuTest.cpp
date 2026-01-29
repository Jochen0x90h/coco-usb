#include <DfuTest.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/StreamOperators.hpp>


/*
    DFU (Device Firmware Upgrade) test for device in DFU mode (see UsbSerialTest for device in run-time mode)
    USB class specification: https://www.usb.org/sites/default/files/DFU_1.1.pdf)
    Windows: May need Zadig to install WinUSB driver

    List devices in DFU mode: $ dfu-util -l
    Download firmware into device: $ dfu-util -D firmware.bin
*/

using namespace coco;
namespace dfu = usb::dfu;

constexpr int USB_VID = 0x1209; // https://pid.codes/1209/
constexpr int USB_PID = 0x0003; // test PID
constexpr int USB_DEVICE_VERSION = 0x0100; // version 1.00


// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
    .bDeviceClass = usb::DeviceClass::NONE,
    .bDeviceSubClass = usb::DeviceSubClass::NONE,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64, // max packet size for endpoint 0
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_DEVICE_VERSION, // device version
    .iManufacturer = 0, // index into string table
    .iProduct = 0, // index into string table
    .iSerialNumber = 0, // index into string table
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
        .bNumEndpoints = 0,
        .bInterfaceClass = usb::InterfaceClass::APPLICATION,
        .bInterfaceSubClass = usb::InterfaceSubClass::DFU,
        .bInterfaceProtocol = usb::InterfaceProtocol::DFU_MODE,
        .iInterface = 0 // index into string table
    },
    .dfuDescriptor = {
        .bmAttributes = dfu::Attributes::CAN_DNLOAD | dfu::Attributes::MANIFESTATION_TOLERANT,
        .wDetachTimeout = 100,
        .wTransferSize = CONTROL_BUFFER_SIZE
    }
};

namespace global {
    uint32_t currentCrc = 0xffffffff;
}

// handle control requests
Coroutine control(UsbDevice &device, Buffer &buffer) {
    dfu::State dfuState = dfu::State::DFU_IDLE;
    while (true) {
        // wait for a control request (https://www.beyondlogic.org/usbnutshell/usb6.shtml)
        co_await device.untilRequest();
        auto setup = device.getSetup();

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
                    //case usb::DescriptorType::STRING:
                    default:
                        device.stall();
                    }
                }
                break;
            default:
                // unknown request
                device.stall();
            }
            break;
        case usb::RequestType::CLASS_INTERFACE_IN:
            switch (setup.bRequest) {
            case dfu::Request::DFU_UPLOAD:
                //debug::out << "DFU_UPLOAD wValue " << dec(setup.wValue) << " wLength " << dec(setup.wLength) << '\n';

                // not supported
                device.stall();
                break;
                case dfu::Request::DFU_GETSTATE:
                    buffer.value<dfu::State>() = dfuState;
                    co_await buffer.write(std::min(int(setup.wLength), 1));
                    break;
            case dfu::Request::DFU_GETSTATUS:
                //debug::out << "DFU_GETSTATUS wValue " << dec(setup.wValue) << " wLength " << dec(setup.wLength) << '\n';
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

                    auto &status = buffer.value<dfu::StatusReport>();
                    status = {
                        dfu::Status::OK,
                        200, 0,
                        dfuState,
                        0
                    };
                    co_await buffer.write(std::min(int(setup.wLength), int(sizeof(dfu::StatusReport))));
                }
                break;
            default:
                // unknown request
                //debug::set(debug::MAGENTA);
                device.stall();
            }
            break;
        case usb::RequestType::CLASS_INTERFACE_OUT:
            switch (setup.bRequest) {
            case dfu::Request::DFU_CLRSTATUS:
                debug::out << "DFU_CLRSTATUS\n";
                device.acknowledge();
                break;
            case dfu::Request::DFU_DNLOAD:
                // download firmware
                //debug::out << "DFU_DNLOAD wValue " << dec(setup.wValue) << " wLength " << dec(setup.wLength) << '\n';
                if (dfuState == dfu::State::DFU_IDLE || dfuState == dfu::State::DFU_DNLOAD_IDLE) {
                    if (setup.wLength > 0) {
                        // download section
                        co_await buffer.read(setup.wLength);
                        global::currentCrc = crc::instance().calc(global::currentCrc, buffer.pointer<uint32_t>(), setup.wLength >> 2);
                        dfuState = dfu::State::DFU_DNLOAD_SYNC;
                    } else {
                        // download finished
                        debug::out << "download finished, CRC = " << hex(global::currentCrc) << '\n';
                        device.acknowledge();
                        dfuState = dfu::State::DFU_MANIFEST_SYNC;
                    }
                } else {
                    // not idle for download
                    device.stall();
                }
                break;
            case dfu::Request::DFU_ABORT:
                debug::out << "DFU_ABORT\n";
                device.acknowledge();
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
                device.stall();
            }
            break;
        default:
            // unknown request type
            device.stall();
        }
    }
}

int main() {
    debug::out << "DfuTest\n";

    control(drivers.usb, drivers.controlBuffer);

    drivers.loop.run();
    return 0;
}
