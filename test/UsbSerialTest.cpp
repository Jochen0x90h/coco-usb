#include <UsbSerialTest.hpp>
#include <coco/Loop.hpp>
#include <coco/convert.hpp>
#include <coco/debug.hpp>


/*
    Virtual COM port test that echos back everything it receives.
    Open a terminal program (e.g. PuTTY or HTerm) and connect to the virtual COM port. Then type something.

    USB subclass specification for PSTN devices: https://gzuliani.github.io/arduino/files/arduino-android-usb/PSTN120.pdf
    Other resources:
    https://github.com/manuelbl/usb-serial
    https://community.st.com/t5/stm32-mcus-embedded-software/usb-cdc-how-send-serial-state-notification/td-p/157193
    https://gist.github.com/tai/acd59b125a007ad47767
    https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
    https://ww1.microchip.com/downloads/en/DeviceDoc/01247a.pdf
    https://github.com/manuelbl/usb-serial/blob/master/firmware/src/usb_conf.cpp

    Also supports DFU in runtime mode. Run
    $ dfu-util -l
    Which should list our device:
    Found Runtime: [0483:5740] ...
*/

using namespace coco;
namespace cdc = usb::cdc;

constexpr int USB_VID = 0x1209; // https://pid.codes/1209/
constexpr int USB_PID = 0x0002; // test PID
constexpr int USB_DEVICE_VERSION = 0x0200; // 2.00

// interface numbers
constexpr int INTF_COMM = 0;
constexpr int INTF_DATA = 1;
constexpr int INTF_DFU = 2;
constexpr int INTF_COUNT = 3;

// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
    .bDeviceClass = usb::DeviceClass::CDC,
    .bDeviceSubClass = usb::DeviceSubClass::CDC_ACM,
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

    usb::InterfaceDescriptor commInterface;
    cdc::FunctionalDescriptors cdcFunctionalDescriptors;
    usb::EndpointDescriptor commEndpoints[1];

    usb::InterfaceDescriptor dataInterface;
    usb::EndpointDescriptor dataEndpoints[2];

    usb::InterfaceDescriptor dfuInterface;
    usb::dfu::FunctionalDescriptor dfuDescriptor;
};
static_assert(sizeof(UsbConfiguration) <= 256);

static const UsbConfiguration configurationDescriptor = {
    .config = {
        .wTotalLength = sizeof(UsbConfiguration),
        .bNumInterfaces = INTF_COUNT,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = usb::ConfigurationDescriptorAttriubtes::BUS_POWERED,
        .bMaxPower = 50 // 100 mA
    },

    .commInterface = {
        .bInterfaceNumber = INTF_COMM,
        .bAlternateSetting = 0,
        .bNumEndpoints = std::size(configurationDescriptor.commEndpoints), // number of endpoints in this interface
        .bInterfaceClass = usb::InterfaceClass::CDC_CONTROL,
        .bInterfaceSubClass = usb::InterfaceSubClass::CDC_ACM,
        .bInterfaceProtocol = 0,//usb::InterfaceProtocol::AT,
        .iInterface = 0 // index into string table
    },
    .cdcFunctionalDescriptors = {
        .callManagement = {
            .bmCapabilities = 0, // no call management
            .bDataInterface = INTF_DATA,
        },
        .acm = {
            .bmCapabilities = cdc::AcmCapability::LINE_CODING,
        },
        .cdcUnion = {
            .bControlInterface = INTF_COMM,
            .bSubordinateInterface0 = INTF_DATA,
        }
    },
    .commEndpoints = {
        {
            .bEndpointAddress = COMM_IN,
            .bmAttributes = usb::EndpointType::INTERRUPT,
            .wMaxPacketSize = 64,
            .bInterval = 16 // polling interval
        }
    },

    .dataInterface = {
        .bInterfaceNumber = INTF_DATA,
        .bAlternateSetting = 0,
        .bNumEndpoints = std::size(configurationDescriptor.dataEndpoints), // number of endpoints in this interface
        .bInterfaceClass = usb::InterfaceClass::CDC_DATA,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0 // index into string table
    },
    .dataEndpoints = {
        {
            .bEndpointAddress = DATA_OUT,
            .bmAttributes = usb::EndpointType::BULK,
            .wMaxPacketSize = 64,
            .bInterval = 0 // polling interval
        },
        {
            .bEndpointAddress = DATA_IN,
            .bmAttributes = usb::EndpointType::BULK,
            .wMaxPacketSize = 64,
            .bInterval = 0 // polling interval
        }
    },

    .dfuInterface = {
        .bInterfaceNumber = INTF_DFU,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0,
        .bInterfaceClass = usb::InterfaceClass::APPLICATION,
        .bInterfaceSubClass = usb::InterfaceSubClass::DFU,
        .bInterfaceProtocol = usb::InterfaceProtocol::DFU_RUNTIME,
        .iInterface = 0 // index into string table
    },
    .dfuDescriptor = {
        .bmAttributes = usb::dfu::Attributes::WILL_DETACH,
        .wDetachTimeout = 100,
        .wTransferSize = CONTROL_BUFFER_SIZE
    }
};


int baudRate = 38400;

// handle control requests
Coroutine control(UsbDevice &device, Buffer &buffer) {
    while (true) {
        // wait for a control request (https://www.beyondlogic.org/usbnutshell/usb6.shtml)
        co_await device.untilRequest();
        auto setup = device.getSetup();

        //debug::out << "Req " << hex(setup.bmRequestType) << ' ' << hex(setup.bRequest) << ' ' << hex(setup.wValue) << ' ' << hex(setup.wIndex) << ' ' << hex(setup.wLength) << '\n';

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
                device.stall();
            }
            break;
        case usb::RequestType::CLASS_INTERFACE_IN:
            // select interface by index
            switch (setup.wIndex) {
            case INTF_COMM:
                // read request to COMM interface
                switch (setup.bRequest) {
                case cdc::PstnRequest::GET_LINE_CODING:
                    // get line coding (baud rate, frame format)
                    debug::out << "GET_LINE_CODING\n";
                    {
                        auto &lineCoding = buffer.value<cdc::PstnLineCoding>();
                        lineCoding.dwDTERate = baudRate;
                        lineCoding.bCharFormat = 0; // 1 stop bit
                        lineCoding.bDataBits = 8; // 8 data bits
                        lineCoding.bParityType = 0; // no parity

                        co_await buffer.write(std::min(int(setup.wLength), int(sizeof(cdc::PstnLineCoding))));
                    }
                    break;
                default:
                    // unknown request
                    //debug::set(debug::MAGENTA);
                    device.stall();
                }
            case INTF_DFU:
                switch (setup.bRequest) {
                case usb::dfu::Request::DFU_GETSTATUS:
                    {
                        auto &status = buffer.value<usb::dfu::StatusReport>();
                        status = {
                            usb::dfu::Status::OK,
                            200, 0, // 200ms
                            usb::dfu::State::APP_IDLE,
                            0 // status description string index
                        };
                        co_await buffer.write(std::min(int(setup.wLength), int(sizeof(usb::dfu::StatusReport))));
                    }
                default:
                    device.stall();
                }
                break;
            default:
                // unknown interface
                device.stall();
            }
            break;
        case usb::RequestType::CLASS_INTERFACE_OUT:
            // select interface by index
            switch (setup.wIndex) {
            case INTF_COMM:
                // write request to COMM interface
                switch (setup.bRequest) {
                case cdc::PstnRequest::SET_LINE_CODING:
                    // set line coding (baud rate, frame format)
                    if (setup.wLength >= sizeof(cdc::PstnLineCoding)) {
                        co_await buffer.read(setup.wLength);
                        auto &lineCoding = buffer.value<cdc::PstnLineCoding>();

                        baudRate = lineCoding.dwDTERate;
                        debug::out << "SET_LINE_CODING " << dec(baudRate) << ' ' << dec(lineCoding.bDataBits) << '\n';
                    } else {
                        device.stall();
                    }
                    break;
                case cdc::PstnRequest::SET_CONTROL_LINE_STATE:
                    // set control line state (RTS, DTR)
                    {
                        bool dtr = (setup.wValue & 1) != 0;
                        bool rts = (setup.wValue & 2) != 0;

                        // acknowledge when reading no data from control endpoint
                        device.acknowledge();

                        // debug output
                        debug::setGreen(dtr);
                        debug::setRed(rts);
                        debug::out << "DTR: " << (dtr ? "on" : "off") << ", RTS: " << (rts ? "on" : "off") << '\n';
                    }
                    break;
                default:
                    // unknown request
                    device.stall();
                }
                break;
            case INTF_DFU:
                // write request to DFU interface
                switch (setup.bRequest) {
                case usb::dfu::Request::DFU_DETACH:
                    device.acknowledge();
                    debug::out << "DFU_DETACH\n";

                    // jump to bootloader here

                    break;
                default:
                    // unknown request
                    device.stall();
                }
                break;
            default:
                // unknown interface
                device.stall();
            }
            break;
        default:
            // unknown request type
            device.stall();
        }
    }
}


// echo data from host
Coroutine echo(Device &device, Buffer &buffer) {
    while (true) {
        // wait until USB device is connected
        co_await device.untilReady();

        while (device.ready()) {
            // receive data from host
            co_await buffer.read();

            // send data back to host
            co_await buffer.write();
        }
    }
}


// simulate status changes
struct StatusMessage {
    usb::Setup header;
    cdc::PstnSerialState status;
};
Coroutine status(Loop &loop, Device &device, Buffer &buffer) {
    while (true) {
        // wait until USB device is connected
        co_await device.untilReady();

        int count = 0;
        while (device.ready()) {
            int gray = count ^ (count >> 1);

            co_await buffer.writeValue<StatusMessage>({
                {usb::RequestType(0xA1), cdc::PstnNotification::SERIAL_STATE, 0, 0, 2},
                cdc::PstnSerialState((gray & 3) | ((gray & 4) << 1)) // DCD, DSR and RingSignal
            });

            co_await loop.sleep(1s);

            //debug::set(gray);
            ++count;
        }
    }
}

int main() {
    debug::setGreen();
    debug::out << "UsbSerialTest\n";

    control(drivers.usb, drivers.controlBuffer);
    echo(drivers.usb, drivers.dataBuffer);
    status(drivers.loop, drivers.usb, drivers.commBuffer);

    drivers.loop.run();
    return 0;
}
