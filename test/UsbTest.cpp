#include <UsbTest.hpp>
#include <coco/convert.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>


/*
    Test for USB device.
    Flash this onto the device, then run UsbTestHost on a PC
    Windows: May need Zadig to install WinUSB driver
*/


using namespace coco;
namespace msos20 = usb::msos20;

constexpr int USB_VID = 0x1209; // https://pid.codes/1209/
constexpr int USB_PID = 0x0001; // test PID
constexpr int USB_DEVICE_VERSION = 0x0100; // device version 1.00

// string id
enum class StringId : uint8_t {
    LANGUAGES = 0,
    MANUFACTURER = 1,
    PRODUCT = 2,
    SERIAL = 3,
};

// vendor specific control request
enum class VendorRequest : uint8_t {
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    COLOR = 3,
    DEVICE_ID = 4,
    VARIANT_ID = 5,
    WINUSB = 6,
};


// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
    .bcdUSB = 0x0210, // USB version 2.10 to support BOS descriptor
    .bDeviceClass = usb::DeviceClass::VENDOR, // vendor specific
    .bDeviceSubClass = 0xff,
    .bDeviceProtocol = 0, // 0 = binary, 1 = text
    .bMaxPacketSize0 = 64, // max packet size for endpoint 0
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_DEVICE_VERSION,
    .iManufacturer = uint8_t(StringId::MANUFACTURER), // index into string table
    .iProduct = uint8_t(StringId::PRODUCT), // index into string table
    .iSerialNumber = uint8_t(StringId::SERIAL), // index into string table
    .bNumConfigurations = 1
};


// configuration descriptor
struct UsbConfiguration {
    usb::ConfigurationDescriptor config;
    usb::InterfaceDescriptor interface;
    usb::EndpointDescriptor endpoints[4]; // number of endpoints must match the number of endpoint initializers
};
static_assert(sizeof(UsbConfiguration) <= CONTROL_BUFFER_SIZE);

static const UsbConfiguration configurationDescriptor = {
    .config = {
        .wTotalLength = sizeof(UsbConfiguration),
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = usb::ConfigurationDescriptorAttriubtes::BUS_POWERED,
        .bMaxPower = 50 // 100 mA
    },
    .interface = {
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = std::size(configurationDescriptor.endpoints),
        .bInterfaceClass = usb::InterfaceClass::VENDOR, // vendor specific, no pre-defined class
        .bInterfaceSubClass = 0xff,
        .bInterfaceProtocol = 0xff,
        .iInterface = 0
    },
    .endpoints = {
        {
            .bEndpointAddress = 1 | usb::IN, // 1 in (device to host)
            .bmAttributes = usb::EndpointType::INTERRUPT,
            .wMaxPacketSize = 64,
            .bInterval = 1 // polling interval
        },
        {
            .bEndpointAddress = 1 | usb::OUT, // 1 out (host to device)
            .bmAttributes = usb::EndpointType::INTERRUPT,
            .wMaxPacketSize = 64,
            .bInterval = 1 // polling interval
        },
        {
            .bEndpointAddress = 2 | usb::IN, // 2 in (device to host)
            .bmAttributes = usb::EndpointType::BULK,
            .wMaxPacketSize = 64,
            .bInterval = 1 // polling interval
        },
        {
            .bEndpointAddress = 2 | usb::OUT, // 2 out (host to device)
            .bmAttributes = usb::EndpointType::BULK,
            .wMaxPacketSize = 64,
            .bInterval = 1 // polling interval
        }
    }
};


// string descriptors
const usb::StringDescriptor<1> languages = {
    .wString = {0x0409} // English (United States)
};
const auto manufacturerString = usb::makeStringDescriptor(u"CoCo");
const auto productString = usb::makeStringDescriptor(u"UsbTest");
const auto serialString = usb::makeStringDescriptor(u"12345");


// binary object store (BOS) descriptor with MS OS 2.0 platform capability descriptor
struct Bos {
    usb::BosDescriptor bos;
    msos20::PlatformCapabilityDescriptorHeader header;
    msos20::DescriptorSetInformation info;
};
static_assert(sizeof(Bos) <= CONTROL_BUFFER_SIZE);

// descriptor that instructs Windows to use WinUSB driver
// Creates registry entry: HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\VID_1209&PID_0001\12345
// Example: https://github.com/pololu/libusbp/blob/master/test/firmware/wixel/main.c
struct WinUsbDescriptor {
    msos20::DescriptorSetHeader header;
    //msos20::ConfigurationSubsetHeader configHeader;
    //msos20::FunctionSubsetHeader functionHeader;
    msos20::CompatibleIdDescriptor compatibleId;
    msos20::RegistryPropertyDescriptor<21, 40> registryProperty;
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
    /*.configHeader = {
        .bConfigurationValue = 0,
        .wTotalLength = sizeof(WinUsbDescriptor) - sizeof(msos20::DescriptorSetHeader),
    },
    .functionHeader = {
        .bFirstInterface = 0,
        .wSubsetLength = sizeof(WinUsbDescriptor) - sizeof(msos20::DescriptorSetHeader) - sizeof(msos20::ConfigurationSubsetHeader),
    },*/
    .compatibleId = {
        .CompatibleID = "WINUSB"//{'W','I','N','U','S','B',0,0}
    },
    .registryProperty = {
        // list of utf-16 strings
        .wPropertyDataType = msos20::PropertyDatatype::MULTI_SZ,

        // property DeviceInterfaceGUIDs
        .PropertyName = u"DeviceInterfaceGUIDs",//{'D','e','v','i','c','e','I','n','t','e','r','f','a','c','e','G','U','I','D','s',0},

        // custom generated UUID for our device (additional zero termination for list of strings)
        .PropertyData = u"{cabf2319-8394-49c1-98c8-12656d393ce0}\0"//{'{','f','6','1','3','4','4','d','8','-','e','f','1','d','-','4','3','e','e','-','8','8','7','c','-','b','b','a','7','2','0','1','8','f','a','c','f','}',0,0}
    }
};



// handle control requests
Coroutine control(UsbDevice &usb, Buffer &buffer) {
    while (true) {
        // wait for a control request (https://www.beyondlogic.org/usbnutshell/usb6.shtml)
        //debug::out << "Waiting for request...\n";
        co_await usb.untilRequest();
        auto setup = usb.getSetup();

        //debug::out << "R " << hex(setup.bmRequestType) << ' ' << hex(setup.bRequest) << ' ' << hex(setup.wValue) << ' ' << hex(setup.wIndex) << ' ' << hex(setup.wLength) << '\n';

        // handle request
        switch (setup.bmRequestType) {
        case usb::RequestType::STANDARD_DEVICE_IN:
            switch (setup.bRequest) {
            case usb::Request::GET_DESCRIPTOR:
                {
                    // get descriptor type and index from high and low bytes of wValue
                    auto descriptorType = usb::DescriptorType(setup.wValue >> 8);
                    switch (descriptorType) {
                    case usb::DescriptorType::DEVICE:
                        // send device descriptor to host
                        //debug::set(debug::CYAN);
                        co_await UsbDevice::controlIn(buffer, setup, deviceDescriptor);
                        break;
                    case usb::DescriptorType::CONFIGURATION:
                        // send configuration descriptor to host
                        //debug::set(debug::MAGENTA);
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
                usb.stall();
            }
            break;
        case usb::RequestType::VENDOR_DEVICE_OUT:
            switch (VendorRequest(setup.bRequest)) {
            case VendorRequest::RED:
                // acknowledge when reading no data from control endpoint
                usb.acknowledge();
                debug::setRed(setup.wValue != 0);
                break;
            case VendorRequest::GREEN:
                // acknowledge when reading no data from control endpoint
                usb.acknowledge();
                debug::setGreen(setup.wIndex != 0);
                break;
            case VendorRequest::BLUE:
                // acknowledge when reading no data from control endpoint
                usb.acknowledge();
                debug::setBlue(setup.wValue == setup.wIndex);
                break;
            case VendorRequest::COLOR:
                {
                    // read color from control endpoint
                    co_await buffer.read(setup.wLength);
                    debug::set(buffer.array<uint32_t>()[0]);
                }
                break;
            default:
                usb.stall();
            }
            break;
        case usb::RequestType::VENDOR_DEVICE_IN:
            switch (VendorRequest(setup.bRequest)) {
            case VendorRequest::DEVICE_ID:
                // send device id to host
                {
#ifdef NATIVE
                    uint32_t value = 0;
#else
                    uint32_t value = getDeviceId();
#endif
#ifdef NRF52
                    //value = NRF_UICR->NRFFW[0]; // flash end
#endif
                    co_await UsbDevice::controlIn(buffer, setup, value);
                }
                break;
            case VendorRequest::VARIANT_ID:
                // send variant id to host
                {
#ifdef NATIVE
                    uint32_t value = 0;
#else
                    uint32_t value = getVariantId();
#endif
                    co_await UsbDevice::controlIn(buffer, setup, value);
                }
                break;
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
            usb.stall();
        }
    }
}

uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
Coroutine write(Loop &loop, Device &device,  Buffer &buffer) {
    while (true) {
        // wait until USB device is connected
        co_await device.untilReady();

        while (buffer.ready()) {
            // send data to host
            co_await buffer.writeArray(data);

            co_await loop.sleep(500ms);
        }
    }
}

// echo data from host
Coroutine echo(Loop &loop, Device &device, Buffer &buffer) {
    while (true) {
        debug::set(debug::BLUE);

        // wait until USB device is connected
        co_await device.untilReady();

        while (buffer.ready()) {
            debug::set(debug::MAGENTA);

            // receive data from host
            co_await buffer.read();
            int transferred = buffer.size();

            debug::set(debug::GREEN);

            // check received data
            bool error = false;
            for (int i = 0; i < transferred; ++i) {
                if (buffer[i] != transferred + i)
                    error = true;
            }
            if (error)
                debug::set(debug::RED);

            // send data back to host
            co_await buffer.write(transferred);
            //co_await loop.sleep(300ms);
        }
    }
}



int main() {
    debug::out << "UsbTest\n";

    // start to receive from usb host
    control(drivers.usb, drivers.controlBuffer);
#ifdef NATIVE
    // emulated device: only write to the dummy device
    write(drivers.loop, drivers.usb, drivers.buffer1);
    write(drivers.loop, drivers.usb, drivers.buffer2);
#else
    // echo data coming from the host
    echo(drivers.loop, drivers.usb, drivers.buffer1);
    echo(drivers.loop, drivers.usb, drivers.buffer2);
#endif

    drivers.loop.run();
    return 0;
}
