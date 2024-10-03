#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <UsbSerialTest.hpp>


/*
	Virtual COM port test that echos back everything it receives
	USB subclass specification for PSTN devices: https://gzuliani.github.io/arduino/files/arduino-android-usb/PSTN120.pdf
	Other resources:
	https://github.com/manuelbl/usb-serial
	https://community.st.com/t5/stm32-mcus-embedded-software/usb-cdc-how-send-serial-state-notification/td-p/157193
	https://gist.github.com/tai/acd59b125a007ad47767
	https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-interface-association-descriptor
	https://ww1.microchip.com/downloads/en/DeviceDoc/01247a.pdf
	https://github.com/manuelbl/usb-serial/blob/master/firmware/src/usb_conf.cpp
*/

using namespace coco;


constexpr int USB_VID = 0x0483; // STMicroelectronics
constexpr int USB_PID = 0x5740; // Virtual COM Port
constexpr int USB_DEVICE_REL = 0x0200; // 2.00

constexpr int INTF_COMM = 0;
constexpr int INTF_DATA = 1;

// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
	.bDeviceClass = usb::DeviceClass::CDC,
	.bDeviceSubClass = usb::DeviceSubClass::CDC_ACM,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64, // max packet size for endpoint 0
	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = USB_DEVICE_REL, // device version
	.iManufacturer = 0, // index into string table
	.iProduct = 0, // index into string table
	.iSerialNumber = 0, // index into string table
	.bNumConfigurations = 1
};

// configuration descriptor
struct UsbConfiguration {
	usb::ConfigurationDescriptor config;
	usb::InterfaceDescriptor commInterface;
	usb::CdcFunctionalDescriptors cdcFunctionalDescriptors;
	usb::EndpointDescriptor commEndpoints[1];
	usb::InterfaceDescriptor dataInterface;
	usb::EndpointDescriptor dataEndpoints[2];
};
static_assert(sizeof(UsbConfiguration) <= 256);

static const UsbConfiguration configurationDescriptor = {
	.config = {
		.wTotalLength = sizeof(UsbConfiguration),
		.bNumInterfaces = 2,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0x80, // bus powered
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
			.bmCapabilities = usb::CdcAcmCapability::LINE_CODING,
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
	}
};


int baudRate = 38400;

// handle control requests
Coroutine control(UsbDevice &device, Buffer &buffer) {
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
				device.stall();
			}
			break;
		case usb::RequestType::CLASS_INTERFACE_IN:
			switch (setup.bRequest) {
			case usb::PstnRequest::GET_LINE_CODING:
				// get line coding (baud rate, frame format)
				{
					auto &lineCoding = buffer.value<usb::PstnLineCoding>();
					lineCoding.dwDTERate = baudRate;
					lineCoding.bCharFormat = 0; // 1 stop bit
					lineCoding.bDataBits = 8; // 8 data bits
					lineCoding.bParityType = 0; // no parity

					co_await buffer.write(std::min(int(setup.wLength), int(sizeof(usb::PstnLineCoding))));
				}
				break;
			default:
				//debug::set(debug::MAGENTA);
				device.stall();
			}
			break;
		case usb::RequestType::CLASS_INTERFACE_OUT:
			switch (setup.bRequest) {
			case usb::PstnRequest::SET_LINE_CODING:
				// set line coding (baud rate, frame format)
				if (setup.wLength >= sizeof(usb::PstnLineCoding) && setup.wIndex == 0) {
					co_await buffer.read(setup.wLength);
					auto &lineCoding = buffer.value<usb::PstnLineCoding>();

					baudRate = lineCoding.dwDTERate;
				} else {
					device.stall();
				}
				break;
			case usb::PstnRequest::SET_CONTROL_LINE_STATE:
				// set control line state (RTS, DTR)
				{
					bool dtr = (setup.wValue & 1) != 0;
					bool rts = (setup.wValue & 2) != 0;

					// set LEDs
					debug::setRed(dtr);
					debug::setGreen(rts);

					// acknowledge when reading no data from control endpoint
					device.acknowledge();
				}
				break;
			default:
				device.stall();
			}
			break;
		default:
			device.stall();
		}
	}
}


// echo data from host
Coroutine echo(Loop &loop, Device &device, Buffer &buffer) {
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
Coroutine status(Loop &loop, Device &device, Buffer &buffer) {
	while (true) {
		// wait until USB device is connected
		co_await device.untilReady();

		buffer.setHeader<usb::Setup>({usb::RequestType(0xA1), usb::PstnNotification::SERIAL_STATE, 0, 0, 2});

		int count = 0;
		while (device.ready()) {
			int gray = count ^ (count >> 1);

			buffer.value<uint16_t>() = (gray & 3) | ((gray & 4) << 1); // DCD, DSR and RingSignal
			co_await buffer.write(2);

			co_await loop.sleep(1s);

			debug::set(gray);
			++count;
		}
	}
}

int main() {
	control(drivers.usb, drivers.controlBuffer);
	echo(drivers.loop, drivers.usb, drivers.dataBuffer);
	status(drivers.loop, drivers.usb, drivers.commBuffer);

	drivers.loop.run();
	return 0;
}
