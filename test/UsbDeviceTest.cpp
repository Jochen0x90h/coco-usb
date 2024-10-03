#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <UsbDeviceTest.hpp>


// Test for USB device.
// Flash this onto the device, then run UsbTestHost on a PC

using namespace coco;


// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
	.bDeviceClass = usb::DeviceClass::VENDOR, // vendor specific
	.bDeviceSubClass = 0xff,
	.bDeviceProtocol = 0, // 0 = binary, 1 = text
	.bMaxPacketSize0 = 64, // max packet size for endpoint 0
	.idVendor = 0x1915, // Nordic Semiconductor
	.idProduct = 0x1337,
	.bcdDevice = 0x0100, // device version
	.iManufacturer = 0, // index into string table
	.iProduct = 0, // index into string table
	.iSerialNumber = 0, // index into string table
	.bNumConfigurations = 1
};

// configuration descriptor
struct UsbConfiguration {
	usb::ConfigurationDescriptor config;
	usb::InterfaceDescriptor interface;
	usb::EndpointDescriptor endpoints[4]; // number of endpoints must match the number of endpoint initializers
};
static_assert(sizeof(UsbConfiguration) <= 256);

static const UsbConfiguration configurationDescriptor = {
	.config = {
		.wTotalLength = sizeof(UsbConfiguration),
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0x80, // bus powered
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


// vendor specific control request
enum class Request : uint8_t {
	RED = 0,
	GREEN = 1,
	BLUE = 2,
	COLOR = 3,
	DEVICE_ID = 4,
	VARIANT_ID = 5
};



// handle control requests
Coroutine control(UsbDevice &usb, Buffer &buffer) {
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
						//debug::set(debug::CYAN);
						co_await UsbDevice::controlIn(buffer, setup, deviceDescriptor);
						break;
					case usb::DescriptorType::CONFIGURATION:
						// send configuration descriptor to host
						//debug::set(debug::MAGENTA);
						co_await UsbDevice::controlIn(buffer, setup, configurationDescriptor);
						break;
					//case usb::DescriptorType::STRING:
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
			switch (Request(setup.bRequest)) {
			case Request::RED:
				// acknowledge when reading no data from control endpoint
				usb.acknowledge();
				debug::setRed(setup.wValue != 0);
				break;
			case Request::GREEN:
				// acknowledge when reading no data from control endpoint
				usb.acknowledge();
				debug::setGreen(setup.wIndex != 0);
				break;
			case Request::BLUE:
				// acknowledge when reading no data from control endpoint
				usb.acknowledge();
				debug::setBlue(setup.wValue == setup.wIndex);
				break;
			case Request::COLOR:
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
			switch (Request(setup.bRequest)) {
			case Request::DEVICE_ID:
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
			case Request::VARIANT_ID:
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
