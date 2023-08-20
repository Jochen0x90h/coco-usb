#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <UsbDeviceTest.hpp>


// Test for USB device.
// Flash this onto the device, then run UsbTestHost on a PC

using namespace coco;


// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
	.bLength = sizeof(usb::DeviceDescriptor),
	.bDescriptorType = usb::DescriptorType::DEVICE,
	.bcdUSB = 0x0200, // USB 2.0
	.bDeviceClass = 0xff, // no class
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
	struct usb::ConfigurationDescriptor config;
	struct usb::InterfaceDescriptor interface;
	struct usb::EndpointDescriptor endpoints[4];
};

static const UsbConfiguration configurationDescriptor = {
	.config = {
		.bLength = sizeof(usb::ConfigurationDescriptor),
		.bDescriptorType = usb::DescriptorType::CONFIGURATION,
		.wTotalLength = sizeof(UsbConfiguration),
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0x80, // bus powered
		.bMaxPower = 50 // 100 mA
	},
	.interface = {
		.bLength = sizeof(usb::InterfaceDescriptor),
		.bDescriptorType = usb::DescriptorType::INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = std::size(configurationDescriptor.endpoints), // number of endpoints, don't forget to call usb.enableEndpoints()
		.bInterfaceClass = 0xff, // no class
		.bInterfaceSubClass = 0xff,
		.bInterfaceProtocol = 0xff,
		.iInterface = 0
	},
	.endpoints = {{
		.bLength = sizeof(usb::EndpointDescriptor),
		.bDescriptorType = usb::DescriptorType::ENDPOINT,
		.bEndpointAddress = 1 | usb::IN, // 1 in (device to host)
		.bmAttributes = usb::EndpointType::BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1 // polling interval
	},
	{
		.bLength = sizeof(usb::EndpointDescriptor),
		.bDescriptorType = usb::DescriptorType::ENDPOINT,
		.bEndpointAddress = 1 | usb::OUT, // 1 out (host to device)
		.bmAttributes = usb::EndpointType::BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1 // polling interval
	},
	{
		.bLength = sizeof(usb::EndpointDescriptor),
		.bDescriptorType = usb::DescriptorType::ENDPOINT,
		.bEndpointAddress = 2 | usb::IN, // 1 in (device to host)
		.bmAttributes = usb::EndpointType::BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1 // polling interval
	},
	{
		.bLength = sizeof(usb::EndpointDescriptor),
		.bDescriptorType = usb::DescriptorType::ENDPOINT,
		.bEndpointAddress = 2 | usb::OUT, // 1 out (host to device)
		.bmAttributes = usb::EndpointType::BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1 // polling interval
	}}
};


// vendor test request
enum class Request : uint8_t {
	RED = 0,
	GREEN = 1,
	BLUE = 2,
	INFO = 3
};


// helpers

/*
template <typename T>
[[nodiscard]] Awaitable<Buffer::State> controlIn(Buffer &buffer, usb::Setup const &setup, const T &data) {
	int size = std::min(int(setup.length), int(sizeof(data)));

	auto d = reinterpret_cast<const uint8_t *>(&data);
	std::copy(d, d + size, buffer.data());

	return buffer.write(size);
}*/


// handle control requests
Coroutine control(UsbDevice &device, Buffer &buffer) {
	while (true) {
		usb::Setup setup;

		// wait for a control request (https://www.beyondlogic.org/usbnutshell/usb6.shtml)
		co_await device.request(setup);

		// handle request
		switch (setup.requestType) {
		case usb::RequestType::STANDARD_DEVICE_IN:
			switch (setup.request) {
			case usb::Request::GET_DESCRIPTOR:
				{
					auto descriptorType = usb::DescriptorType(setup.value >> 8);
					//int descriptorIndex = setup.value & 0xff;
					switch (descriptorType) {
					case usb::DescriptorType::DEVICE:
						debug::set(debug::CYAN);
						co_await UsbDevice::controlIn(buffer, setup, deviceDescriptor);
						break;
					case usb::DescriptorType::CONFIGURATION:
						debug::set(debug::MAGENTA);
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
		case usb::RequestType::VENDOR_DEVICE_OUT:
			switch (Request(setup.request)) {
			case Request::RED:
				device.acknowledge();
				debug::setRed(setup.value != 0);
				break;
			case Request::GREEN:
				device.acknowledge();
				debug::setGreen(setup.index != 0);
				break;
			case Request::BLUE:
				device.acknowledge();
				debug::setBlue(setup.value == setup.index);
				break;
			case Request::INFO:
				{
					// read uint32 and set as debug color
					co_await buffer.read(setup.length);
					debug::set(buffer.array<uint32_t>()[0]);
				}
				break;
			default:
				device.stall();
			}
			break;
		case usb::RequestType::VENDOR_DEVICE_IN:
			switch (Request(setup.request)) {
			case Request::INFO:
				{
					uint32_t value = 0x01020304;
#ifdef NRF52
					value = NRF_FICR->INFO.VARIANT; // nrf52840 chip id
					//value = NRF_UICR->NRFFW[0]; // flash end
#endif
					co_await UsbDevice::controlIn(buffer, setup, value);
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

uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
Coroutine write(Loop &loop, Buffer &buffer) {
	while (true) {
		// wait until device is connected
		co_await buffer.untilReady();

		while (buffer.ready()) {
			// send data to host
			co_await buffer.writeArray(data);

			co_await loop.sleep(500ms);
		}
	}
}

// echo data from host
Coroutine echo(Loop &loop, Buffer &buffer) {
	while (true) {
		debug::set(debug::YELLOW);

		// wait until device is connected
		co_await buffer.untilReady();

		while (buffer.ready()) {
			debug::set(debug::MAGENTA);

			// receive data from host
			co_await buffer.read();
			int transferred = buffer.transferred();

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
	debug::init();
	Drivers drivers;

	// start to receive from usb host
	control(drivers.device, drivers.controlBuffer);
#ifdef NATIVE
	// emulated device: only write to the dummy device
	write(drivers.loop, drivers.buffer1);
	write(drivers.loop, drivers.buffer2);
#else
	// echo data coming from the host
	echo(drivers.loop, drivers.buffer1);
	echo(drivers.loop, drivers.buffer2);
#endif

	drivers.loop.run();
	return 0;
}
