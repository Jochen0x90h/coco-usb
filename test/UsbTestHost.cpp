#include <coco/Coroutine.hpp>
#include <coco/loop.hpp>
#include <coco/String.hpp>
#include <coco/StringBuffer.hpp>
#include <coco/usb.hpp>
#include <coco/platform/UsbHost_native.hpp>
#include <iostream>
#include <iomanip>


// Host for UsbDeviceTest
// Flash UsbDeviceTest onto the device, then run this


using namespace coco;
using UsbHost = UsbHost_native;


std::ostream &operator <<(std::ostream &s, String v) {
	return s << std::string(v.data(), v.size());
}

struct dec {
	dec(int i) : i(i) {}
	int i;
};
std::ostream &operator <<(std::ostream &s, dec v) {
	return s << std::dec << v.i;
}

struct hex {
	hex(uint8_t i) : w(2), i(i) {}
	hex(uint16_t i) : w(4), i(i) {}
	int w;
	int i;
};
std::ostream &operator <<(std::ostream &s, hex v) {
	return s << std::setfill('0') << std::setw(v.w) << std::hex << v.i;
}

/*
// https://github.com/libusb/libusb/blob/master/examples/listdevs.c
static void printDevices(libusb_device **devices) {
	libusb_device *device;
	int i = 0, j = 0;
	uint8_t path[8]; 

	// iterate over devices
	while ((device = devices[i++]) != nullptr) {
		libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(device, &desc);
		if (r < 0) {
			std::cerr << "failed to get device descriptor" << std::endl;
			return;
		}

		std::cout << hex(desc.idVendor) << ':' << hex(desc.idProduct)
			<< " (bus " << dec(libusb_get_bus_number(device)) << ", device " << dec(libusb_get_device_address(device)) << ")";

		r = libusb_get_port_numbers(device, path, sizeof(path));
		if (r > 0) {
			std::cout << " path: " << dec(path[0]);
			for (j = 1; j < r; j++)
				std::cout << "." << dec(path[j]);
		}
		std::cout << std::endl;
	}
}

// https://github.com/libusb/libusb/blob/master/examples/testlibusb.c
static void printDevice(libusb_device *dev, int idVendor = 0, int idProduct = 0) {
	libusb_device_descriptor desc;
	libusb_device_handle *handle = NULL;
	unsigned char string[256];
	int ret;

	ret = libusb_get_device_descriptor(dev, &desc);
	if (ret < 0) {
		std::cerr << "failed to get device descriptor" << std::endl;
		return;
	}

	if ((idVendor | idProduct) != 0 && (desc.idVendor != idVendor || desc.idProduct != idProduct))
		return;

	std::cout << hex(desc.idVendor) << ':' << hex(desc.idProduct) << std::endl;
	std::cout << "\tBus: " << dec(libusb_get_bus_number(dev)) << std::endl;
	std::cout << "\tDevice: " << dec(libusb_get_device_address(dev)) << std::endl;
	ret = libusb_open(dev, &handle);
	if (LIBUSB_SUCCESS == ret) {
		std::cout << "\tOpen" << std::endl;

		// manufacturer
		if (desc.iManufacturer) {
			ret = libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, string, sizeof(string));
			if (ret > 0)
				std::cout << "\t\tManufacturer: " << str(string) << std::endl;
		}

		// product
		if (desc.iProduct) {
			ret = libusb_get_string_descriptor_ascii(handle, desc.iProduct, string, sizeof(string));
			if (ret > 0)
				std::cout << "\t\tProduct: " << str(string) << std::endl;
		}
		
	
		libusb_close(handle);
	} else {
		std::cout << "\tOpen error: " << dec(ret) << std::endl;
	}

	// configurations
	for (int i = 0; i < desc.bNumConfigurations; i++) {
		libusb_config_descriptor *config;
		ret = libusb_get_config_descriptor(dev, i, &config);
		if (LIBUSB_SUCCESS == ret) {
			std::cout << "\tConfiguration[" << dec(i) << "]" << std::endl;
			std::cout << "\t\tTotalLength:         " << dec(config->wTotalLength) << std::endl;
			std::cout << "\t\tNumInterfaces:       " << dec(config->bNumInterfaces) << std::endl;
			std::cout << "\t\tConfigurationValue:  " << dec(config->bConfigurationValue) << std::endl;
			std::cout << "\t\tConfiguration:       " << dec(config->iConfiguration) << std::endl;
			std::cout << "\t\tAttributes:          " << hex(config->bmAttributes) << std::endl;
			std::cout << "\t\tMaxPower:            " << dec(config->MaxPower) << std::endl;
		}
		
		// interfaces
		for (int j = 0; j < config->bNumInterfaces; j++) {
			libusb_interface const & interface = config->interface[j];
			
			// alternate settings
			for (int k = 0; k < interface.num_altsetting; k++) {
				libusb_interface_descriptor const & descriptor = interface.altsetting[k];

				std::cout << "\t\tInterface[" << dec(j) << "][" << dec(k) << "]" << std::endl;
				//std::cout << "\t\t\tInterfaceNumber:   %d\n" << dec(descriptor.bInterfaceNumber) << std::endl;
				//std::cout << "\t\t\tAlternateSetting:  %d\n" << dec(descriptor.bAlternateSetting) << std::endl;
				std::cout << "\t\t\tNumEndpoints:      " << dec(descriptor.bNumEndpoints) << std::endl;
				std::cout << "\t\t\tInterfaceClass:    " << dec(descriptor.bInterfaceClass) << std::endl;
				std::cout << "\t\t\tInterfaceSubClass: " << dec(descriptor.bInterfaceSubClass) << std::endl;
				std::cout << "\t\t\tInterfaceProtocol: " << dec(descriptor.bInterfaceProtocol) << std::endl;
				std::cout << "\t\t\tInterface:         " << dec(descriptor.iInterface) << std::endl;

				// endpoints
				for (int l = 0; l < descriptor.bNumEndpoints; l++) {
					libusb_endpoint_descriptor const & endpoint = descriptor.endpoint[l];
					
					std::cout << "\t\t\tEndpoint[" << dec(l) << "]" << std::endl;
					std::cout << "\t\t\t\tEndpointAddress: " << hex(endpoint.bEndpointAddress) << std::endl;
					std::cout << "\t\t\t\tAttributes:      " << hex(endpoint.bmAttributes) << std::endl;
					std::cout << "\t\t\t\tMaxPacketSize:   " << dec(endpoint.wMaxPacketSize) << std::endl;
					std::cout << "\t\t\t\tInterval:        " << dec(endpoint.bInterval) << std::endl;
					std::cout << "\t\t\t\tRefresh:         " << dec(endpoint.bRefresh) << std::endl;
					std::cout << "\t\t\t\tSynchAddress:    " << dec(endpoint.bSynchAddress) << std::endl;
				}
			}
		
		}
		
		libusb_free_config_descriptor(config);
	}
}
*/

// vendor specific control request
enum class Request : uint8_t {
	RED = 0,
	GREEN = 1,
	BLUE = 2
};

inline Awaitable<UsbHostDevice::ControlParameters> controlOut(UsbHostDevice &device, Request request,
	uint16_t wValue, uint16_t wIndex)
{
	return device.controlTransfer(usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), wValue, wIndex, nullptr, 0);
}


// print test status
void printLength(int endpoint, int length) {
	std::cout << endpoint << ": length: " << dec(length) << std::endl;
}

void printStatus(int endpoint, String message, bool ok) {
	std::cout << endpoint << ": " << message << ": ";
	std::cout << (ok ? "ok" : "error!");
	std::cout << std::endl;
}

Coroutine handler(UsbHostDevice &device, Stream &stream, int endpoint) {
	StringBuffer<129> buffer;

	while (true) {
		// wait until device is connected
		std::cout << "wait for device to be connected" << std::endl;
		co_await device.targetState(UsbHostDevice::State::CONNECTED);

		// test control request
		co_await controlOut(device, Request::RED, 1, 0); // set on if wValue != 0
		co_await controlOut(device, Request::RED, 0, 0);
		co_await controlOut(device, Request::GREEN, 0, 1); // set on if wIndex != 0
		co_await controlOut(device, Request::GREEN, 0, 0);
		co_await controlOut(device, Request::BLUE, 5, 5); // set on if wValue == wIndex
		co_await controlOut(device, Request::BLUE, 0, 256);

		// flush out data from last run
		for (int i = 0; i < 4; ++i) {
			int r = co_await select(stream.read(buffer), loop::sleep(100ms));
			//std::cout << i << ' ' << r << std::endl;
		}

		// echo loop: send data to device and check if we get back the same data
		int sendLength = 128;
		bool allOk = true;
		while (device.isConnected()) {

			// send to device
			buffer.resize(sendLength);
			for (int i = 0; i < sendLength; ++i) {
				buffer[i] = sendLength + i;
			}
			co_await stream.write(buffer);
			printStatus(endpoint, "send", buffer.size() == sendLength);
			allOk &= buffer.size() == sendLength;

			// send zero length packet to indicate that transfer is complete if length is multiple of 64
			if (sendLength > 0 && (sendLength & 63) == 0) {
				co_await stream.write(nullptr, 0);
			}

			// receive from device (we get back the same data that we sent)
			// note: usb driver does not wait for the zero length packet if device sends 128 bytes, therefore use 129 instead of 128
			co_await stream.read(buffer);
			printStatus(endpoint, "receive", buffer.size() == sendLength);
			allOk &= buffer.size() == sendLength;

			// check received data
			bool ok = true;
			for (int i = 0; i < buffer.size(); ++i) {
				if (buffer[i] != char(buffer.size() + i))
					ok = false;
			}
			printStatus(endpoint, "data", ok);
			allOk &= ok;

			printStatus(endpoint, "all", allOk);

			// wait
			co_await loop::sleep(100ms);

			// modify the send length
			sendLength = (sendLength + 5) % 129;
		}
	}
}

int main() {
	UsbHost host;
	UsbHost::Device device(host, [](const usb::DeviceDescriptor &deviceDescriptor) {
		return deviceDescriptor.idVendor == 0x1915 && deviceDescriptor.idProduct == 0x1337;
	});
	UsbHost::BulkEndpoint endpoint1(device, 1);
	UsbHost::BulkEndpoint endpoint2(device, 2);

	handler(device, endpoint1, 1);
	//handler(device, endpoint2, 2);

	loop::run();

	return 0;
}
