#include <coco/String.hpp>
#include <coco/usb.hpp>
#include "libusb.hpp"
#include <iostream>
#include <iomanip>


// Host for UsbDeviceTest
// Flash UsbDeviceTest onto the device, then run this


using namespace coco;


std::ostream &operator <<(std::ostream &s, String v) {
	return s << std::string(v.data(), v.size());
}

struct str {
	str(const char *s) : s(s) {}
	str(const unsigned char *s) : s((const char *)s) {}
	const char *s;
};
std::ostream &operator <<(std::ostream &s, str v) {
	return s << v.s;
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

// vendor specific control request
enum class Request : uint8_t {
	RED = 0,
	GREEN = 1,
	BLUE = 2
};

// sent vendor specific control request to usb device
int controlOut(libusb_device_handle *handle, Request request, uint16_t wValue, uint16_t wIndex) {
	return libusb_control_transfer(handle,
		uint8_t(usb::Request::VENDOR_DEVICE_OUT),
		uint8_t(request),
		wValue,
		wIndex,
		nullptr,
		0,
		1000);
}

// print test status
void printStatus(String message, bool status) {
	std::cout << message << ": " << (status ? "ok" : "error") << std::endl;
}

int main() {
	int r = libusb_init(nullptr);
	if (r < 0)
		return r;

	// get device list
	libusb_device **devices;
	ssize_t cnt = libusb_get_device_list(nullptr, &devices);
	if (cnt < 0) {
		libusb_exit(nullptr);
		return (int) cnt;
	}

	// print list of devices
	printDevices(devices);
	for (int i = 0; devices[i]; ++i) {
		printDevice(devices[i], 0x1915, 0x1337);
	}

	// iterate over devices
	for (int deviceIndex = 0; devices[deviceIndex]; ++deviceIndex) {
		libusb_device *dev = devices[deviceIndex];

		// get device descriptor
		libusb_device_descriptor desc;
		int ret = libusb_get_device_descriptor(dev, &desc);
		if (ret != LIBUSB_SUCCESS) {
			std::cerr << "get device descriptor error: " << dec(ret) << std::endl;
			continue;
		}
		
		// check vendor/product
		if (desc.idVendor != 0x1915 && desc.idProduct != 0x1337)
			continue;

		// print device
		//printDevice(devs[i]);

		// protocol: 0: binary, 1: text
		bool text = desc.bDeviceProtocol == 1;

		// open device
		libusb_device_handle *handle;
		ret = libusb_open(dev, &handle);
		if (ret != LIBUSB_SUCCESS) {
			std::cerr << "open error: " << dec(ret) << std::endl;
			continue;
		}

		if (libusb_kernel_driver_active(handle, 0) == 1) {
			std::cout << "detach active kernel driver" << std::endl;
			ret = libusb_detach_kernel_driver(handle, 0);
			if (ret != LIBUSB_SUCCESS) {
				std::cout << "detach kernel driver error: " << dec(ret) << std::endl;
				continue;
			}
		}
			
		// set configuration (reset alt_setting, reset toggles)
		ret = libusb_set_configuration(handle, 1);
		if (ret != LIBUSB_SUCCESS) {
			std::cerr << "set configuration error: " << dec(ret) << std::endl;
			continue;
		}

		// claim interface with bInterfaceNumber = 0
		ret = libusb_claim_interface(handle, 0);
		if (ret != LIBUSB_SUCCESS) {
			std::cerr << "claim interface error: " << dec(ret) << std::endl;
			continue;
		}

		//ret = libusb_set_interface_alt_setting(handle, 0, 0);
		//std::cout << "set alternate setting " << dec(ret) << std::endl;


		// test control request
		ret = controlOut(handle, Request::RED, 1, 0); // set on if wValue != 0
		ret = controlOut(handle, Request::RED, 0, 0);
		ret = controlOut(handle, Request::GREEN, 0, 1); // set on if wIndex != 0
		ret = controlOut(handle, Request::GREEN, 0, 0);
		ret = controlOut(handle, Request::BLUE, 5, 5); // set on if wValue == wIndex
		ret = controlOut(handle, Request::BLUE, 0, 256);

		
		uint8_t buffer[129] = {};
		int transferred;

		// flush out data from last run
		for (int i = 0; i < 4; ++i)
			ret = libusb_bulk_transfer(handle, 1 | usb::IN, buffer, 129, &transferred, 100);

		// echo loop: send data to device and check if we get back the same data
		int sendLength = 128;
		bool allOk = true;
		for (int iter = 0; iter < 10000000; ++iter) {
			std::cout << "length: " << dec(sendLength) << std::endl;

			// send to device
			for (int i = 0; i < sendLength; ++i) {
				buffer[i] = sendLength + i;
			}
			ret = libusb_bulk_transfer(handle, 1 | usb::OUT, buffer, sendLength, &transferred, 10000);
			if (ret == LIBUSB_ERROR_TIMEOUT)
				std::cout << "send timeout!" << std::endl;
			printStatus("sent", ret == 0 && transferred == sendLength);
			allOk &= ret == 0 && transferred == sendLength;
			
			// send zero length packet to indicate that transfer is complete if length is multiple of 64
			if (sendLength > 0 && (sendLength & 63) == 0)
				libusb_bulk_transfer(handle, 1 | usb::OUT, buffer, 0, &transferred, 1000);

			// debug: check if one packet of maximum 64 bytes can be written while the device is not waiting in UsbDevice::receive()
			for (int i = 0; i < 45; ++i) buffer[i] = 45 + i;
			ret = libusb_bulk_transfer(handle, 1 | usb::OUT, buffer, 45, &transferred, 10000);
			if (ret == LIBUSB_ERROR_TIMEOUT)
				std::cout << "debug send timeout!" << std::endl;

			// receive from device (we get back the same data that we sent)
			// note: libusb does not wait for the zero length packet if device sends 128 bytes, therefore use 129 instead of 128
			ret = libusb_bulk_transfer(handle, 1 | usb::IN, buffer, 129, &transferred, 10000);
			if (ret == LIBUSB_ERROR_TIMEOUT)
				std::cout << "receive timeout!" << std::endl;
			//printStatus("received", ret == 0 && transferred == sendLength);
			std::cout << "received: ";
			if (ret != 0) {
				std::cout << "error " << dec(ret) << std::endl;
			} else if (transferred != sendLength) {
				std::cout << dec(transferred) << " != " << dec(sendLength) << std::endl;
			} else {
				std::cout << "ok" << std::endl;
			}
			allOk &= ret == 0 && transferred == sendLength;


			// check received data
			bool ok = true;
			for (int i = 0; i < transferred; ++i) {
				if (buffer[i] != transferred + i)
					ok = false;
			}
			printStatus("data", ok);
			allOk &= ok;

			printStatus("all", allOk);

			// debug: remove additional packet
			ret = libusb_bulk_transfer(handle, 1 | usb::IN, buffer, 45, &transferred, 10000);
			for (int i = 0; i < 45; ++i) allOk &= buffer[i] == 45 + i;

			// wait
			//usleep(1000000);

			// modify the send length
			sendLength = (sendLength + 5) % 129;
		}
		libusb_close(handle);
		break;
	}

	libusb_free_device_list(devices, 1);
	libusb_exit(nullptr);
	return 0;
}
