#include "UsbDeviceOut.hpp"
#include <coco/loop.hpp>
#include <iostream>
#include <iomanip>


namespace coco {

UsbDeviceOut::UsbDeviceOut(
	std::function<ConstData (usb::DescriptorType)> const &getDescriptor,
	std::function<void (UsbDevice &usb, uint8_t bConfigurationValue)> const &onSetConfiguration,
	std::function<bool (uint8_t bRequest, uint16_t wValue, uint16_t wIndex)> const &onRequest)
{
	// get device descriptor
	auto *deviceDescriptor = getDescriptor(usb::DescriptorType::DEVICE).cast<usb::DeviceDescriptor>();
	this->text = deviceDescriptor->bDeviceProtocol == 1;
	
	// set configuration
	this->onSetConfiguration = onSetConfiguration;

	// call onSetConfiguration from event loop
	this->time = loop::now();
	coco::timeHandlers.add(*this);
}

void UsbDeviceOut::enableEndpoints(uint8_t inFlags, uint8_t outFlags) {	
	std::cout << "enable in";
	for (int i = 0; i < 8; ++i) {
		if (inFlags & (1 << i))
			std::cout << ' ' << i;
	}
	std::cout << " out";
	for (int i = 0; i < 8; ++i) {
		if (outFlags & (1 << i))
			std::cout << ' ' << i;
	}
	std::cout << std::endl;
}

Awaitable<UsbDevice::ReceiveParameters> UsbDeviceOut::receive(int index, void *data, int &length) {
	assert(index == 1);
	auto &endpoint = this->endpoints[index - 1];
	if (!isInList()) {
		this->time = loop::now() + 10ms; // emulate 10ms transfer time
		coco::timeHandlers.add(*this);
	}
	return {endpoint.receiveWaitlist, data, length};
}

Awaitable<UsbDevice::SendParameters> UsbDeviceOut::send(int index, void const *data, int length) {
	assert(index == 1);
	auto &endpoint = this->endpoints[index - 1];
	if (!isInList()) {
		this->time = loop::now() + 10ms; // emulate 10ms transfer time
		coco::timeHandlers.add(*this);
	}
	return {endpoint.sendWaitlist, data, length};
}

void UsbDeviceOut::activate() {
	this->remove();

	// call onSetConfiguration once
	if (this->onSetConfiguration != nullptr) {
		this->onSetConfiguration(*this, 1);
		this->onSetConfiguration = nullptr;
	}

	int endpointIndex = 1;
	for (auto &endpoint : this->endpoints) {
		endpoint.sendWaitlist.resumeFirst([this, endpointIndex](SendParameters p) {
			if (this->text) {
				std::cout << std::string(reinterpret_cast<char const *>(p.data), p.length);
			} else {
				// binary
				std::cout << endpointIndex << ": ";
				for (int i = 0; i < p.length; ++i) {
					if ((i & 15) == 0) {
						if (i != 0)
							std::cout << "," << std::endl;
					} else {
						std::cout << ", ";
					}
					std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(reinterpret_cast<uint8_t const *>(p.data)[i]);
				}
				std::cout << '(' << p.length << ')' << std::endl;
			}
			return true;
		});
		++endpointIndex;
	}
}

} // namespace coco
