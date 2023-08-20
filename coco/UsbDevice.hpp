#pragma once

#include "usb.hpp"
#include <coco/Device.hpp>


namespace coco {

/**
	Interface for USB device support

	https://www.beyondlogic.org/usbnutshell/usb1.shtml
*/
class UsbDevice : public Device {
public:
	/**
		Wait for a request from the host
		@param setup contents of the setup packet where setup.requestType determines the direction
		@return use co_await on return value to wait for a request
	*/
	[[nodiscard]] virtual Awaitable<usb::Setup *> request(usb::Setup &setup) = 0;

	/**
		Acknowledge a conrol request without data stage
	*/
	virtual void acknowledge() = 0;

	/**
		Indicate that a control request is not supported or has invalid parameters
	*/
	virtual void stall() = 0;

	/**
		Helper for control in transfers, e.g. sending a descriptor to the host
	*/
	template <typename T>
	[[nodiscard]] static Awaitable<> controlIn(Buffer &buffer, usb::Setup const &setup, const T &data) {
		int size = std::min(int(setup.length), int(sizeof(data)));
		return buffer.writeData(&data, size);
	}
};

} // namespace coco
