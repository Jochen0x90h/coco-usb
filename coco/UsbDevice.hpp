#pragma once

#include "usb.hpp"
#include <coco/Device.hpp>


namespace coco {

/**
	Interface for USB device support

	https://www.beyondlogic.org/usbnutshell/usb1.shtml
*/
class UsbDevice {
public:

	using State = Device::State;

	virtual ~UsbDevice();

	/**
		Get the current state of the device
	*/
	virtual State state() = 0;
	bool disabled() {return state() == State::DISABLED;}
	bool ready() {return state() == State::READY;}

	/**
		Wait until the device is in the given target state (e.g. co_await device.untilState(UsbDevice::State::READY))
		@param state target state
		@return use co_await on return value to await the given state
	*/
	[[nodiscard]] virtual Awaitable<State> untilState(State state) = 0;
	[[nodiscard]] Awaitable<State> untilDisabled() {return untilState(State::DISABLED);}
	[[nodiscard]] Awaitable<State> untilReady() {return untilState(State::READY);}

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
	[[nodiscard]] static Awaitable<Buffer::State> controlIn(Buffer &buffer, usb::Setup const &setup, const T &data) {
		int size = std::min(int(setup.length), int(sizeof(data)));
		return buffer.writeData(reinterpret_cast<const uint8_t *>(&data), size);
	}
};

} // namespace coco
