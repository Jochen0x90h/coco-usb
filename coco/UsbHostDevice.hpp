#pragma once

#include "usb.hpp"
#include <coco/Device.hpp>


namespace coco {

/**
	Interface for an USB device from the host side

	https://www.beyondlogic.org/usbnutshell/usb1.shtml
*/
class UsbHostDevice {
public:

	using State = Device::State;

	virtual ~UsbHostDevice();

	/**
		Get the current state of the device
	*/
	virtual State state() = 0;
	bool disabled() {return state() == State::DISABLED;}
	bool ready() {return state() == State::READY;}

	/**
		Wait until the device is in the given state (e.g. co_await device.untilState(UsbHostDevice::State::READY))
		@param state state to wait for
		@return use co_await on return value to await the given state
	*/
	[[nodiscard]] virtual Awaitable<State> untilState(State state) = 0;
	[[nodiscard]] Awaitable<State> untilDisabled() {return untilState(State::DISABLED);}
	[[nodiscard]] Awaitable<State> untilReady() {return untilState(State::READY);}


/*	class ControlBuffer : public Buffer {
	public:
		ControlBuffer(uint8_t *data, int capacity, State state) : Buffer(data, capacity, state) {}
		~ControlBuffer() override;

		virtual void setSetup(const usb::Setup &setup) = 0;
	};*/
/*
	virtual void getDescriptor(usb::DescriptorType type, void *data, int &size) = 0;

	template <int N>
	void getDescriptor(DeviceDescriptorBuffer<N> &buffer) {
		buffer.length = N;
		getDescriptor(usb::DescriptorType::DEVICE, buffer.data, buffer.length);
	}
	template <int N>
	void getDescriptor(ConfigurationDescriptorBuffer<N> &buffer) {
		buffer.length = N;
		getDescriptor(usb::DescriptorType::CONFIGURATION, buffer.data, buffer.length);
	}
*/

	//virtual void setInterface(int index) = 0;
};

} // namespace coco
