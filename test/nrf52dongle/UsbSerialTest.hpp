#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/UsbDevice_USBD.hpp>
#include <coco/board/config.hpp>


// endpoint addresses
constexpr int COMM_IN = 0x82;
constexpr int DATA_OUT = 0x01;
constexpr int DATA_IN = 0x81;


using namespace coco;

// drivers for UsbSerialTest
struct Drivers {
	Loop_RTC0 loop;

	// USB
	using Usb = UsbDevice_USBD;
	Usb usb{loop};
	Usb::ControlBuffer<256> controlBuffer{usb};
	Usb::Endpoint commEndpoint{usb, COMM_IN & 0x7f, 0};
	Usb::Endpoint dataEndpoint{usb, DATA_IN & 0x7f, DATA_OUT};
	Usb::Buffer<64> commBuffer{commEndpoint};
	Usb::Buffer<128> dataBuffer{dataEndpoint};
};

Drivers drivers;

extern "C" {
void USBD_IRQHandler() {
	drivers.usb.USBD_IRQHandler();
}
}
