#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/UsbDevice_USBD.hpp>


using namespace coco;

// drivers for UsbDeviceTest
struct Drivers {
	Loop_RTC0 loop;

	using Usb = UsbDevice_USBD;
	Usb usb{loop};
	Usb::ControlBuffer<256> controlBuffer{usb};
	Usb::Endpoint endpoint1{usb, 1};
	Usb::Endpoint endpoint2{usb, 2};
	Usb::Buffer<129> buffer1{endpoint1};
	Usb::Buffer<129> buffer2{endpoint2};
};

Drivers drivers;

extern "C" {
void USBD_IRQHandler() {
	drivers.usb.USBD_IRQHandler();
}
}
