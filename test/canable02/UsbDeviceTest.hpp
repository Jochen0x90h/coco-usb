#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/UsbDevice_USB.hpp>
#include <coco/board/config.hpp>


using namespace coco;

// drivers for UsbDeviceTest
struct Drivers {
	Loop_TIM2 loop{APB1_TIMER_CLOCK};

	using Usb = UsbDevice_USB;
	Usb usb{loop};
	Usb::ControlBuffer<256> controlBuffer{usb};
	Usb::BulkEndpoint endpoint1{usb, 1};
	Usb::BulkEndpoint endpoint2{usb, 2};
	Usb::BulkBuffer<129> buffer1{endpoint1};
	Usb::BulkBuffer<129> buffer2{endpoint2};
};

Drivers drivers;

extern "C" {
void USB_IRQHandler() {
	drivers.usb.USB_IRQHandler();
}
}
