#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/UsbDevice_USBD.hpp>


using namespace coco;

// drivers for UsbDeviceTest
struct Drivers {
	Loop_RTC0 loop;
	UsbDevice_USBD device{loop};
	UsbDevice_USBD::ControlBuffer<256> controlBuffer{device};
	UsbDevice_USBD::BulkEndpoint endpoint1{device, 1};
	UsbDevice_USBD::BulkEndpoint endpoint2{device, 2};
	UsbDevice_USBD::BulkBuffer<129> buffer1{endpoint1};
	UsbDevice_USBD::BulkBuffer<129> buffer2{endpoint2};
};
