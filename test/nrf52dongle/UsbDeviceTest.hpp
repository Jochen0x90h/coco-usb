#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/UsbDevice_USBD.hpp>


// drivers for UsbDeviceTest
struct Drivers {
	using UsbDevice = coco::UsbDevice_USBD;
	coco::Loop_RTC0 loop;
	UsbDevice device{loop};
	UsbDevice::BulkEndpoint endpoint1{device, 1};
	UsbDevice::BulkEndpoint endpoint2{device, 2};
};
