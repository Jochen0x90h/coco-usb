#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbDevice_cout.hpp>


// drivers for UsbDeviceTest
struct Drivers {
	using UsbDevice = coco::UsbDevice_cout;
	coco::Loop_native loop;
	UsbDevice device{loop};
	UsbDevice::BulkEndpoint endpoint1{device, 1};
	UsbDevice::BulkEndpoint endpoint2{device, 2};
};
