#pragma once

#include <coco/usb.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbHost_native.hpp>


// drivers for UsbTestHost
struct Drivers {
	coco::Loop_native loop;
	coco::UsbHost_native host{loop};
	coco::UsbHost_native::Device device{host, [](const coco::usb::DeviceDescriptor &deviceDescriptor) {
		return deviceDescriptor.idVendor == 0x1915 && deviceDescriptor.idProduct == 0x1337;
	}};
	coco::UsbHost_native::BulkEndpoint endpoint1{device, 1};
	coco::UsbHost_native::BulkEndpoint endpoint2{device, 2};
};
