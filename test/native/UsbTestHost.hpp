#pragma once

#include <coco/usb.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbHost_native.hpp>


using namespace coco;

// drivers for UsbTestHost
struct Drivers {
	Loop_native loop;
	UsbHost_native host{loop};
	UsbHost_native::Device device{host, [](const usb::DeviceDescriptor &deviceDescriptor) {
		return deviceDescriptor.idVendor == 0x1915 && deviceDescriptor.idProduct == 0x1337;
	}};
	UsbHost_native::ControlBuffer controlBuffer{device, 32};
	UsbHost_native::BulkEndpoint endpoint1{device, 1};
	UsbHost_native::BulkBuffer buffer1{endpoint1, 129};
	UsbHost_native::BulkEndpoint endpoint2{device, 2};
	UsbHost_native::BulkBuffer buffer2{endpoint2, 129};
};
