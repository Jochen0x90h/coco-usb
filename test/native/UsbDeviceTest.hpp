#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbDevice_cout.hpp>


using namespace coco;

// drivers for UsbDeviceTest
struct Drivers {
	Loop_native loop;

	// "fake" USB device
	UsbDevice_cout device{loop};
	UsbDevice_cout::ControlBuffer controlBuffer{device, 256};
	UsbDevice_cout::BulkEndpoint endpoint1{device, 1};
	UsbDevice_cout::BulkEndpoint endpoint2{device, 2};
	UsbDevice_cout::BulkBuffer buffer1{endpoint1, 129};
	UsbDevice_cout::BulkBuffer buffer2{endpoint2, 129};
};
