#pragma once

#include <coco/usb.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbHost_native.hpp>
#include <iostream>


using namespace coco;

// drivers for UsbTestHost
struct Drivers {
    Loop_native loop;

    using Usb = UsbHost_native;
    Usb host{loop};
    Usb::Device device{host, [](const usb::DeviceDescriptor &deviceDescriptor) {
        std::cout << std::hex << deviceDescriptor.idVendor << std::endl;
        bool found = deviceDescriptor.idVendor == 0x0483 && deviceDescriptor.idProduct == 0x5740;
        if (found) {
            int x = 0;
        }
        return deviceDescriptor.idVendor == 0x0483 && deviceDescriptor.idProduct == 0x5740;
    }};
    Usb::ControlBuffer controlBuffer{32, device};
    /*Usb::Endpoint endpoint1{device, 1};
    Usb::Endpoint endpoint2{device, 2};
    Usb::Buffer buffer1{129, endpoint1};
    Usb::Buffer buffer2{129, endpoint2};*/
};
