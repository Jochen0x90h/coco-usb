#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/UsbDevice_USB.hpp>
#include <coco/platform/crc.hpp>
#include <coco/board/config.hpp>



using namespace coco;

constexpr int CONTROL_BUFFER_SIZE = 256;

// drivers for DfuTest, needs X-NUCLEO-SNK1M1 expansion board
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Usb = UsbDevice_USB;
    Usb usb{loop};
    Usb::ControlBuffer<CONTROL_BUFFER_SIZE> controlBuffer{usb};

    Drivers() {
        // init crc in 32 bit mode and default polynomial (0x04C11DB7)
        crc::enableClock();
    }
};

Drivers drivers;

extern "C" {
void USB_LP_IRQHandler() {
    drivers.usb.USB_IRQHandler();
}
}
