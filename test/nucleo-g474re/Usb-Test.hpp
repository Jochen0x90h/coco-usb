#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/UsbDevice_USB.hpp>
#include <coco/board/config.hpp>


using namespace coco;

constexpr int CONTROL_BUFFER_SIZE = 256;

// drivers for UsbTest, needs X-NUCLEO-SNK1M1 expansion board
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Usb = UsbDevice_USB;
    Usb usb{loop};
    Usb::ControlBuffer<CONTROL_BUFFER_SIZE> controlBuffer{usb};
    Usb::Endpoint endpoint1{usb, 1, usbd::EndpointType::INTERRUPT};
    Usb::Endpoint endpoint2{usb, 2};
    Usb::Buffer<129> buffer1{endpoint1};
    Usb::Buffer<129> buffer2{endpoint2};

    Drivers() {
        // re-enable dead battery feature for USB-C support (not needed because of X-NUCLEO-SNK1M1 expansion board)
        //PWR->CR3 = PWR->CR3 & ~PWR_CR3_UCPD_DBDIS;
    }
};

Drivers drivers;

extern "C" {
void USB_LP_IRQHandler() {
    drivers.usb.USB_IRQHandler();
}
}
