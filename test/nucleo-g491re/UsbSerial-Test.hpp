#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/UsbDevice_USB.hpp>
#include <coco/board/config.hpp>


constexpr int CONTROL_BUFFER_SIZE = 256;

// endpoint addresses for virutal COM port
constexpr int COMM_IN = 0x82;
constexpr int DATA_OUT = 0x01;
constexpr int DATA_IN = 0x81;


using namespace coco;

// drivers for UsbSerialTest, needs X-NUCLEO-SNK1M1 expansion board
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    // USB (uses PA11 and PA12)
    using Usb = UsbDevice_USB;
    Usb usb{loop};
    Usb::ControlBuffer<CONTROL_BUFFER_SIZE> controlBuffer{usb};
    Usb::Endpoint commEndpoint{usb, COMM_IN & 0x7f, 0, usbd::EndpointType::INTERRUPT};
    Usb::Endpoint dataEndpoint{usb, DATA_IN & 0x7f, DATA_OUT};
    Usb::Buffer<64> commBuffer{commEndpoint};
    Usb::Buffer<128> dataBuffer{dataEndpoint};
};

Drivers drivers;

extern "C" {
void USB_LP_IRQHandler() {
    drivers.usb.USB_IRQHandler();
}
}
