#pragma once

#include "usb.hpp"
#include <coco/Device.hpp>
#include <coco/Buffer.hpp>


namespace coco {

/// @brief USb device as seen from the host
///
class UsbHostDevice : public Device {
public:
    UsbHostDevice(State state) : Device(state) {}

    /// @brief Open the device (transitions to OPENING if in DISABLED state).
    ///
    //virtual void open() = 0;
};

} // namespace coco
