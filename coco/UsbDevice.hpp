#pragma once

#include "usb.hpp"
#include <coco/Device.hpp>
#include <coco/Buffer.hpp>


namespace coco {

/// @brief Interface for USB device support (device side, not host side)
///
/// https://www.beyondlogic.org/usbnutshell/usb1.shtml
class UsbDevice : public Device {
public:
    UsbDevice(State state) : Device(state) {}

    /// @brief Wait for a request to the device (e.g. control request for USB)
    /// @return use co_await on return value to wait until the device receives a request
    [[nodiscard]] Awaitable<Events> untilRequest() {
        //auto &st = getStateTasks();
        return {this->st.tasks, Events::REQUEST};
    }

    /// @brief Get the current setup packet. Call after waiting for a request (co_await device.untilRequest();)
    ///
    virtual usb::Setup getSetup() = 0;

    /// @brief Acknowledge a conrol request without data stage
    ///
    virtual void acknowledge() = 0;

    /// @brief Indicate that a control request is not supported or has invalid parameters
    ///
    virtual void stall() = 0;

    /// @brief Helper for control in transfers, e.g. sending a descriptor to the host
    ///
    template <typename T>
    [[nodiscard]] static Awaitable<Buffer::Events> controlIn(Buffer &buffer, usb::Setup const &setup, const T &data) {
        int size = std::min(int(setup.wLength), int(sizeof(data)));
        return buffer.writeData(&data, size);
    }
};

} // namespace coco
