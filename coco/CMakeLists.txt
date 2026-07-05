set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PUBLIC FILE_SET headers TYPE HEADERS FILES
        usb.hpp
        UsbDevice.hpp
        UsbHostDevice.hpp
)

if(${PLATFORM} STREQUAL "native" OR ${PLATFORM} STREQUAL "emu")
    # native platform (Windows, MacOS, Linux)
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
            native/coco/platform/UsbDevice_cout.hpp
            native/coco/platform/UsbHost_native.hpp
        PRIVATE
            native/coco/platform/UsbDevice_cout.cpp
    )
    if(WIN32)
        # Win32/WinUSB
        target_sources(${PROJECT_NAME}
            PUBLIC FILE_SET platform_headers FILES
                native/coco/platform/UsbHost_WinUSB.hpp
                native/coco/platform/UsbMonitor_SetupAPI.hpp
            PRIVATE
                native/coco/platform/UsbHost_WinUSB.cpp
                native/coco/platform/UsbMonitor_SetupAPI.cpp
        )
    endif()
    if(LINUX)
        # io_uring/libudev
        target_sources(${PROJECT_NAME}
            PUBLIC FILE_SET platform_headers FILES
                native/coco/platform/UsbHost_io_uring.hpp
                native/coco/platform/UsbMonitor_udev.hpp
            PRIVATE
                native/coco/platform/UsbHost_io_uring.cpp
                native/coco/platform/UsbMonitor_udev.cpp
        )
    endif()
elseif(${PLATFORM} MATCHES "^nrf52")
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nrf52 FILES
            nrf52/coco/platform/UsbDevice_USBD.hpp
        PRIVATE
            nrf52/coco/platform/UsbDevice_USBD.cpp
    )
elseif(${PLATFORM} MATCHES "^stm32")
    target_sources(${PROJECT_NAME}
        PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/stm32 FILES
            stm32/coco/platform/UsbDevice_USB.hpp
        PRIVATE
            stm32/coco/platform/UsbDevice_USB.cpp
    )
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ..
)

target_link_libraries(${PROJECT_NAME}
    coco::coco
    coco-loop::coco-loop
    coco-device::coco-device
)
if(WIN32)
    target_link_libraries(${PROJECT_NAME} SetupAPI Winusb)
endif()
if(LINUX)
    target_link_libraries(${PROJECT_NAME} libudev::libudev)
endif()

# install the library
install(TARGETS ${PROJECT_NAME}
    FILE_SET headers DESTINATION include/coco
    FILE_SET platform_headers DESTINATION include
)
