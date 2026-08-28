# Generate a test for a board
# TEST the test application, implemented in ${TEST}.cpp
# BOARD_LIB a library for a board containing SystemInit() and a linker script for embedded platforms
function(board_test TEST BOARD_LIB)
    # check if board library exists for the current platform
    if(TARGET ${BOARD_LIB})
        string(REGEX REPLACE ".*\\:" "" BOARD ${BOARD_LIB})
        set(NAME "${TEST}-${BOARD}")
        message("*** Test ${TEST} on board ${BOARD}")

        add_executable(${NAME}
            ${TEST}.cpp
        )
        target_include_directories(${NAME}
            PRIVATE
                ../
                ${BOARD}
        )
        target_link_libraries(${NAME}
            ${BOARD_LIB}
            ${PROJECT_NAME}
        )

        # generate hex file for flashing the target
        if(${CMAKE_CROSSCOMPILING})
            #message("*** Generate Hex for ${NAME} using ${CMAKE_OBJCOPY}")
            add_custom_command(TARGET ${NAME}
                POST_BUILD
                COMMAND ${CMAKE_OBJCOPY} -O ihex ${NAME} ${NAME}.hex
            )
        endif()
    endif()
endfunction()


board_test(Usb-Test coco-devboards::native)
board_test(Usb-Test coco-devboards::nrf52dongle)
board_test(Usb-Test coco-devboards::canable02)
board_test(Usb-Test coco-devboards::nucleo-g0b1re) # with X-NUCLEO-SNK1M1 expansion board
board_test(Usb-Test coco-devboards::nucleo-g431rb) # with X-NUCLEO-SNK1M1 expansion board
board_test(Usb-Test coco-devboards::nucleo-g474re) # with X-NUCLEO-SNK1M1 expansion board
board_test(Usb-Test coco-devboards::nucleo-g491re) # with X-NUCLEO-SNK1M1 expansion board
board_test(Usb-Test coco-devboards::nucleo-h503rb)
board_test(Usb-Test coco-devboards::nucleo-u385rg-q)
board_test(Usb-TestHost coco-devboards::native)

board_test(UsbSerial-Test coco-devboards::nrf52dongle)
board_test(UsbSerial-Test coco-devboards::nucleo-g0b1re) # with X-NUCLEO-SNK1M1 expansion board
board_test(UsbSerial-Test coco-devboards::nucleo-g431rb) # with X-NUCLEO-SNK1M1 expansion board
board_test(UsbSerial-Test coco-devboards::nucleo-g474re) # with X-NUCLEO-SNK1M1 expansion board
board_test(UsbSerial-Test coco-devboards::nucleo-g491re) # with X-NUCLEO-SNK1M1 expansion board
board_test(UsbSerial-Test coco-devboards::nucleo-h503rb)
board_test(UsbSerial-Test coco-devboards::nucleo-u385rg-q)

board_test(Dfu-TestHost coco-devboards::native)
board_test(Dfu-Test coco-devboards::nucleo-g0b1re) # with X-NUCLEO-SNK1M1 expansion board
board_test(Dfu-Test coco-devboards::nucleo-g431rb) # with X-NUCLEO-SNK1M1 expansion board
board_test(Dfu-Test coco-devboards::nucleo-g474re) # with X-NUCLEO-SNK1M1 expansion board
board_test(Dfu-Test coco-devboards::nucleo-g491re) # with X-NUCLEO-SNK1M1 expansion board

board_test(UsbMonitor-Test coco-devboards::native)
