# Generate a test for a board
# TEST the test application, implemented in ${TEST}.cpp
# BOARD_LIB a library for a board containing SystemInit() and a linker script for embedded platforms
function(board_test TEST BOARD_LIB)
	# check if board library exists for the current platform
	if(TARGET ${BOARD_LIB})
		string(REGEX REPLACE ".*\\:" "" BOARD ${BOARD_LIB})
		set(NAME "${TEST}-${BOARD}")
		message("*** Board: ${BOARD}")

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

		# windows specific libraries
		if(WIN32)
			target_link_libraries(${NAME} SetupAPI Winusb)
		endif()

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


board_test(UsbDeviceTest coco-devboards::native)
board_test(UsbDeviceTest coco-devboards::nrf52dongle)
board_test(UsbDeviceTest coco-devboards::stm32g431nucleo) # with X-NUCLEO-SNK1M1 expansion board
board_test(UsbDeviceTest coco-devboards::stm32g474nucleo) # with X-NUCLEO-SNK1M1 expansion board
board_test(UsbDeviceTest coco-devboards::canable02)
board_test(UsbDeviceTest limes2)

board_test(UsbTestHost coco-devboards::native)

board_test(UsbSerialTest coco-devboards::nrf52dongle)
board_test(UsbSerialTest coco-devboards::stm32g431nucleo) # with X-NUCLEO-SNK1M1 expansion board
