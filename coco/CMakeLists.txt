add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
	PUBLIC FILE_SET headers TYPE HEADERS FILES
		usb.hpp
		UsbDevice.hpp
	PRIVATE
		UsbDevice.cpp
)

if(${PLATFORM} STREQUAL "native")
	# native platform (Windows, MacOS, Linux)
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
			native/coco/platform/UsbDeviceOut.hpp
		PRIVATE
			native/coco/platform/UsbDeviceOut.cpp
	)
elseif(${PLATFORM} MATCHES "^nrf52")
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nrf52 FILES
			nrf52/coco/platform/UsbDevice_USBD.hpp
		PRIVATE
			nrf52/coco/platform/UsbDevice_USBD.cpp
	)
elseif(${PLATFORM} MATCHES "^stm32f0")

endif()

target_link_libraries(${PROJECT_NAME}
	coco::coco
	coco-loop::coco-loop # for loop::now() in native
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		..
)

# install the library
install(TARGETS ${PROJECT_NAME}
	FILE_SET headers DESTINATION include/coco
	FILE_SET platform_headers DESTINATION include
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
