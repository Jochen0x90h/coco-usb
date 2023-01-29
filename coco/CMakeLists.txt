add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
	PUBLIC FILE_SET headers TYPE HEADERS FILES
		Stream.hpp
		usb.hpp
		UsbDevice.hpp
		UsbHostDevice.hpp
	PRIVATE
		Stream.cpp
		UsbDevice.cpp
		UsbHostDevice.cpp
)

if(${PLATFORM} STREQUAL "native")
	# native platform (Windows, MacOS, Linux)
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
			native/coco/platform/UsbDevice_cout.hpp
			native/coco/platform/UsbHost_native.hpp
		PRIVATE
			native/coco/platform/UsbDevice_cout.cpp
	)
	if(${OS} STREQUAL "Windows")
		target_sources(${PROJECT_NAME}
			PUBLIC FILE_SET platform_headers FILES
				native/coco/platform/UsbHost_Win32.hpp
			PRIVATE
				native/coco/platform/UsbHost_Win32.cpp
		)	
	endif()
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
	coco-loop::coco-loop
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
