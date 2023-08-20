#include "UsbHost_WinUSB.hpp" // includes Windows.h and winusb.h
#include <coco/convert.hpp>
#include <iostream>
#include <SetupAPI.h>
#include <initguid.h>
#include <usbiodef.h>


namespace coco {

UsbHost_WinUSB::UsbHost_WinUSB(Loop_Win32 &loop)
	: loop(loop), callback(makeCallback<UsbHost_WinUSB, &UsbHost_WinUSB::handle>(this))
{
	// regularly scan for usb devices
	loop.invoke(this->callback);
}

//UsbHost_WinUSB::~UsbHost_WinUSB() {
//}
/*
static int parseHex(const char *s) {
	int value = 0;
	for (int i = 0; i < 4; ++i) {
		value <<= 4;

		char ch = s[i];
		if (ch >= '0' && ch <= '9')
			value += ch - '0';
		else if (ch >= 'A' && ch <= 'F')
			value += ch - 'A' + 10;
		else if (ch >= 'a' && ch <= 'f')
			value += ch - 'a' + 10;
	}
	return value;
}*/

void UsbHost_WinUSB::handle() {
	//this->time = this->loop.now() + 1s;
	loop.invoke(this->callback, 1s);

	// enumerate devices
	HDEVINFO deviceInfo = SetupDiGetClassDevsA(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	int index = 0;
	SP_DEVINFO_DATA deviceData;
	deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
	while (SetupDiEnumDeviceInfo(deviceInfo, index, &deviceData)) {
		++index;

		// get interface data
		SP_DEVICE_INTERFACE_DATA interfaceData;
		interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
		if (!SetupDiEnumDeviceInterfaces(deviceInfo, &deviceData, &GUID_DEVINTERFACE_USB_DEVICE, 0, &interfaceData)) {
			continue;
		}

		// get device path
		union {
			SP_DEVICE_INTERFACE_DETAIL_DATA_A devicePath;
			uint8_t space[2 + 128];
		} u;
		u.devicePath.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
		if (!SetupDiGetDeviceInterfaceDetailA(deviceInfo, &interfaceData, &u.devicePath, sizeof(u), nullptr, nullptr)) {
			// error
			continue;
		}

		// check if device is new
		auto p = this->deviceInfos.insert(std::map<std::string, Device *>::value_type{u.devicePath.DevicePath, nullptr});
		auto it = p.first;
		if (p.second) {
			// found a new device, path has the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}

			// try to open the device
			auto file = CreateFileA(it->first.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
			if (file == INVALID_HANDLE_VALUE)
				continue;

			// get USB interface
			WINUSB_INTERFACE_HANDLE interface;
			if (!WinUsb_Initialize(file, &interface)) {
				CloseHandle(file);
				continue;
			}

			// read device descriptor
			usb::DeviceDescriptor deviceDescriptor;
		  	ULONG transferred;
			bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::DEVICE), 0, 0, (UCHAR*)&deviceDescriptor, sizeof(deviceDescriptor), &transferred);
			if (!result || transferred < sizeof(deviceDescriptor)) {
				WinUsb_Free(interface);
				CloseHandle(file);
				continue;
			}

			// now check if a filter of a device accepts the device descriptor
			for (Device &device : this->devices) {
				if (device.stat == Device::State::DISABLED && device.filter(deviceDescriptor)) {
					// add file to completion port of event loop
					Loop_Win32::CompletionHandler *handler = &device;
					CreateIoCompletionPort(
						file,
						this->loop.port,
						ULONG_PTR(handler),
						0);

					// set device to map and store the iterator
					it->second = &device;
					device.it = it;

					// connect and transfer ownership of file to device
					device.connect(file, interface);
					file = INVALID_HANDLE_VALUE;

					// set iterator and flag the device to indicate that it is present
					device.flag = true;
					break;
				}
			}

			if (file != INVALID_HANDLE_VALUE) {
				WinUsb_Free(interface);
				CloseHandle(file);
			}
		} else {
			// flag the device to indicate that it is still present
			if (it->second != nullptr)
				it->second->flag = true;
		}
	}

	// detect devices that have disappeared
	for (Device &device : this->devices) {
		if (device.ready() && !device.flag)
			device.disconnect();
		device.flag = false;
	}
}


// Device

UsbHost_WinUSB::Device::Device(UsbHost_WinUSB &host, std::function<bool (const usb::DeviceDescriptor &)> filter)
	: host(host), filter(filter)
{
	host.devices.add(*this);
}

UsbHost_WinUSB::Device::~Device() {
	WinUsb_Free(this->interface);
	CloseHandle(this->file);
}

Device::State UsbHost_WinUSB::Device::state() {
	return this->stat;
}

Awaitable<> UsbHost_WinUSB::Device::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(this->stat))) == 0)
		return {};
	return {this->stateTasks};
}

/*
void UsbHost_WinUSB::Device::getDescriptor(usb::DescriptorType type, void *data, int &size) {
	int index = 0;
	int languageId = 0;
  	ULONG transferred;
	bool result = WinUsb_GetDescriptor(this->interface, int(type), index, languageId, (UCHAR*)data, size, &transferred);
	size = result ? transferred : 0;
}*/

static bool getIsoPipeInfo(void *interface, int endpointCount, uint8_t endpointAddress, WINUSB_PIPE_INFORMATION_EX &pipeInfo) {
	for (int i = 0; i < endpointCount; ++i) {
		auto result = WinUsb_QueryPipeEx(interface, 0, i, &pipeInfo);

		if (result && pipeInfo.PipeType == UsbdPipeTypeIsochronous && pipeInfo.PipeId == endpointAddress)
			return true;
	}
	return false;
}

void UsbHost_WinUSB::Device::connect(HANDLE file, void *interface) {
	this->file = file;
	this->interface = interface;


	USB_INTERFACE_DESCRIPTOR interfaceDescriptor;
	auto result = WinUsb_QueryInterfaceSettings(interface, 0, &interfaceDescriptor);
/*
	// allocate buffers of isochronous endpoints
	for (auto &endpoint : this->isoEndpoints) {
		WINUSB_PIPE_INFORMATION_EX pipeInfo;
		if ((endpoint.inAddress & 0x7f) != 0) {
			if (getIsoPipeInfo(this->interface, interfaceDescriptor.bNumEndpoints, endpoint.inAddress, pipeInfo)) {
				int packetSize = pipeInfo.MaximumBytesPerInterval;
				int packetCount = (endpoint.duration * (8 / pipeInfo.Interval) + 1) / 3;
				int bufferSize = packetCount * packetSize;

				uint8_t *buffer = new uint8_t[bufferSize * 3];

				WINUSB_ISOCH_BUFFER_HANDLE bufferHandle;
				result = WinUsb_RegisterIsochBuffer(
					interface,
					endpoint.inAddress,
					buffer,
					bufferSize * 3,
					&bufferHandle);

				endpoint.inPacketSize = packetSize;
				endpoint.inPacketCount = packetCount;
				endpoint.inBufferSize = bufferSize;
				endpoint.inBuffer = buffer;
				endpoint.inBufferHandle = bufferHandle;
				endpoint.inPacketDescriptors = new USBD_ISO_PACKET_DESCRIPTOR[packetCount * 3];

			}
		}
	}
*/
	// set state
	this->stat = Device::State::READY;

	// enable control buffers
	for (auto &buffer : this->controlBuffers) {
		buffer.setReady(0);
	}

	// enable bulk buffers
	for (auto &endpoint : this->bulkEndpoints) {
		for (auto &buffer : endpoint.buffers) {
			buffer.setReady(0);
		}
	}

	// resume all coroutines waiting for ready state
	this->stateTasks.doAll();
}

void UsbHost_WinUSB::Device::disconnect() {
	if (this->stat == Device::State::READY) {
		this->flag = false;
		WinUsb_Free(this->interface);
		CloseHandle(this->file);
		this->interface = nullptr;
		this->file = INVALID_HANDLE_VALUE;

		// erase from device map of host
		this->host.deviceInfos.erase(this->it);

		// set state
		this->stat = Device::State::DISABLED;

		// set state of buffers to disabled
		for (auto &buffer : this->controlBuffers) {
			buffer.setDisabled();
		}
		for (auto &endpoint : this->bulkEndpoints) {
			for (auto &buffer : endpoint.buffers) {
				buffer.setDisabled();
			}
		}

		// resume all coroutines waiting for disabled state
		this->stateTasks.doAll();
	}
}

void UsbHost_WinUSB::Device::handle(OVERLAPPED *overlapped) {
	for (auto &buffer : this->controlBuffers) {
		if (overlapped == &buffer.overlapped) {
			buffer.handle(overlapped);
			return;
		}
	}
	for (auto &endpoint : this->bulkEndpoints) {
		for (auto &buffer : endpoint.buffers) {
			if (overlapped == &buffer.overlapped[buffer.index]) {
				buffer.handle(overlapped);
				return;
			}
		}
	}
}


// ControlBuffer

UsbHost_WinUSB::ControlBuffer::ControlBuffer(Device &device, int size)
	: BufferImpl(new uint8_t[size], size, device.stat)
	, device(device)
{
	device.controlBuffers.add(*this);
}

UsbHost_WinUSB::ControlBuffer::~ControlBuffer() {
	delete [] this->dat;
}

bool UsbHost_WinUSB::ControlBuffer::setHeader(const uint8_t *data, int size) {
	if (size != sizeof(WINUSB_SETUP_PACKET)) {
		assert(false);
		return false;
	}
	this->setup = *reinterpret_cast<const WINUSB_SETUP_PACKET *>(data);
	return true;
}

bool UsbHost_WinUSB::ControlBuffer::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(this->stat != State::BUSY);
		return false;
	}

	// start the transfer
	memset(&this->overlapped, 0, sizeof(OVERLAPPED));
	auto result = WinUsb_ControlTransfer(this->device.interface, this->setup, this->dat, size, nullptr,
		&this->overlapped);

	// set state
	setBusy();

	return true;
}

void UsbHost_WinUSB::ControlBuffer::cancel() {
	if (this->stat != State::BUSY)
		return;

	// cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
	auto result = CancelIoEx(this->device.file, &this->overlapped);
	if (!result) {
		auto e = GetLastError();
		std::cerr << "cancel error " << e << std::endl;
	}
}

void UsbHost_WinUSB::ControlBuffer::handle(OVERLAPPED *overlapped) {
	DWORD transferred;
	auto result = GetOverlappedResult(this->device.file, overlapped, &transferred, false);
	if (!result) {
		auto error = GetLastError();
		//if (error != ERROR_IO_INCOMPLETE) {
			// "real" error or cancelled (ERROR_OPERATION_ABORTED): return zero
			transferred = 0;
			result = true;

			// check if the USB device was disconnected
			if (error == ERROR_GEN_FAILURE)
				this->device.disconnect();
		//}
	}
	if (result) {
		// transfer finished
		setReady(transferred);
	}
}


// BulkBufferBase

UsbHost_WinUSB::BulkBuffer::BulkBuffer(BulkEndpoint &endpoint, int size)
	: BufferImpl(new uint8_t[size], size, endpoint.device.stat)
	, endpoint(endpoint)
{
	endpoint.buffers.add(*this);
}

UsbHost_WinUSB::BulkBuffer::~BulkBuffer() {
	delete [] this->dat;
}

bool UsbHost_WinUSB::BulkBuffer::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(this->stat != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	memset(&this->overlapped[0], 0, sizeof(OVERLAPPED));
	int index = 0;
	auto data = this->dat;
	if ((op & Op::WRITE) == 0) {
		// read
		auto result = WinUsb_ReadPipe(this->endpoint.device.interface, this->endpoint.inAddress, data, size,
			nullptr, &this->overlapped[0]);
	} else {
		// write

		// check if we need to send a zero packet at the end
		bool zero = (op & Op::PARTIAL) == 0 && size > 0 && (size & 63) == 0;
		if (zero) {
			memset(&this->overlapped[1], 0, sizeof(OVERLAPPED));
			index = 1;
		}

		auto result = WinUsb_WritePipe(this->endpoint.device.interface, this->endpoint.outAddress, data, size,
			nullptr, &this->overlapped[index]);

		// check if we need to send a zero packet at the end
		if (zero) {
			result = WinUsb_WritePipe(this->endpoint.device.interface, this->endpoint.outAddress, data + size, 0,
				nullptr, &this->overlapped[0]);
		}
	}
	this->index = index;
	this->xferred = 0;

	// set state
	setBusy();

	return true;
}

void UsbHost_WinUSB::BulkBuffer::cancel() {
	if (this->stat != State::BUSY)
		return;

	// cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
	for (int i = this->index; i >= 0; --i) {
		auto result = CancelIoEx(this->endpoint.device.file, &this->overlapped[i]);
		if (!result) {
			auto e = GetLastError();
			std::cerr << "cancel error " << e << std::endl;
		}
	}
}

void UsbHost_WinUSB::BulkBuffer::handle(OVERLAPPED *overlapped) {
	DWORD transferred;
	auto result = GetOverlappedResult(this->endpoint.device.file, overlapped, &transferred, false);
	if (!result) {
		auto error = GetLastError();
		//if (error != ERROR_IO_INCOMPLETE) {
			// "real" error or cancelled (ERROR_OPERATION_ABORTED): return zero size
			transferred = 0;
			this->xferred = 0;
			result = true;

			// check if the USB device was disconnected
			if (error == ERROR_GEN_FAILURE)
				this->endpoint.device.disconnect();
		//}
	}
	if (result) {
		// transfer finished
		if (this->index > 0) {
			// need to wait for completion of the zero packet
			this->index = 0;
			this->xferred = transferred;
		} else {
			setReady(transferred + this->xferred);
		}
	}
}


// BulkEndpoint

UsbHost_WinUSB::BulkEndpoint::BulkEndpoint(UsbHost_WinUSB::Device &device, int inAddress, int outAddress)
	: device(device), inAddress(inAddress), outAddress(outAddress)
{
	device.bulkEndpoints.add(*this);
}

UsbHost_WinUSB::BulkEndpoint::~BulkEndpoint() {
}

Device::State UsbHost_WinUSB::BulkEndpoint::state() {
	return this->device.stat;
}

Awaitable<> UsbHost_WinUSB::BulkEndpoint::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(this->device.stat))) == 0)
		return {};
	return {this->device.stateTasks};
}

int UsbHost_WinUSB::BulkEndpoint::getBufferCount() {
	return this->buffers.count();
}

UsbHost_WinUSB::BulkBuffer &UsbHost_WinUSB::BulkEndpoint::getBuffer(int index) {
	return this->buffers.get(index);
}

} // namespace coco
