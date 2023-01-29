#include "UsbHost_Win32.hpp"
#include <coco/convert.hpp>
#include <coco/loop.hpp>
#include <iostream>
#define NOMINMAX
//#include <Windows.h>
#include <SetupAPI.h>
#include <initguid.h>
#include <usbiodef.h>
#include <winusb.h>
#undef IN
#undef OUT
#undef interface


namespace coco {

UsbHost_Win32::UsbHost_Win32(Loop_Win32 &loop)
	: loop(loop)
{	
	// regularly scan for devices
	this->time = loop.now();
	loop.timeHandlers.add(*this);
}

UsbHost_Win32::~UsbHost_Win32() {
}
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

void UsbHost_Win32::handle() {
	this->time = this->loop.now() + 1s;
	
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
		if (p.second) {
			// found a new device, path has the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}

			// try to open the device
			auto file = CreateFileA(p.first->first.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
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
				if (device.state == Device::State::DISCONNECTED && device.filter(deviceDescriptor)) {
					// add file to completion port
					CompletionHandler *handler = &device;
					CreateIoCompletionPort(
						file,
						this->loop.port,
						(ULONG_PTR)handler,
						0);
					
					// transfer ownership of file to device
					device.it = p.first;
					p.first->second = &device;
					device.file = file;
					device.interface = interface;
					file = INVALID_HANDLE_VALUE;

					// flag the device to indicate that it is present
					device.flag = true;

					// change state
					device.state = Device::State::CONNECTED;
					
					// resume all coroutines waiting for connected state
					device.stateWaitlist.resumeAll([](Device::State state) {
						return state == Device::State::CONNECTED;
					});
					break;
				}
			}

			if (file != INVALID_HANDLE_VALUE) {
				WinUsb_Free(interface);
				CloseHandle(file);
			}
		} else {
			// flag the device to indicate that it is still present
			if (p.first->second != nullptr)
				p.first->second->flag = true;
		}
	}

	// detect devices that have disappeared
	for (Device &device : this->devices) {
		if (device.isConnected() && !device.flag)
			device.disconnect();
		device.flag = false;
	}
}


// Device

UsbHost_Win32::Device::Device(UsbHost_Win32 &host, std::function<bool (const usb::DeviceDescriptor &)> filter)
	: host(host), filter(filter)
{
	host.devices.add(*this);
}

UsbHost_Win32::Device::~Device() {
	WinUsb_Free(this->interface);
	CloseHandle(this->file);
}

UsbHostDevice::State UsbHost_Win32::Device::getState() {
	return this->state;
}

Awaitable<UsbHostDevice::State> UsbHost_Win32::Device::targetState(Device::State state) {
	if (this->state == state)
		return {};
	return {this->stateWaitlist, state};
}

static void cancelControlTransfer(UsbHostDevice::ControlParameters &p) {
	auto &device = *reinterpret_cast<UsbHost_Win32::Device *>(p.context);

	// cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
	auto result = CancelIoEx(device.file, &device.controlOverlapped);
	if (!result) {
		auto e = GetLastError();
		std::cout << "cancel error " << e << std::endl;
	}
	p.remove();
}

Awaitable<UsbHostDevice::ControlParameters> UsbHost_Win32::Device::controlTransfer(usb::Setup const &setup,
	void *data, int size)
{
	assert(size < std::size(this->controlBuffer));
	if (!isConnected())
		return {};
	if (!this->transferring) 
		startTransfer(setup, data, size);
	return {this->controlWaitlist, setup, data, size, this, cancelControlTransfer};
}
/*
void UsbHost_Win32::Device::getDescriptor(usb::DescriptorType type, void *data, int &size) {
	int index = 0;
	int languageId = 0;
  	ULONG transferred;
	bool result = WinUsb_GetDescriptor(this->interface, int(type), index, languageId, (UCHAR*)data, size, &transferred);
	size = result ? transferred : 0;
}*/

void UsbHost_Win32::Device::startTransfer(const usb::Setup &setup, void *data, int size) {
	/*WINUSB_SETUP_PACKET packet;
	packet.RequestType = UCHAR(setup.requestType);
	packet.Request = setup.request;
	packet.Value = setup.value;
	packet.Index = setup.index;
	packet.Length = setup.length;*/

	if ((setup.requestType & usb::RequestType::DIRECTION_MASK) == usb::RequestType::OUT)
		memcpy(this->controlBuffer, data, size);
	memset(&this->controlOverlapped, 0, sizeof(OVERLAPPED));
	auto result = WinUsb_ControlTransfer(this->interface, (const WINUSB_SETUP_PACKET &)setup,
		this->controlBuffer, size, nullptr, &this->controlOverlapped);
	this->transferring = true;
}

void UsbHost_Win32::Device::handle(OVERLAPPED *overlapped) {
	// check if control transfer finished
	if (this->transferring && overlapped == &this->controlOverlapped) {
		DWORD transferred;
		auto result = GetOverlappedResult(this->file, &this->controlOverlapped, &transferred, false);
		if (!result) {
			auto error = GetLastError();
			if (error == ERROR_OPERATION_ABORTED) {
				// cancelled
				this->transferring = false;

				// start next control transfer operation
				this->controlWaitlist.visitFirst([this](ControlParameters &p) {
					startTransfer(p.setup, p.data, p.size);
				});
			} else if (error != ERROR_IO_INCOMPLETE) {
				// "real" error: return zero length buffer
				transferred = 0;
				result = true;

				// check if the USB device was disconnected
				if (error == ERROR_GEN_FAILURE)
					this->disconnect();
			}
		}

		if (result) {
			// resume waiting coroutine			
			this->controlWaitlist.resumeFirst([this, transferred](ControlParameters &p) {
				// copy buffer
				if ((p.setup.requestType & usb::RequestType::DIRECTION_MASK) == usb::RequestType::IN)
					memcpy(p.data, this->controlBuffer, transferred);
					//std::copy(this->controlBuffer, this->controlBuffer + transferred, reinterpret_cast<uint8_t *>(p.data));

				//*p.size = transferred;
				return true;
			});

			this->transferring = false;

			// start next control transfer operation
			this->controlWaitlist.visitFirst([this](ControlParameters &p) {
				startTransfer(p.setup, p.data, p.size);
			});
		}
	}

	for (auto &endpoint : this->bulkEndpoints) {
		endpoint.handle(overlapped);
	}
}

void UsbHost_Win32::Device::disconnect() {
	if (this->state == Device::State::CONNECTED) {
		this->flag = false;
		WinUsb_Free(this->interface);
		CloseHandle(this->file);
		this->interface = nullptr;
		this->file = INVALID_HANDLE_VALUE;	
		this->host.deviceInfos.erase(this->it);

		// change state
		this->state = Device::State::DISCONNECTED;

		// resume all coroutines waiting for transfers
		this->controlWaitlist.resumeAll([](ControlParameters &p) {
			return true;
		});
		this->transferring = false;
		for (auto &endpoint : this->bulkEndpoints) {
			endpoint.readWaitlist.resumeAll([](BulkEndpoint::ReadParameters &p) {
				*p.size = 0;
				return true;
			});
			endpoint.reading = false;
			endpoint.writeWaitlist.resumeAll([](BulkEndpoint::WriteParameters &p) {
				return true;
			});
			endpoint.writing = false;
		}

		// resume all coroutines waiting for disconnected state
		this->stateWaitlist.resumeAll([](Device::State state) {
			return state == Device::State::DISCONNECTED;
		});
	}
}


// BulkEndpoint

UsbHost_Win32::BulkEndpoint::BulkEndpoint(Device &device, int inAddress, int outAddress, bool packet)
	: device(device), inAddress(inAddress), outAddress(outAddress), readPacket(packet), writePacket(packet)
{
	device.bulkEndpoints.add(*this);
}

UsbHost_Win32::BulkEndpoint::~BulkEndpoint() {
}

static void cancelRead(Stream::ReadParameters &p) {
	auto &endpoint = *reinterpret_cast<UsbHost_Win32::BulkEndpoint *>(p.context);

	// cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
	auto result = CancelIoEx(endpoint.device.file, &endpoint.readOverlapped);
	if (!result) {
		auto e = GetLastError();
		std::cout << "cancel error " << e << std::endl;
	}
	p.remove();
}

Awaitable<Stream::ReadParameters> UsbHost_Win32::BulkEndpoint::read(void *data, int &size/*, bool packet*/) {
	if (!this->device.isConnected())
		return {};
	if (!this->reading)
		startRead(data, size/*, packet*/);
	return {this->readWaitlist, data, &size, /*packet,*/ this, cancelRead};
}

static void cancelWrite(Stream::WriteParameters &p) {
	auto &endpoint = *reinterpret_cast<UsbHost_Win32::BulkEndpoint *>(p.context);

	// cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
	auto result = CancelIoEx(endpoint.device.file, &endpoint.writeOverlapped);
	if (!result) {
		auto e = GetLastError();
		std::cout << "cancel error " << e << std::endl;
	}
	p.remove();
}

Awaitable<Stream::WriteParameters> UsbHost_Win32::BulkEndpoint::write(void const *data, int size/*, bool packet*/) {
	if (!this->device.isConnected())
		return {};
	if (!this->writing)
		startWrite(data, size/*, packet*/);
	return {this->writeWaitlist, data, size/*, packet*/, this, cancelWrite};
}

void UsbHost_Win32::BulkEndpoint::startRead(void *data, int size/*, bool packet*/) {
	//this->readPacket = packet;
	int toRead = size < std::size(this->readBuffer) && !this->readPacket ? size : std::size(this->readBuffer);
	memset(&this->readOverlapped, 0, sizeof(OVERLAPPED));
	auto result = WinUsb_ReadPipe(this->device.interface, this->inAddress, this->readBuffer, toRead, nullptr, &this->readOverlapped);
	this->reading = true;
	this->readData = reinterpret_cast<uint8_t *>(data);
	this->readSize = size;
}

void UsbHost_Win32::BulkEndpoint::startWrite(const void *data, int size/*, bool packet*/) {
	int toWrite = std::min(size, int(std::size(this->writeBuffer)));
	memcpy(this->writeBuffer, data, toWrite);
	memset(&this->writeOverlapped, 0, sizeof(OVERLAPPED));
	auto result = WinUsb_WritePipe(this->device.interface, this->outAddress, this->writeBuffer, toWrite, nullptr, &this->writeOverlapped);
	this->writing = true;
	this->writeData = reinterpret_cast<const uint8_t *>(data);
	this->writeSize = size;
	//this->writePacket = packet;
}

void UsbHost_Win32::BulkEndpoint::handle(OVERLAPPED *overlapped) {
	// check if read finished
	if (this->reading && overlapped == &this->readOverlapped) {
		DWORD transferred;
		auto result = GetOverlappedResult(this->device.file, &this->readOverlapped, &transferred, false);
		if (!result) {
			auto error = GetLastError();
			if (error == ERROR_OPERATION_ABORTED) {
				// cancelled
				this->reading = false;

				// start next read operation
				this->readWaitlist.visitFirst([this](ReadParameters &p) {
					startRead(p.data, *p.size);//, p.packet);
				});
			} else if (error != ERROR_IO_INCOMPLETE) {
				// "real" error: return zero length buffer
				transferred = 0;
				result = true;

				// check if the USB device was disconnected
				if (error == ERROR_GEN_FAILURE)
					this->device.disconnect();
			}
		}

		if (result) {
			// copy data into read buffer
			auto readData = this->readData;
			int readSize = this->readSize;
			int t = std::min(readSize, int(transferred));
			memcpy(readData, this->readBuffer, t);
			//std::copy(this->readBuffer, this->readBuffer + t, readData);
			readData += t;
			readSize -= t;
			if ((this->readPacket || readSize > 0) && transferred >= std::size(this->readBuffer)) {
				// more to read
				startRead(readData, readSize);//, this->readPacket);
			} else {
				// resume waiting coroutine			
				this->readWaitlist.resumeFirst([readData](ReadParameters &p) {
					*p.size = readData - reinterpret_cast<uint8_t *>(p.data);
					return true;
				});

				this->reading = false;

				// start next read operation
				this->readWaitlist.visitFirst([this](ReadParameters &p) {
					startRead(p.data, *p.size);//, p.packet);
				});
			}
		}
	}

	// check if write finished
	if (this->writing && overlapped == &this->writeOverlapped) {
		DWORD transferred;
		auto result = GetOverlappedResult(this->device.file, &this->writeOverlapped, &transferred, false);
		if (!result) {
			auto error = GetLastError();
			if (error == ERROR_OPERATION_ABORTED) {
				// cancelled
				this->writing = false;

				// start next write operation
				this->writeWaitlist.visitFirst([this](WriteParameters &p) {
					startWrite(p.data, p.size);//, p.packet);
				});
			} else if (error != ERROR_IO_INCOMPLETE) {
				// "real" error: return zero length buffer
				transferred = 0;
				result = true;

				// check if the USB device was disconnected
				if (error == ERROR_GEN_FAILURE)
					this->device.disconnect();
			}
		}

		if (result) {
			auto writeData = this->writeData + transferred;
			int writeSize = this->writeSize - transferred;
			if (writeSize > 0 || (this->writePacket && transferred > 0 && (transferred & 63) == 0)) { //todo: get packet size from endpoint descriptor
				// more to write
				startWrite(writeData, writeSize);//, this->writePacket);
			} else {
				// resume waiting coroutine			
				this->writeWaitlist.resumeFirst([](WriteParameters &p) {
					return true;
				});		

				this->writing = false;
			
				// start next write operation
				this->writeWaitlist.visitFirst([this](WriteParameters &p) {
					startWrite(p.data, p.size);//, p.packet);
				});
			}
		}
	}
}

} // namespace coco
