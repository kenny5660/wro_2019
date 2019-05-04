#pragma once
#include  <memory>
#include "Uart.h"
#include "DIO.h"
namespace rp {
	namespace standalone {
		namespace rplidar {
			class ChannelDeviceMyRioSerial : public ChannelDevice
			{
			public:
				ChannelDeviceMyRioSerial(std::shared_ptr<Uart> uart, std::shared_ptr<MyRio_Dio> dtr);
				bool bind(const char*, uint32_t) override;
				bool open() override;
				void close()override;
				void flush()override;
				bool waitfordata(size_t data_count, _u32 timeout = -1, size_t * returned_size = NULL)override;
				int senddata(const _u8 * data, size_t size) override;
				int recvdata(unsigned char * data, size_t size) override;
				void setDTR() override;
				void clearDTR() override;
				void ReleaseRxTx() override;
			private:
				std::shared_ptr<Uart> uart_;
				std::shared_ptr<MyRio_Dio> dtr_;
			};

			
			class RpLidarDriverMyRioSerial : public RPlidarDriverImplCommon
			{
			public:
				RpLidarDriverMyRioSerial(std::shared_ptr<Uart> uart, std::shared_ptr<MyRio_Dio> dtr);
				virtual ~RpLidarDriverMyRioSerial()
				{}
				;
				virtual u_result connect(const char * port_path, _u32 baudrate, _u32 flag = 0);
				virtual void disconnect();
			};
		}
	}
}

