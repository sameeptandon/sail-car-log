#ifndef USB_CAN_H_
#define USB_CAN_H_

#include <inttypes.h>
#include <string>

#define    USB_DLE        0x10
#define    USB_STX        0x02
#define    USB_ETX        0x03

#define    SET_BAUD_1MEG  0x30
#define    SET_BAUD_500K  0x31
#define    SET_BAUD_250K  0x32
#define    SET_BAUD_125K  0x33

#define    USB_TIMEOUT    0.25

#define    USBCAN_BUFFER_SIZE    10000
#define    USBCAN_READ_TIMEOUT   0.1

namespace vlr {

class UsbCan {

public:
  UsbCan();
  virtual ~UsbCan();
  void open(const std::string& device);
  void close();

  bool readMessage(unsigned int* can_id, uint8_t* candata, int* can_length);
  void sendMessage(unsigned int can_id, uint8_t* candata, int can_length);
  void configCommand(uint8_t code);

private:
  void packageAndSend(const uint8_t* data, int data_size);
  bool validMessage(uint8_t* buffer, int buf_size, uint8_t* decoded, int* decoded_length);

private:
  int fd_;
  int buf_size_;
  uint8_t buffer_[USBCAN_BUFFER_SIZE];
  uint8_t decoded_[USBCAN_BUFFER_SIZE];
};

} // namespace vlr

#endif
