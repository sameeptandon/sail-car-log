#include <usbcan.h>

UsbCan::UsbCan() {
  _dwCtrlHandle = ECI_INVALID_HANDLE;
}

UsbCan::~UsbCan() {

}

void UsbCan::open(const std::string& device) {
  _dwCtrlHandle = 0;

  cout << "Initializing CAN Device" << endl; 
  ECI_RESULT  hResult     = ECI_OK;
  ECI_HW_PARA stcHwPara   = {0};
  ECI_HW_INFO stcHwInfo   = {0};
  DWORD       dwHwIndex   = 0;
  DWORD       dwCtrlIndex = 0;

  //*** Prepare Hardware parameter structure
  stcHwPara.wHardwareClass = ECI_HW_USB;

  //*** At first call Initialize to prepare ECI driver
  //*** with one board
  hResult = ECIDRV_Initialize(1, &stcHwPara);
  assert(ECI_OK == hResult);
  ECIDEMO_CHECKERROR(ECIDRV_Initialize);
  
  //*** Get info about Board
  hResult = ECIDRV_GetInfo(dwHwIndex, &stcHwInfo);
  assert(ECI_OK == hResult);
  ECIDEMO_CHECKERROR(ECIDRV_GetInfo);

  //*** Find first CAN Controller of Board
  hResult = EciGetNthCtrlOfClass(&stcHwInfo,
      ECI_CTRL_CAN,
      0, //first relative controller
      &dwCtrlIndex);
  assert(ECI_OK == hResult);

  //*** Set CAN Controller configuration
  ECI_CTRL_CONFIG stcCtrlConfig = {0};
  stcCtrlConfig.wCtrlClass                = ECI_CTRL_CAN;
  stcCtrlConfig.u.sCanConfig.dwVer        = ECI_STRUCT_VERSION_V0;
  stcCtrlConfig.u.sCanConfig.u.V0.bBtReg0 = ECI_CAN_BT0_500KB;
  stcCtrlConfig.u.sCanConfig.u.V0.bBtReg1 = ECI_CAN_BT1_500KB;
  stcCtrlConfig.u.sCanConfig.u.V0.bOpMode = ECI_CAN_OPMODE_STANDARD | ECI_CAN_OPMODE_EXTENDED | ECI_CAN_OPMODE_ERRFRAME;

  //*** Open and Initialize given Controller of given board
  hResult = ECIDRV_CtrlOpen(&_dwCtrlHandle, dwHwIndex, dwCtrlIndex, &stcCtrlConfig);
  assert(hResult == ECI_OK); 
  ECIDEMO_CHECKERROR(ECIDRV_CtrlOpen);

  ECI_CTRL_CAPABILITIES stcCtrlCaps = {0}; 
  hResult = ECIDRV_CtrlGetCapabilities(_dwCtrlHandle, &stcCtrlCaps);
  ECIDEMO_CHECKERROR(ECIDRV_CtrlGetCapabilities); 

  hResult = ECIDRV_CtrlStart(_dwCtrlHandle);
  assert(hResult == ECI_OK);
  ECIDEMO_CHECKERROR(ECIDRV_CtrlStart);

  cout << _dwCtrlHandle << endl;

  cout << "Connected to CAN Device and Initialized" << endl; 
/*

  // connect to USB-CAN device
  if (dgc_serial_connect(&fd_, device.c_str(), 500000) < 0) {
    throw VLRException("Cannot connect to device " + device);
  }

  // set USB-CAN to 500K bitrate
  configCommand(SET_BAUD_500K);

  // send USB-CAN device into run mode
  configCommand('R');

  buf_size_ = 0;
*/ 
}

void UsbCan::close() {
  ECI_RESULT  hResult     = ECI_OK;

  hResult = ECIDRV_CtrlStop(_dwCtrlHandle, ECI_STOP_FLAG_RESET_CTRL);
  ECIDEMO_CHECKERROR(ECIDRV_CtrlStop);

  hResult = ECIDRV_CtrlClose(_dwCtrlHandle);
  ECIDEMO_CHECKERROR(ECIDRV_CtrlClose);
  _dwCtrlHandle = ECI_INVALID_HANDLE; 

  /*
  dgc_serial_close(fd_);
  */
}

void UsbCan::packageAndSend(const uint8_t* data, int data_size) {
  /*
  uint8_t buffer[300], chksum = 0;
  int i, n, size = 0;

  // start the transmission
  buffer[size++] = USB_DLE;
  buffer[size++] = USB_STX;

  // BYTE Stuff the data and calc checksum
  for (i = 0; i < data_size; i++) {
    chksum ^= data[i];
    if (data[i] == USB_DLE) buffer[size++] = USB_DLE;
    buffer[size++] = data[i];
  }

  // BYTE STUFF checksum if necessary
  if (chksum == USB_DLE) {
    buffer[size++] = USB_DLE;
  }
  // Send the check sum
  buffer[size++] = chksum;

  // terminate the transmission
  buffer[size++] = USB_DLE;
  buffer[size++] = USB_ETX;

  n = dgc_serial_writen(fd_, buffer, size, USB_TIMEOUT);

  if (n != size) {
    throw VLRException("Sending message failed.");
  }
  */
}

void UsbCan::sendMessage(uint32_t can_id, uint8_t* candata, uint32_t can_length) {
  ECI_RESULT  hResult     = ECI_OK;
  ECI_CTRL_MESSAGE stcCtrlMsg   = {0};

  assert(can_length <= 8); // can protocol limitation 
  stcCtrlMsg.wCtrlClass                            = ECI_CTRL_CAN;
  stcCtrlMsg.u.sCanMessage.dwVer                   = ECI_STRUCT_VERSION_V0;
  stcCtrlMsg.u.sCanMessage.u.V0.dwMsgId            = (can_id % (ECI_CAN_MAX_11BIT_ID +1));
  stcCtrlMsg.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc  = can_length;  
    
  memcpy(&stcCtrlMsg.u.sCanMessage.u.V0.abData[0],
      candata,
      can_length); 
  
  hResult = ECIDRV_CtrlSend(_dwCtrlHandle, &stcCtrlMsg, ECI_TX_TIMEOUT);
  ECIDEMO_CHECKERROR(ECIDRV_CtrlSend);
  assert(ECI_OK == hResult);

  /*
  uint8_t buffer[100];
  int i, buffer_len = 0;

  if (can_length <= 0 || can_length > 8) {
    throw VLRException("Invalid can message size request");
  }

  buffer[buffer_len++] = 0x80;
  buffer[buffer_len++] = (uint8_t) (can_id >> 24) & 0x1f;
  buffer[buffer_len++] = (uint8_t) (can_id >> 16) & 0xff;
  buffer[buffer_len++] = (uint8_t) (can_id >> 8) & 0xff;
  buffer[buffer_len++] = (uint8_t) (can_id) & 0xff;
  buffer[buffer_len++] = 0;
  buffer[buffer_len++] = 0;
  buffer[buffer_len++] = 0; // TxFlags - don't know what goes here
  buffer[buffer_len++] = can_length;
  for (i = 0; i < can_length; i++) {
    buffer[buffer_len++] = candata[i];
  }

  packageAndSend(buffer, buffer_len);
  */
}

void UsbCan::configCommand(uint8_t code) {
  /*
  uint8_t txBuf[5];
  int size = 0;

  txBuf[size++] = 0x00;
  txBuf[size++] = code | 0x80;
  packageAndSend(txBuf, size);
  */
}

bool UsbCan::validMessage(uint8_t* buffer, int buf_size, uint8_t* decoded, int* decoded_length) {
  return false; 
  /*
  uint8_t computed_checksum;
  int i;
  

  // if message is too short, it can't be valid
  if (buf_size < 5) {
//    throw VLRException("Message too short.");
    return false;
  }

  // make sure message starts with preamble
  if (buffer[0] != USB_DLE || buffer[1] != USB_STX) {
    //throw VLRException("Invalid message preamble.");
    return false;
  }

  // make sure message ends with end bytes
  if (buffer[buf_size - 2] != USB_DLE || buffer[buf_size - 1] != USB_ETX) {
    // throw VLRException("Invalid message end bytes.");
    return false;
  }

  // un-stuff message
  *decoded_length = 0;
  for (i = 2; i < buf_size - 2; i++) {
    if (i == 2 || buffer[i] != USB_DLE || buffer[i - 1] != USB_DLE) {
      decoded[(*decoded_length)++] = buffer[i];
    }
  }

  computed_checksum = 0;
  for (i = 0; i < *decoded_length - 1; i++) {
    computed_checksum ^= decoded[i];
  }

  // return the decoded message if the checksum is valid
  if (decoded[*decoded_length - 1] != computed_checksum) {
    (*decoded_length)--;
   // throw VLRException(std::string(": Message checksum does not match."));
    return false;
  }

return true;
  */
}

bool UsbCan::readMessage(uint32_t& can_id, uint8_t* candata, uint32_t& can_length) {
  ECI_RESULT hResult = ECI_OK;
  ECI_CTRL_MESSAGE message = {0};  // struct to store data
  DWORD dwCount = 1; // number of messages to receive
  hResult = ECIDRV_CtrlReceive(_dwCtrlHandle, &dwCount, &message, ECI_RX_TIMEOUT);
  //ECIDEMO_CHECKERROR(ECIDRV_CtrlReceive);

  if (dwCount == 0 || ECI_ERR_NO_MORE_DATA == hResult) return false;

  assert(ECI_OK == hResult);
  assert(ECI_CTRL_CAN == message.wCtrlClass);
  assert(ECI_STRUCT_VERSION_V0 == message.u.sCanMessage.dwVer); 

  can_id = message.u.sCanMessage.u.V0.dwMsgId; 
  can_length = message.u.sCanMessage.u.V0.uMsgInfo.Bits.dlc; 
  memcpy(candata, 
        &message.u.sCanMessage.u.V0.abData[0],
        can_length);

  return true; 

  /*
  int decoded_length, i, found_start, found_end, found_message = 0;

  // read what is available in the buffer
  int bytes_available = dgc_serial_bytes_available(fd_);
  if (bytes_available > USBCAN_BUFFER_SIZE - buf_size_) {
    bytes_available = USBCAN_BUFFER_SIZE - buf_size_;
  }
  int bytes_read = dgc_serial_readn(fd_, buffer_ + buf_size_, bytes_available, USBCAN_READ_TIMEOUT);
  if (bytes_read > 0) {buf_size_ += bytes_read;}

  // look for message preamble
  found_start = -1;
  for (int i = 0; i < buf_size_ - 1; i++) {
    if (buffer_[i] == USB_DLE && buffer_[i + 1] == USB_STX) {
      found_start = i;
      break;
    }
  }

  if (found_start == -1) {
    buf_size_ = 0;
  }
  else {
      // move the message so preamble appears at start of buffer
    if (found_start > 0) {
      memmove(buffer_, buffer_ + found_start, buf_size_ - found_start);
      buf_size_ -= found_start;
    }

      // look for message end
    found_end = -1;
    for (i = 0; i < buf_size_ - 1; i++) {
      if (buffer_[i] == USB_DLE && buffer_[i + 1] == USB_ETX) {
        found_end = i;
        break;
      }
    }

    // try and decode the message, see if it is valid
    if (found_end != -1) {
      if(validMessage(buffer_, found_end + 2, decoded_, &decoded_length)) {

      *can_id = (decoded_[4] << 24) | (decoded_[3] << 16) | (decoded_[2] << 8) | decoded_[1];
      *can_length = decoded_[6];
      for (i = 0; i < *can_length; i++) {
        candata[i] = decoded_[7 + i];
      }
      
      found_message = 1;
    }

        // delete the message from the buffer
      if (buf_size_ == found_end + 2) buf_size_ = 0;
      else {
        memmove(buffer_, buffer_ + found_end + 2, buf_size_ - (found_end + 2));
        buf_size_ -= (found_end + 2);
      }
    }
  }

  return found_message;
  */
}
