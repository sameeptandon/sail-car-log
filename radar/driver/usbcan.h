#pragma once
 
#include <inttypes.h>
#include <ECI10A.h>
#include <EciDemoCommon.h>
#include <string>
#include <assert.h>
#include <iostream>

/*
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
*/ 

#define ECI_TX_TIMEOUT 100 // time in milliseconds for sending packets
#define ECI_RX_TIMEOUT 100 // time in milliseconds for sending packets

/** ECI Demo error check macro @ingroup EciDemo */
#define ECIDEMO_CHECKERROR(FuncName) \
{\
  if(ECI_OK == hResult)\
  {\
    OS_Printf(#FuncName "...succeeded.\n"); \
  }\
  else\
  {\
    OS_Printf( #FuncName "...failed with errorcode: 0x%08X. %s\n", \
               hResult, \
               ECI10A_GetErrorString(hResult)); \
  }\
}

using namespace std; 

class UsbCan {

public:
  UsbCan();
  virtual ~UsbCan();
  void open(const std::string& device);
  void close();

  bool readMessage(uint32_t& can_id, uint8_t* candata, uint32_t& can_length);
  void sendMessage(uint32_t can_id, uint8_t* candata, uint32_t can_length);
  void configCommand(uint8_t code);

private:
  void packageAndSend(const uint8_t* data, int data_size);
  bool validMessage(uint8_t* buffer, int buf_size, uint8_t* decoded, int* decoded_length);

private:
  ECI_CTRL_HDL _dwCtrlHandle; 
  //int fd_;
  //int buf_size_;
  //uint8_t buffer_[USBCAN_BUFFER_SIZE];
  //uint8_t decoded_[USBCAN_BUFFER_SIZE];
};

/**
  Returns the nth controller of given controller class.

  @param pstcHwInfo
    Reference to ECI_HW_INFO to find controller from.

  @param eCtrlClass
    Class of controller to find.

  @param dwRelCtrlIndex
    Relative controller index of given controller class.

  @param pdwCtrIndex
    Variable which receives the controller index.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciDemo
ECI_RESULT EciGetNthCtrlOfClass( const ECI_HW_INFO* pstcHwInfo,
                                 e_CTRLCLASS        eCtrlClass,
                                 DWORD              dwRelCtrlIndex,
                                 DWORD*             pdwCtrIndex)
{
  ECI_RESULT hResult = ECI_ERR_RESOURCE_NOT_FOUND;

  //*** Check Arguments
  if((NULL != pstcHwInfo) && (NULL != pdwCtrIndex))
  {
    //*** Struct Version 0
    if(pstcHwInfo->dwVer == ECI_STRUCT_VERSION_V0)
    {
      DWORD dwIndex = 0;
      //*** Iterate through all controllers
      for(dwIndex=0; dwIndex < pstcHwInfo->u.V0.dwCtrlCount; dwIndex++)
      {
        if(pstcHwInfo->u.V0.sCtrlInfo[dwIndex].wCtrlClass == eCtrlClass)
        {
          //*** Controller not found yet
          if(hResult != ECI_OK)
          {
            if(dwRelCtrlIndex == 0)
            {
              //*** Controller found
              *pdwCtrIndex = dwIndex;
              hResult    = ECI_OK;
            }
            else
            {
              dwRelCtrlIndex--;
            }
          }
        }//endif
      }//end for
    }//endif
  }
  else
  {
    hResult = ECI_ERR_INVALID_POINTER;
  }

  return hResult;
}
*/
