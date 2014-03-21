///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of types, structs and unions for LIN initialization and 
  communication.

  @file ECI_lintype.h
*/


#ifndef __ECI_LINTYPE_H__
#define __ECI_LINTYPE_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include <OsEci.h>

#include <ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros

/*****************************************************************************
* LIN baud rates
****************************************************************************/
#define ECI_LIN_BITRATE_UNDEF      0xFFFF ///< Undefined bit-rate @ingroup LinTypes
#define ECI_LIN_BITRATE_MIN        1000   ///< Lowest specified bit-rate @ingroup LinTypes
#define ECI_LIN_BITRATE_MAX        20000  ///< Highest specified bit-rate @ingroup LinTypes


/**
  LIN bitrate definitions

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_BITRATE_AUTO  = 0,           ///<  Automatic bit-rate detection
  ECI_LIN_BITRATE_1000  = 1000,        ///<  1000 baud
  ECI_LIN_BITRATE_1200  = 1200,        ///<  1200 baud
  ECI_LIN_BITRATE_2400  = 2400,        ///<  2400 baud
  ECI_LIN_BITRATE_4800  = 4800,        ///<  4800 baud
  ECI_LIN_BITRATE_9600  = 9600,        ///<  9600 baud
  ECI_LIN_BITRATE_10400 = 10400,       ///< 10400 baud
  ECI_LIN_BITRATE_19200 = 19200,       ///< 19200 baud
  ECI_LIN_BITRATE_20000 = 20000        ///< 20000 baud
} e_LINBITRATES;


/** Maximum possible 6bit LIN ID  @ingroup LinTypes */
#define ECI_LIN_MAX_6BIT_ID 0x3F

/**
  LIN controller types

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_CTRL_UNKNOWN = 0x00,         ///< unknown
  ECI_LIN_CTRL_GENERIC = 0x01,         ///< Generic LIN Controller
  ECI_LIN_CTRL_MAXVAL  = 0xFF          ///< Maximum value for controller type
} e_LINCTRLCLASS;


/**
  LIN controller supported features.

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_FEATURE_UNDEFINED = 0x0000,   ///< undefined
  ECI_LIN_FEATURE_MASTER    = 0x0001,   ///< Master mode
  ECI_LIN_FEATURE_AUTORATE  = 0x0002,   ///< Automatic bitrate detection
  ECI_LIN_FEATURE_ERRFRAME  = 0x0004,   ///< Reception of error frames
  ECI_LIN_FEATURE_BUSLOAD   = 0x0008    ///< Bus load measurement
} e_LINCTRLFEATURE;


/**
  LIN controller bus coupling types.

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_BUSC_UNDEFINED = 0x0000,     ///< undefined
  ECI_LIN_BUSC_STANDARD  = 0x0001      ///< Standard
} e_LINBUSC ;


/**
  LIN controller operating modes.

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_OPMODE_SLAVE     = 0x00,     ///< Enable slave mode
  ECI_LIN_OPMODE_MASTER    = 0x01,     ///< Enable master mode
  ECI_LIN_OPMODE_ERRFRAME  = 0x02      ///< Enable reception of error frames
} e_LINOPMODE;


/**
  LIN message types (used by <LINMSGINFO.Bytes.bType>).

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_MSGTYPE_DATA    = 0,         ///< Data frame
  ECI_LIN_MSGTYPE_INFO    = 1,         ///< Info frame
  ECI_LIN_MSGTYPE_ERROR   = 2,         ///< Error frame
  ECI_LIN_MSGTYPE_STATUS  = 3,         ///< Status frame
  ECI_LIN_MSGTYPE_WAKEUP  = 4,         ///< Wakeup frame
  ECI_LIN_MSGTYPE_TIMEOVR = 5,         ///< Timer overrun
  ECI_LIN_MSGTYPE_TIMERST = 6          ///< Timer reset
} e_LINMSGTYPE;


/**
  LIN message information flags (used by <LINMSGINFO.Bytes.bFlags>).

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_MSGFLAGS_DLC = 0x0F,         ///< Data length code
  ECI_LIN_MSGFLAGS_OVR = 0x10,         ///< Data overrun flag (RX direction only).
  ECI_LIN_MSGFLAGS_SOR = 0x20,         ///< Sender of response. @copydetails ECI_LINMSGINFO::sor
  ECI_LIN_MSGFLAGS_ECS = 0x40,         ///< Enhanced checksum (according to LIN Spec. 2.0)
  ECI_LIN_MSGFLAGS_IDO = 0x80          ///< ID only. @copydetails ECI_LINMSGINFO::ido
} e_LINMSGFLAGS;


/**
  LIN message information flags2 (used by <LINMSGINFO.Bytes.bFlags2>).

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_MSGFLAGS2_BUF = 0x01          ///< Update LIN slave response buffer only
                                        ///< (TX direction only).
} e_LINMSGFLAGS2;


/**
  Information supplied in the abData[0] field of info frames
  (LINMSGINFO.Bytes.bType = LIN_MSGTYPE_INFO).

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_INFO_START = 1,              ///< Start of LIN controller
  ECI_LIN_INFO_STOP  = 2,              ///< Stop of LIN controller
  ECI_LIN_INFO_RESET = 3               ///< Reset of LIN controller
} e_LININFO;


/**
  Error information supplied in the abData[0] field of error frames
  (LINMSGINFO.Bytes.bType = LIN_MSGTYPE_ERROR).

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_ERROR_BIT    = 1,            ///< Bit error
  ECI_LIN_ERROR_CHKSUM = 2,            ///< Checksum error
  ECI_LIN_ERROR_PARITY = 3,            ///< Identifier parity error
  ECI_LIN_ERROR_SLNORE = 4,            ///< Slave not responding error
  ECI_LIN_ERROR_SYNC   = 5,            ///< Inconsistent sync field error
  ECI_LIN_ERROR_NOBUS  = 6,            ///< No bus activity error
  ECI_LIN_ERROR_OTHER  = 7             ///< Other (unspecified) error
} e_LINERROR;


/**
  Status information supplied in the abData[0] field of status frames
  (LINMSGINFO.Bytes.bType = ECI_LIN_MSGTYPE_STATUS) and in
  ECI_LINSTATUS::u::V0::dwStatus.

  @ingroup LinTypes
*/
typedef enum
{
  ECI_LIN_STATUS_OVRRUN = 0x01,        ///< Data overrun occurred
  ECI_LIN_STATUS_ININIT = 0x10         ///< Init mode active
} e_LINSTATUS;


//////////////////////////////////////////////////////////////////////////
// data types


/**
  LIN controller configuration.

  @ingroup LinTypes
*/
typedef struct
{
  DWORD dwVer;                ///< Version of valid union struct

  union
  {
    struct
    {
      BYTE  bOpMode;          ///< LIN operating mode @see e_LINOPMODE
      BYTE  bReserved;        ///< Reserved set to 0
      WORD  wBitrate;         ///< LIN bitrate @see e_LINBITRATES
    } V0;                     ///< Version 0
  } u;                        ///< Version controlled structs container
} ECI_LININITLINE;


/**
  LIN controller capabilities.
  
  @ingroup LinTypes
*/
typedef struct
{
  DWORD dwVer;                ///< Version of valid union struct

  union
  {
    struct
    {
      WORD  wLinType;         ///< Type of LIN controller @see e_LINCTRLCLASS
      WORD  wBusCoupling;     ///< Type of Bus coupling @see e_LINBUSC
      DWORD dwFeatures;       ///< Supported features @see e_LINCTRLFEATURE
      DWORD dwClockFreq;      ///< Clock frequency of the primary counter in Hz
      DWORD dwTscDivisor;     ///< Divisor for the message time stamp counter
      DWORD dwDtxDivisor;     ///< Divisor for the delayed message transmitter
      DWORD dwDtxMaxTicks;    ///< Maximum tick count value of the delayed message transmitter
      DWORD dwNoOfPrioQueues; ///< Number of priority TX queues
    } V0;                     ///< Version 0
  } u;                        ///< Version controlled structs container
} ECI_LINCAPABILITIES;


/**
  LIN controller status.
  
  @ingroup LinTypes
*/
typedef struct
{
  DWORD dwVer;                ///< Version of valid union struct

  union
  {
    struct
    {
      BYTE  bOpMode;          ///< Current LIN operating mode @see e_LINOPMODE
      BYTE  bBusLoad;         ///< Average bus load in percent (0..100)
      WORD  wBitrate;         ///< Current LIN bitrate @see e_LINBITRATES
      DWORD dwStatus;         ///< Status of the LIN controller @see e_LINSTATUS
    } V0;                     ///< Version 0
  } u;                        ///< Version controlled structs container
} ECI_LINSTATUS;


/**
  LIN message information.
  
  @ingroup LinTypes
*/
typedef union
{
  struct
  {
    BYTE  bType;          ///< Message type @see e_LINMSGTYPE
    BYTE  bFlags2;        ///< Flags @see e_LINMSGFLAGS2
    BYTE  bFlags;         ///< Flags @see e_LINMSGFLAGS
    BYTE  bReserved2;     ///< Reserved for future use
  } Bytes;                ///< LIN Message information in byte format

  struct
  {
    DWORD type: 8;        ///< Message type @see e_LINMSGTYPE
    DWORD buf : 1;        ///< Update LIN slave response buffer only (TX direction only).
    DWORD res : 7;        ///< Reserved for future use
    DWORD dlc : 4;        ///< Data length code
    DWORD ovr : 1;        ///< Possible data overrun (RX direction only).
    DWORD sor : 1;        ///< Sender of response. Set to send slave buffer data upon reception if LIN ID in 
                          ///< slave mode. To disable a slave buffer send a slave message without setting this flag.
    DWORD ecs : 1;        ///< Enhanced checksum (according to LIN Spec. 2.0)
    DWORD ido : 1;        ///< ID only. Set to send a LIN frame without data in master mode. Upon reception 
                          ///< this flag indicates that no slave responded to this LIN ID.
    DWORD res2: 8;        ///< Reserved for future use
  } Bits;                 ///< LIN Message information in bit format
} ECI_LINMSGINFO;


/**
  LIN message information.
  
  @ingroup LinTypes
*/
typedef struct
{
  DWORD dwVer;                    ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD           dwTime;     ///< Time stamp for receive message in [us]
      DWORD           dwMsgId;    ///< LIN message identifier (INTEL format)
      ECI_LINMSGINFO  uMsgInfo;   ///< LIN message information (bit field)
      BYTE            abData[8];  ///< Message data
    } V0;                         ///< Version 0
  } u;                            ///< Version controlled structs container
} ECI_LINMESSAGE;


/**
  LIN command structure.
  
  @ingroup LinTypes
*/
typedef struct
{
  WORD  wCode;                    ///< Command request code
} ECI_LIN_CMD_REQ_HD;


/**
  LIN command structure.
  
  @ingroup LinTypes
*/
typedef struct
{
  WORD  wResult;                  ///< Command result code
} ECI_LIN_CMD_RES_HD;


/**
  LIN command structure.
  
  @ingroup LinTypes
*/
typedef struct
{
  DWORD                   dwVer;      ///< Version of valid union struct

  union
  {
    struct
    {
      ECI_LIN_CMD_REQ_HD  sCmdHeader; ///< LIN command header @see ECI_LIN_CMD_REQ_HD
      DWORD               dwReserved; ///< reserved for future use
    } V0;                             ///< Version 0
  } u;                                ///< Version controlled structs container
} ECI_LINCMDREQUEST;


/**
  LIN Command structure.
  
  @ingroup LinTypes
*/
typedef struct
{
  DWORD                   dwVer;      ///< Version of valid union struct

  union
  {
    struct
    {
      ECI_LIN_CMD_RES_HD  sCmdHeader; ///< LIN command header @see ECI_LIN_CMD_RES_HD
      DWORD               dwReserved; ///< reserved for future use
    } V0;                             ///< Version 0
  } u;                                ///< Version controlled structs container
} ECI_LINCMDRESPONSE;


/**
  LIN filter structure.
  
  @ingroup LinTypes
*/
typedef struct
{
  DWORD dwVer;                    ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwReserved;           ///< Reserved for future use
    } V0;                         ///< Version 0
  } u;                            ///< Version controlled structs container
} ECI_LINFILTER ;

#include <ECI_poppack.h>

#endif