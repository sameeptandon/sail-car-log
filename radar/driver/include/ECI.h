///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API function definitions for all IXXAT Interfaces.

  Header file of the ECI ( embedded / real time communication interface),
  a generic library for IXXAT hardware interfaces.

  @file ECI.h
*/

#ifndef ECIDRV_
  #error Error device driver type is not defined. Please device driver type
#endif

//#ifndef __ECI_H__
//#define __ECI_H__

//////////////////////////////////////////////////////////////////////////
// include files

#include <OsEci.h>
#include "ECI_hwtype.h"
#include "ECI_error.h"
#include "ECI_logging.h"


//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif


///////////////////////////////////////////////////////////////////////////////
/**
  Initializes the ECI internal hardware structures. This function must be
  called at first. After that the ECI is in the state ECI_INITIALIZED.

  @param dwCount
    Number of defined hardware parameters defined in ECI_HW_PARA.
  @param astcHwPara
    Array of structure of ECI_HW_PARA to define hardware parameters, which 
    cannot be obtained from the OS.
    ( e.g. ISA-card: memory-Address and the interrupt line)

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @note
    Calling this function invalidates all already distributed controller handles!
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_Initialize ( const DWORD       dwCount,
                                           const ECI_HW_PARA astcHwPara[] );


///////////////////////////////////////////////////////////////////////////////
/**
  Releases the ECI internal hardware structures and any initializations made
  by the user before. After that the ECI is in the state ECI_UNINITIALIZED.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @note
    Calling this function invalidates all already distributed controller handles!
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_Release ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  This function returns information of the hardware given by index.

  The following information is available:
    - firmware version
    - number of available controllers
    - controller types
    - controller states

  @param dwHwIndex
    Index of hardware resp. index into the astcHwPara array (hardware dependent).
  @param pstcHwInfo
    Pointer to ECI_HW_INFO which receives the hardware information.
  
  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_GetInfo ( const DWORD  dwHwIndex,
                                        ECI_HW_INFO* pstcHwInfo );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to open and / or initialize a controller of the given hardware. 
  The hardware has to be initialized before by calling ECIDRV_Initialize.

  The function:
    - loads and starts the firmware (if necessary)
    - initializes the controller
    - delivers a handle for further controller usage
  After that the controller can be started.

  This function can also be used to re-configure the controller e.g. changing
  the baud rate without closing the controller handle. Therefore the controller 
  has to be stopped by calling ECIDRV_CtrlStop before. The controller handle 
  does not change and thus remains valid. It also saves time by avoiding 
  reconfiguration of the whole ECI interface.

  ECI library state transition: (for the first controller opened)
    ECI_INITIALIZED -> ECI_CONFIGURED

  ECI controller state transition: (if controller was closed before)
    ECI_CTRL_UNCONFIGURED -> ECI_CTRL_INITIALIZED

  @param phCtrl
    Pointer to ECI_CTRL_HDL which receives the handle to the controller.
  @param dwHwIndex
    Index of hardware resp. index into the astcHwPara array (hardware dependent).
  @param dwCtrlIndex
    Index of the controller (LIN or CAN) to initialize (hardware relative).
  @param pstcCtrlConfig
    Pointer to ECI_CTRL_CONFIG which holds the configuration to initialize
    the controller.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @see ECIDRV_CtrlClose
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlOpen ( ECI_CTRL_HDL*          phCtrl,
                                         const DWORD            dwHwIndex,
                                         const DWORD            dwCtrlIndex,
                                         const ECI_CTRL_CONFIG* pstcCtrlConfig );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to close the controller. After this function, the initialization 
  settings are lost and the controller is in ECI_CTRL_UNCONFIGURED state. The
  passed controller handle becomes invalid and should afterwards be set to
  ECI_INVALID_HANDLE.

  ECI library state transition:  (for the last controller closed)
    ECI_CONFIGURED -> ECI_INITIALIZED

  ECI controller state transition:
    ECI_CTRL_INITIALIZED or ECI_CTRL_RUNNING -> ECI_CTRL_UNCONFIGURED

  @param hCtrl
    Controller handle

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @see ECIDRV_CtrlOpen
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlClose ( const ECI_CTRL_HDL hCtrl );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to start the controller communication.

  ECI controller state transition:
    ECI_CTRL_INITIALIZED -> ECI_CTRL_RUNNING

  @param hCtrl
    Controller handle

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @see ECIDRV_CtrlStop
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlStart ( const ECI_CTRL_HDL hCtrl );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to stop the controller communication.

  ECI state transition:
    ECI_CTRL_RUNNING -> ECI_CTRL_INITIALIZED

  @param hCtrl
    Controller handle
  @param dwMode (bit field)
    Different stop mode bits defined in e_STOP_FLAGS.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @see ECIDRV_CtrlStart
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlStop  ( const ECI_CTRL_HDL hCtrl,
                                          const DWORD        dwMode );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the capabilities of the controller. 

  @param hCtrl
    Controller handle
  @param pstcCapabilities
    Pointer to ECI_CTRL_CAPABILITIES which receives the controller capabilities.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlGetCapabilities ( const ECI_CTRL_HDL     hCtrl,
                                                    ECI_CTRL_CAPABILITIES* pstcCapabilities );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current status of the controller. 

  @param hCtrl
    Controller handle
  @param pstcStatus
    Pointer to ECI_CTRL_STATUS which receives the controller status.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlGetStatus ( const ECI_CTRL_HDL hCtrl,
                                              ECI_CTRL_STATUS*   pstcStatus );


///////////////////////////////////////////////////////////////////////////////
/**
  Writes on message to the controller's TX FIFO. Therefore the controller has 
  to be in the ECI_CTRL_RUNNING state.

  @param hCtrl
    Controller handle
  @param pstcMessage
    Pointer to ECI_CTRL_MESSAGE which holds the message to send.
  @param dwTimeout
    Timeout in [ms] to wait for successful transmission.
    (Waits until the message could be written into the TX FIFO, dependent from 
     hardware type) @n
    Following pre-defined values can be used for polling or blocking mode:
    @ref ECI_NO_WAIT, @ref ECI_WAIT_FOREVER

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @note
    If driver is running in polling mode, the timeout value must be
    configured to zero. Timeout handling is not available in polling
    mode and must be handled by the user as follows:
    Message transmission:
    * Call ECIDRV_CtrlSend without timeout.
    * If function returns with success you can immediately call this
      function again.
    * If function returns with error, wait some time, e.g. 1 ms
      by calling \ref OS_Sleep before calling this function again.
      Please also refer to \ref api_InterruptVsPolling.

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlSend ( const ECI_CTRL_HDL      hCtrl,
                                         const ECI_CTRL_MESSAGE* pstcMessage,
                                         const DWORD             dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Reads one or messages from the controller's RX FIFO. Therefore the 
  controller has to be in the ECI_CTRL_INITIALIZED or ECI_CTRL_RUNNING state.
  
  This function should be called the after controller is started to ensure no 
  receive messages are lost. This is also necessary if no message reception
  is expected at this time. The ECI interface sends status and timer messages
  to the ECI API which have to processed by the ECI internally. This is done 
  in the ECIDRV_CtrlReceive function to avoid thread locking and thread
  synchronization overhead.

  @param hCtrl
    Controller handle
  @param pdwCount
    Pointer to DWORD @n
    [in] : Holds the number of entries of ECI_CTRL_MESSAGE which should be 
           retrieved. @n
    [out]: Number of entries of ECI_CTRL_MESSAGE copied into.
  @param pstcMessage
    Pointer to ECI_CTRL_MESSAGE which receives the message(s) read from RX FIFO.
  @param dwTimeout
    Timeout in [ms] to wait for message reception.
    (Waits until one or more messages could be read from the RX FIFO, dependent 
    from hardware type) @n
    Following pre-defined values can be used for polling or blocking mode:
    @ref ECI_NO_WAIT, @ref ECI_WAIT_FOREVER

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @note
    If driver is running in polling mode, the timeout value must be
    configured to zero. Timeout handling is not available in polling
    mode and must be handled by the user as follows:
    Message reception:
    * Call ECIDRV_CtrlReceive without timeout.
    * If function returns with success immediately call this function again.
    * If function returns with no message, wait some time, e.g. 1 ms
      by calling \ref OS_Sleep before calling this function again.
      Please also refer to \ref api_InterruptVsPolling.

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlReceive ( const ECI_CTRL_HDL hCtrl,
                                            DWORD*             pdwCount,
                                            ECI_CTRL_MESSAGE*  pstcMessage,
                                            const DWORD        dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to configure message filtering.

  @param hCtrl
    Controller handle
  @param pstcFilter
    Pointer ECI_CTRL_FILTER which holds the filter settings to be set.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlSetFilter ( const ECI_CTRL_HDL     hCtrl,
                                              const ECI_CTRL_FILTER* pstcFilter );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to send a user defined command to the driver or hardware.

  @param hCtrl
    Controller handle
  @param pstcCmdRequest
    Pointer to ECI_CTRL_CMDREQUEST which holds the request to send.
  @param pstcCmdResponse
    Pointer to ECI_CTRL_CMDRESPONSE which receives the response on success.
  @param dwTimeout
    Timeout in [ms] to wait for the response.
    Following pre-defined values can be used for polling or blocking mode:
    @ref ECI_NO_WAIT, @ref ECI_WAIT_FOREVER

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_CtrlCommand ( const ECI_CTRL_HDL         hCtrl,
                                            const ECI_CTRL_CMDREQUEST* pstcCmdRequest,
                                            ECI_CTRL_CMDRESPONSE*      pstcCmdResponse,
                                            const DWORD                dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to configure ECI API's build in logging mechanism.

  @param pstcConfig
    Pointer to ECI_LOG_CONFIG which holds the configuration to set.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".
 
  @ingroup EciApi

  @note 
    This function can only be called if the logging is stopped. @n
    To set the logging API to uninitialized state, call this function with
    dwLogMode set to ECI_LOGGING_MODE_UNDEFINED and
    dwLogSources set to ECI_LOGGING_SOURCE_NONE and
    dwLogSize set to zero.
 */
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_LogConfig ( const ECI_LOG_CONFIG* pstcConfig );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to read one or more entries logged.

  @param pdwCount
    Pointer to DWORD @n
    [in] : Holds the number of entries of ECI_LOG_ENTRY which should be 
           retrieved. @n
    [out]: Number of entries of ECI_LOG_ENTRY received.
  @param astcEntry
    Array of ECI_LOG_ENTRY which receives the logging entries.
  @param dwTimeout
    Time to wait for the given amount of logging entries.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
 */
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_LogRead ( DWORD*        pdwCount,
                                        ECI_LOG_ENTRY astcEntry[],
                                        const DWORD   dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to start the logging mechanism.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_LogStart ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to stop the logging mechanism.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi
*/
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_LogStop ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to delete all currently logged entries.

  @retval ECI_RESULT
    ECI_OK on success, otherwise an error code from the @ref e_ECIERROR "ECI error list".

  @ingroup EciApi

  @note This function can only be called if the logging is stopped.
 */
ECI_DLLEXPORT
ECI_RESULT ECI_APICALL ECIDRV_LogClear ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable error description of the given error
  code. The returned string is zero-terminated and valid until a further call
  (from any thread) to this function.

  @param dwError
    ECI error code to retrieve error description from

  @retval "const char*"
    Pointer to a zero-terminated error string.
  
  @see e_ECIERROR

  @ingroup EciApi
  
  @note An ECI error string can only be retrieved after a successful call of
    @ref ECIDRV_Initialize.
*/
ECI_DLLEXPORT
const char* ECI_APICALL ECIDRV_GetErrorString (const ECI_RESULT dwError );

#ifdef __cplusplus
}
#endif // __cplusplus


//#endif //__ECI_H__
