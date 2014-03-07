///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definition of types, structs and unions for logging feature.

  @file ECI_logging.h
*/

#ifndef __ECI_LOGGING_H__
#define __ECI_LOGGING_H__


//////////////////////////////////////////////////////////////////////////
// include files
#include "ECI_hwtype.h"

#include <ECI_pshpack1.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros


//////////////////////////////////////////////////////////////////////////
// data types

/**
  ECI logging mode

  @ingroup LogTypes
*/
typedef enum
{
  ECI_LOGGING_MODE_UNDEFINED = 0x00, ///< Undefined
  ECI_LOGGING_MODE_FIFO      = 0x01  ///< Use FIFO, new log events are discarded if FIFO is full.
} e_ECI_LOGGING_MODE;


/**
  ECI logging sources

  @ingroup LogTypes
*/
typedef enum
{
  ECI_LOGGING_SOURCE_NONE          = 0x0000, ///< Log no messages
  ECI_LOGGING_SOURCE_INFO_ALL      = 0x1FFF, ///< Log all info messages
  ECI_LOGGING_SOURCE_INFO_ECI_API  = 0x1001, ///< Log info ECI API messages
  ECI_LOGGING_SOURCE_WARNING_ALL   = 0x2FFF, ///< Log all warning messages
  ECI_LOGGING_SOURCE_WARNING_TS    = 0x2001, ///< Log warning timestamp messages
  ECI_LOGGING_SOURCE_ERROR_ALL     = 0x4FFF, ///< Log all error messages
  ECI_LOGGING_SOURCE_ERROR_CCI     = 0x4001, ///< Log error CCI messages
  ECI_LOGGING_SOURCE_DEBUG_ALL     = 0x8FFF, ///< Log all debug messages
  ECI_LOGGING_SOURCE_DEBUG_TS      = 0x8001, ///< Log debug timestamp messages
  ECI_LOGGING_SOURCE_DEBUG_DEVMSG  = 0x8002, ///< Log debug device messages
  ECI_LOGGING_SOURCE_ALL           = 0xFFFF  ///< Log all message
} e_ECI_LOGGING_SOURCE;


/**
  ECI logging flags

  @ingroup LogTypes
*/
typedef enum
{
  ECI_LOGGING_FLAG_NONE     = 0x00, ///< Undefined
  ECI_LOGGING_FLAG_OVERRUN  = 0x01  ///< Set if an overrun occured,
                                    ///< at least one entry is missing after this entry.
} e_ECI_LOGGING_FLAGS;


/**
  Structure for a logging message.

  @ingroup LogTypes
*/
typedef struct
{
  DWORD     dwVer;          ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwTime;        ///< System time on which log entry was generated in [ms]
      DWORD dwSource;      ///< Source of log entry
      DWORD dwFlags;       ///< Flags for log entry @see e_ECI_LOGGING_FLAGS
      DWORD dwLostCount;   ///< Total number of log entries lost since last configuration.
      char  szLog[0x100];  ///< Logging message
    } V0;                  ///< Version 0
  } u;                     ///< Version controlled structs container

} ECI_LOG_ENTRY;


/**
  Structure to configure the logging.

  @ingroup LogTypes
*/
typedef struct
{
  DWORD     dwVer;          ///< Version of valid union struct

  union
  {
    struct
    {
      DWORD dwLogMode;      ///< Logging mode @see e_ECI_LOGGING_MODE
      DWORD dwLogSources;   ///< Sources to log @see e_ECI_LOGGING_SOURCE
      DWORD dwLogSize;      ///< Number of entries, dependant from dwLogMode.
    } V0;                   ///< Version 0
  } u;                      ///< Version controlled structs container

} ECI_LOG_CONFIG;

#include <ECI_poppack.h>

#endif //__ECI_LOGGING_H__
