///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  OS dependent function hiding

  @file OsEci.h
*/

#ifndef __OSECI_H__
#define __OSECI_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <stdio.h>
#include <memory.h> // for memset (softer than including string.h)
#include <errno.h>  // for ENOMEM
#include <poll.h>   // for poll
#include <stddef.h> // for offsetof
#include <stdlib.h> // for malloc / free

#include <ECI_error.h> // for error definition

//////////////////////////////////////////////////////////////////////////
// constants and macros

/** DLL EXPORT definition */
#ifdef __DLL_EXPORT__
  #define ECI_DLLEXPORT __attribute__ ((visibility("default")))
#else
  #define ECI_DLLEXPORT
#endif

/** Calling convention */
#if defined(__i386__)
  #define ECI_APICALL __attribute__ ((stdcall))
#elif defined(__amd64__)
  #define ECI_APICALL
#endif

typedef void          VOID;         ///< Define used datatype to easier porting the ECI to another OS
typedef char          CHAR;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint8_t     BYTE;         ///< Define used datatype to easier porting the ECI to another OS
typedef __int32_t     BOOL;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint16_t    WORD;         ///< Define used datatype to easier porting the ECI to another OS
typedef __uint32_t    DWORD;        ///< Define used datatype to easier porting the ECI to another OS
typedef __uint64_t    QWORD;        ///< Define used datatype to easier porting the ECI to another OS
typedef long          LONG;         ///< Define used datatype to easier porting the ECI to another OS
typedef unsigned long ULONG;        ///< Define used datatype to easier porting the ECI to another OS
typedef __int32_t     INT32;        ///< Define used datatype to easier porting the ECI to another OS
typedef __int64_t     INT64;        ///< Define used datatype to easier porting the ECI to another OS
typedef __int32_t     HRESULT;      ///< Define used datatype to easier porting the ECI to another OS
typedef float         FLOAT;        ///< Define used datatype to easier porting the ECI to another OS

typedef VOID          *PVOID;       ///< Define used datatype to easier porting the ECI to another OS
typedef CHAR          *PCHAR;       ///< Define used datatype to easier porting the ECI to another OS
typedef BYTE          *PBYTE;       ///< Define used datatype to easier porting the ECI to another OS
typedef BOOL          *PBOOL;       ///< Define used datatype to easier porting the ECI to another OS
typedef WORD          *PWORD;       ///< Define used datatype to easier porting the ECI to another OS
typedef DWORD         *PDWORD;      ///< Define used datatype to easier porting the ECI to another OS
typedef LONG          *PLONG;       ///< Define used datatype to easier porting the ECI to another OS
typedef FLOAT         *PFLOAT;      ///< Define used datatype to easier porting the ECI to another OS
typedef void          *HANDLE;      ///< Define used datatype to easier porting the ECI to another OS
typedef HANDLE        *PHANDLE;     ///< Define used datatype to easier porting the ECI to another OS


#define LOBYTE(wVal)  ((BYTE) wVal)           ///< Macro for accessing low byte @ingroup OsEci
#define HIBYTE(wVal)  ((BYTE) ( wVal >> 8))   ///< Macro for accessing high byte @ingroup OsEci
#define LOWORD(dwVal) ((WORD) dwVal)          ///< Macro for accessing low word @ingroup OsEci
#define HIWORD(dwVal) ((WORD) ( dwVal >> 16)) ///< Macro for accessing high word @ingroup OsEci

#define OS_WAIT_FOREVER  ((DWORD)0xFFFFFFFF)  ///< Blocking function call @ingroup OsEci

// return values of OS_WaitForSingleObject
#define OS_WAIT_OBJECT_0 0UL                  ///< Wait succeeded @ingroup OsEci
#define OS_WAIT_TIMEOUT  1UL                  ///< Wait timed out @ingroup OsEci
#define OS_WAIT_FAILED   ((DWORD)0xFFFFFFFF)  ///< Wait failed @ingroup OsEci


// Definition of TRUE and FALSE
#ifndef FALSE
  #define FALSE 0 ///< FALSE @ingroup OsEci
#endif
#ifndef TRUE
  #define TRUE 1  ///< TRUE @ingroup OsEci
#endif

/** Macro to find max value */
#ifndef max
  #define max(a,b)   (((a) > (b)) ? (a) : (b))
#endif

/** Macro to find min value */
#ifndef min
  #define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

/** Macro to get element count of an array  @ingroup OsEci */
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))

/** Macro to swap the bytes of a WORD (ab -> ba)  @ingroup OsEci */
#define SWAP16(w)  ( (WORD)  ((LOBYTE(w) << 8) | HIBYTE (w)))

/** Macro to swap the bytes of a DWORD (abcd -> dcba)  @ingroup OsEci */
#define SWAP32(dw) ( (DWORD) (((SWAP16 (LOWORD (dw))) << 16) | (SWAP16 (HIWORD (dw)))))

/** Macro for printf redirection  @ingroup OsEci */
#define OS_Printf printf

/** Macro for fflush redirection  @ingroup OsEci */
#define OS_Fflush(stream) fflush(stream)

/** Printf format specifier for 64bit integer */
#if defined(__i386__)
  #define _FMT64_ "ll"
#elif defined(__amd64__)
  #define _FMT64_ "l"
#endif


///////////////////////////////////////////////////////////////////////////////
/**
  This macro retrieve a reference to the host object.

  @param t
    Type of the host object
  @param m
    Member variable which represents the inner object

  @return
   Reference to the host object.

  @note
    Set -Wno-invalid-offsetof flag in compiler to omit warning messages
    generated by this macro
*/
#ifndef HOSTOBJECT
  #define HOSTOBJECT(t,m) ((t&) *((char*) this - offsetof(t,m)))
#endif


//////////////////////////////////////////////////////////////////////////
// exported functions

//*** C-API
#ifdef __cplusplus
extern "C"
{
#endif


///////////////////////////////////////////////////////////////////////////////
/**
  Suspends the current thread for the specified time.

  @param dwMicroseconds
    Number of microseconds [us] to suspend thread

  @ingroup OsEci
*/
ECI_DLLEXPORT void  ECI_APICALL OS_uSleep ( DWORD dwMicroseconds );


///////////////////////////////////////////////////////////////////////////////
/**
  Suspends the current thread for the specified time.

  @param dwMilliseconds
    Number of milliseconds [ms] to suspend thread

  @ingroup OsEci
*/
ECI_DLLEXPORT void ECI_APICALL OS_Sleep ( DWORD dwMilliseconds );


///////////////////////////////////////////////////////////////////////////////
/**
  Sets a LONG variable to the specified value as an atomic operation.

  @param plDest
    A pointer to the value to be exchanged. The function sets this variable
    to Value, and returns its prior value.
  @param lValue
    The value to be exchanged with the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OS_InterlockedExchange ( PLONG plDest,
                                                        LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Performs an atomic addition of two LONG values.

  @param plDest
    A pointer to the variable. The value of this variable will be replaced
    with the result of the operation
  @param lValue
    The value to be added to the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OS_InterlockedExchangeAdd( PLONG plDest,
                                                          LONG  lValue );


///////////////////////////////////////////////////////////////////////////////
/**
  Performs an atomic OR operation on the specified LONG values. The function 
  prevents more than one thread from using the same variable simultaneously.

  @param plDest
    A pointer to the first operand. This value will be ORed with lValue.
  @param lValue
    The value to be ORed to the value pointed to by plDest.

  @retval LONG
    The function returns the initial value of the plDest parameter.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OS_InterlockedOr( PLONG plDest,
                                                 LONG  lValue );

                                                 
///////////////////////////////////////////////////////////////////////////////
/**
  Increments the value of the specified LONG variable as an atomic operation

  @param plDest
    A pointer to the variable to be incremented.

  @retval LONG
    The function returns the resulting incremented value.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OS_InterlockedIncrement( PLONG plDest );


///////////////////////////////////////////////////////////////////////////////
/**
  Decrements the value of the specified LONG variable as an atomic operation

  @param plDest
    A pointer to the variable to be decremented.

  @retval LONG
    The function returns the resulting decremented value.

  @ingroup OsEci
*/
ECI_DLLEXPORT LONG ECI_APICALL OS_InterlockedDecrement( PLONG plDest );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current time in microseconds [us].

  @retval DWORD
    Current time in microseconds [us].

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_GetTimeInUs ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current time in milliseconds [ms].

  @retval DWORD
    Current time in milliseconds [ms].

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_GetTimeInMs ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new unnamed event. The event is initially not signaled and auto 
  reset.

  @retval HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OS_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT HANDLE ECI_APICALL OS_CreateEvent ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Set the state of the specified event object to signaled

  @param hEvent
    Event handle to set

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OS_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_SetEvent ( HANDLE hEvent );


///////////////////////////////////////////////////////////////////////////////
/**
  Wait until either the specified object is in the signaled state, or until
  the time-out interval elapses.

  @param hEvent
    Handle to wait for get signaled
  @param dwTimeout
    Specifies the time-out interval, in milliseconds [ms]. If the interval
    elapses, the function returns, even if the object's state is non signaled.

  @retval DWORD
    @ref OS_WAIT_OBJECT_0
      Success. The specified object’s state is signaled. @n
    @ref OS_WAIT_TIMEOUT
      Failure. The time-out interval elapsed, and the object's state is
      non signaled. @n
    @ref OS_WAIT_FAILED
      Failure. Waiting failed maybe due to an invalid handle. To get extended
      error information, call @ref OS_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_WaitForSingleObject ( HANDLE hEvent,
                                                         DWORD  dwTimeout );


///////////////////////////////////////////////////////////////////////////////
/**
  Creates a new unnamed mutex which can be uses for mutual exclusion

  @retval HANDLE
    If the function succeeds, the return value is a handle to the event
    object. If the function fails, the return value is NULL.
    To get extended error information, call @ref OS_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT HANDLE ECI_APICALL OS_CreateMutex ( void );


///////////////////////////////////////////////////////////////////////////////
/**
  Acquires a mutual lock

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OS_GetLastError.

  @note
    This mutex is not re-entrant. Trying to acquire the mutex from the
    same thread a second time will lead to a dead-lock situation!

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_LockMutex ( HANDLE hMutex );


///////////////////////////////////////////////////////////////////////////////
/**
  Release a mutual lock.

  @param hMutex
    Mutex to acquire

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OS_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_UnlockMutex ( HANDLE hMutex );


///////////////////////////////////////////////////////////////////////////////
/**
  Closes a handle and deletes the object.

  @param hHandle
    Handle to close

  @retval DWORD
    If the function succeeds, the return value is TRUE.
    If the function fails, the return value is FALSE.
    To get extended error information, call @ref OS_GetLastError.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_CloseHandle ( HANDLE hHandle );


///////////////////////////////////////////////////////////////////////////////
/**
  Returns the current thread's last exception code.

  @retval DWORD
    The calling thread’s last-error code value.

  @ingroup OsEci
*/
ECI_DLLEXPORT DWORD ECI_APICALL OS_GetLastError ( void );


#ifdef __cplusplus
}
#endif // __cplusplus


#endif //__OSECI_H__
