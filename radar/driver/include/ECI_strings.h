///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2012 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Utility function to convert ECI defines to human readable string.

  @file ECI_strings.h
*/

#ifndef __ECI_STRINGS_H__
#define __ECI_STRINGS_H__

//////////////////////////////////////////////////////////////////////////
// include files
#include <OsEci.h>

//////////////////////////////////////////////////////////////////////////
// constants and macros


//////////////////////////////////////////////////////////////////////////
// functions


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable string from a e_HWCLASS define.

  @param dwHwClass
    Numeric value of e_HWCLASS to convert to a human readable string.

  @retval Sting
    Human readable string.

  @ingroup EciStrings
*/
const char* ECI_StrFromHwClass(DWORD dwHwClass)
{
  const char* pszString = NULL;

  //*** Switch on define
  switch(dwHwClass)
  {
    case ECI_HW_UNDEFINED:
      pszString = "undefined";
    break;

    case ECI_HW_PCI:
      pszString = "PCI";
    break;

    case ECI_HW_ISA:
      pszString = "ISA";
    break;

    case ECI_HW_USB:
      pszString = "USB";
    break;

    case ECI_HW_IP:
      pszString = "Ethernet";
    break;

    default:
      pszString = "n/a";
    break;
  }//end switch

  return pszString;
}


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable string from a e_CTRLCLASS define.

  @param dwCtrlClass
    Numeric value of e_CTRLCLASS to convert to a human readable string.

  @retval Sting
    Human readable string.

  @ingroup EciStrings
*/
const char* ECI_StrFromCtrlClass(DWORD dwCtrlClass)
{
  const char* pszString = NULL;

  //*** Switch on define
  switch(dwCtrlClass)
  {
    case ECI_CTRL_UNDEFINED:
      pszString = "undefined";
    break;

    case ECI_CTRL_CAN:
      pszString = "CAN";
    break;

    case ECI_CTRL_LIN:
      pszString = "LIN";
    break;

    case ECI_CTRL_FLX:
      pszString = "FlexRay";
    break;

    case ECI_CTRL_KLI:
      pszString = "K-Line";
    break;

    default:
      pszString = "n/a";
    break;
  }//end switch

  return pszString;
}


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable string from a e_CANCTRLCLASS define.

  @param dwCanCtrlClass
    Numeric value of e_CANCTRLCLASS to convert to a human readable string.

  @retval Sting
    Human readable string.

  @ingroup EciStrings
*/
const char* ECI_StrFromCanCtrlClass(DWORD dwCanCtrlClass)
{
  const char* pszString = NULL;

  //*** Switch on define
  switch(dwCanCtrlClass)
  {
    case ECI_CAN_CTRL_UNKNOWN:
      pszString = "undefined";
    break;

    case ECI_CAN_CTRL_82527:
      pszString = "Intel 82527";
    break;

    case ECI_CAN_CTRL_82C200:
      pszString = "Intel 82C200";
    break;

    case ECI_CAN_CTRL_81C90:
      pszString = "Intel 81C90";
    break;

    case ECI_CAN_CTRL_81C92:
      pszString = "Intel 81C92";
    break;

    case ECI_CAN_CTRL_SJA1000:
      pszString = "Philips SJA 1000";
    break;

    case ECI_CAN_CTRL_82C900:
      pszString = "Infineon 82C900 (TwinCAN)";
    break;

    case ECI_CAN_CTRL_TOUCAN:
      pszString = "Motorola TOUCAN";
    break;

    case ECI_CAN_CTRL_MSCAN:
      pszString = "Freescale Star12 MSCAN";
    break;

    case ECI_CAN_CTRL_FLEXCAN:
      pszString = "Freescale Coldfire FLEXCAN";
    break;

    case ECI_CAN_CTRL_IFI:
      pszString = "IFI CAN (ALTERA FPGA CAN)";
    break;

    default:
      pszString = "n/a";
    break;
  }//end switch

  return pszString;
}


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable string from a e_CANBUSC define.

  @param dwCanBusCoupling
    Numeric value of e_CANBUSC to convert to a human readable string.

  @retval Sting
    Human readable string.

  @ingroup EciStrings
*/
const char* ECI_StrFromCanBusCoupling(DWORD dwCanBusCoupling)
{
  const char* pszString = NULL;

  //*** Switch on define
  switch(dwCanBusCoupling)
  {
    case ECI_CAN_BUSC_UNDEFINED:
      pszString = "undefined";
    break;

    case ECI_CAN_BUSC_LOWSPEED:
      pszString = "Low Speed (ISO/IS 11898-3)";
    break;

    case ECI_CAN_BUSC_HIGHSPEED:
      pszString = "High Speed (ISO/IS 11898-2)";
    break;

    case ECI_CAN_BUSC_HIGHSPEED | ECI_CAN_BUSC_LOWSPEED:
      pszString = "High or Low Speed (ISO/IS 11898-2 or 11898-3)";
    break;

    default:
      pszString = "n/a";
    break;
  }//end switch

  return pszString;
}


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable string from a e_LINCTRLCLASS define.

  @param dwLinCtrlClass
    Numeric value of e_LINCTRLCLASS to convert to a human readable string.

  @retval Sting
    Human readable string.

  @ingroup EciStrings
*/
const char* ECI_StrFromLinCtrlClass(DWORD dwLinCtrlClass)
{
  const char* pszString = NULL;

  //*** Switch on define
  switch(dwLinCtrlClass)
  {
    case ECI_LIN_CTRL_UNKNOWN:
      pszString = "undefined";
    break;

    case ECI_LIN_CTRL_GENERIC:
      pszString = "Generic";
    break;

    default:
      pszString = "n/a";
    break;
  }//end switch

  return pszString;
}


///////////////////////////////////////////////////////////////////////////////
/**
  Function to retrieve a human readable string from a e_LINBUSC define.

  @param dwLinBusCoupling
    Numeric value of e_LINBUSC to convert to a human readable string.

  @retval Sting
    Human readable string.

  @ingroup EciStrings
*/
const char* ECI_StrFromLinBusCoupling(DWORD dwLinBusCoupling)
{
  const char* pszString = NULL;

  //*** Switch on define
  switch(dwLinBusCoupling)
  {
    case ECI_LIN_BUSC_UNDEFINED:
      pszString = "undefined";
    break;

    case ECI_LIN_BUSC_STANDARD:
      pszString = "Standard";
    break;

    default:
      pszString = "n/a";
    break;
  }//end switch

  return pszString;
}


#endif
