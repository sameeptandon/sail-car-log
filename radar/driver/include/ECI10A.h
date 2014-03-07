///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2010 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  ECI API function redefintions for IXXAT Interface:  USB-to-CAN II.
  
  Headerfile of the ECI ( embedded / realtime communication interface), 
  a generic library for IXXAT hardware interfaces.

  @file ECI10A.h
*/

#ifndef __ECI10A_H__
#define __ECI10A_H__

//*** At first undefine maybe previously defined ECI export functions
#undef ECIDRV_                     
#undef ECIDRV_Initialize           
#undef ECIDRV_Release              
#undef ECIDRV_GetInfo              
#undef ECIDRV_CtrlOpen             
#undef ECIDRV_CtrlClose            
#undef ECIDRV_CtrlStart            
#undef ECIDRV_CtrlStop             
#undef ECIDRV_CtrlGetCapabilities  
#undef ECIDRV_CtrlGetStatus        
#undef ECIDRV_CtrlSend             
#undef ECIDRV_CtrlReceive          
#undef ECIDRV_CtrlSetFilter        
#undef ECIDRV_CtrlCommand          
#undef ECIDRV_LogConfig            
#undef ECIDRV_LogRead              
#undef ECIDRV_LogStart             
#undef ECIDRV_LogStop              
#undef ECIDRV_LogClear             
#undef ECIDRV_GetErrorString       


//*** Define ECI export functions for defined hardware type
#define ECIDRV_                     ECI10A_                   
#define ECIDRV_Initialize           ECI10A_Initialize         
#define ECIDRV_Release              ECI10A_Release            
#define ECIDRV_GetInfo              ECI10A_GetInfo            
#define ECIDRV_CtrlOpen             ECI10A_CtrlOpen           
#define ECIDRV_CtrlClose            ECI10A_CtrlClose          
#define ECIDRV_CtrlStart            ECI10A_CtrlStart          
#define ECIDRV_CtrlStop             ECI10A_CtrlStop           
#define ECIDRV_CtrlGetCapabilities  ECI10A_CtrlGetCapabilities
#define ECIDRV_CtrlGetStatus        ECI10A_CtrlGetStatus      
#define ECIDRV_CtrlSend             ECI10A_CtrlSend           
#define ECIDRV_CtrlReceive          ECI10A_CtrlReceive        
#define ECIDRV_CtrlSetFilter        ECI10A_CtrlSetFilter      
#define ECIDRV_CtrlCommand          ECI10A_CtrlCommand        
#define ECIDRV_LogConfig            ECI10A_LogConfig          
#define ECIDRV_LogRead              ECI10A_LogRead            
#define ECIDRV_LogStart             ECI10A_LogStart           
#define ECIDRV_LogStop              ECI10A_LogStop            
#define ECIDRV_LogClear             ECI10A_LogClear           
#define ECIDRV_GetErrorString       ECI10A_GetErrorString     


/** @def ECIDRV_
  General ECI API function renaming */
/** @def ECIDRV_Initialize 
  @brief @copybrief ECIDRV_Initialize @n <b> See function:</b> @ref ECIDRV_Initialize */
/** @def ECIDRV_Release 
  @brief @copybrief ECIDRV_Release @n <b> See function:</b> @ref ECIDRV_Release */
/** @def ECIDRV_GetInfo 
  @brief @copybrief ECIDRV_GetInfo @n <b> See function:</b> @ref ECIDRV_GetInfo */
/** @def ECIDRV_CtrlOpen 
  @brief @copybrief ECIDRV_CtrlOpen @n <b> See function:</b> @ref ECIDRV_CtrlOpen */
/** @def ECIDRV_CtrlClose 
  @brief @copybrief ECIDRV_CtrlClose @n <b> See function:</b> @ref ECIDRV_CtrlClose */
/** @def ECIDRV_CtrlStart 
  @brief @copybrief ECIDRV_CtrlStart @n <b> See function:</b> @ref ECIDRV_CtrlStart */
/** @def ECIDRV_CtrlStop 
  @brief @copybrief ECIDRV_CtrlStop @n <b> See function:</b> @ref ECIDRV_CtrlStop */
/** @def ECIDRV_CtrlGetCapabilities 
  @brief @copybrief ECIDRV_CtrlGetCapabilities @n <b> See function:</b> @ref ECIDRV_CtrlGetCapabilities */
/** @def ECIDRV_CtrlGetStatus 
  @brief @copybrief ECIDRV_CtrlGetStatus @n <b> See function:</b> @ref ECIDRV_CtrlGetStatus */
/** @def ECIDRV_CtrlSend 
  @brief @copybrief ECIDRV_CtrlSend @n <b> See function:</b> @ref ECIDRV_CtrlSend */
/** @def ECIDRV_CtrlReceive 
  @brief @copybrief ECIDRV_CtrlReceive @n <b> See function:</b> @ref ECIDRV_CtrlReceive */
/** @def ECIDRV_CtrlSetFilter 
  @brief @copybrief ECIDRV_CtrlSetFilter @n <b> See function:</b> @ref ECIDRV_CtrlSetFilter */
/** @def ECIDRV_CtrlCommand 
  @brief @copybrief ECIDRV_CtrlCommand @n <b> See function:</b> @ref ECIDRV_CtrlCommand */
/** @def ECIDRV_LogConfig 
  @brief @copybrief ECIDRV_LogConfig @n <b> See function:</b> @ref ECIDRV_LogConfig */
/** @def ECIDRV_LogRead 
  @brief @copybrief ECIDRV_LogRead @n <b> See function:</b> @ref ECIDRV_LogRead */
/** @def ECIDRV_LogStart 
  @brief @copybrief ECIDRV_LogStart @n <b> See function:</b> @ref ECIDRV_LogStart */
/** @def ECIDRV_LogStop 
  @brief @copybrief ECIDRV_LogStop @n <b> See function:</b> @ref ECIDRV_LogStop */
/** @def ECIDRV_LogClear 
  @brief @copybrief ECIDRV_LogClear @n <b> See function:</b> @ref ECIDRV_LogClear */
/** @def ECIDRV_GetErrorString 
  @brief @copybrief ECIDRV_GetErrorString @n <b> See function:</b> @ref ECIDRV_GetErrorString */


//////////////////////////////////////////////////////////////////////////
// include files
#include <ECI.h>


#endif //__ECI10A_H__
