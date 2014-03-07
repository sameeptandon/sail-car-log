///////////////////////////////////////////////////////////////////////////////
// (C) 2008-2011 IXXAT Automation GmbH, all rights reserved
///////////////////////////////////////////////////////////////////////////////
/**
  Definitions for alignment of structures.

  @file ECI_align.h
*/

#ifdef __GNUC__
  #define __ALIGN(x) __attribute__((aligned(x)))
#endif

#ifdef _MSC_VER
  #define __ALIGN(x) __declspec(align(x))

  // Disable warning C4324: structure was padded due to __declspec(align())
  #pragma warning( disable : 4324 )
#endif

