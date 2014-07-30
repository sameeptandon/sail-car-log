#ifndef RTXT_CONVDATA_H
#define RTXT_CONVDATA_H

//#define BIG_ENDIAN    1
//#define LITTLE_ENDIAN     2
//#define SIGNED      1
//#define UNSIGNED    2

union U16_DATA_TYPE {
  unsigned short WORD;
  struct {
    unsigned char BYTE0;
    unsigned char BYTE1;
  } BYTE;
};

union U32_DATA_TYPE {
  unsigned long DWORD;
  struct {
    unsigned char BYTE0;
    unsigned char BYTE1;
    unsigned char BYTE2;
    unsigned char BYTE3;
  } BYTE;
};

struct CAN_DATASET {
  unsigned char BYTE0;
  unsigned char BYTE1;
  unsigned char BYTE2;
  unsigned char BYTE3;
  unsigned char BYTE4;
  unsigned char BYTE5;
  unsigned char BYTE6;
  unsigned char BYTE7;
};

struct CanId730
{
  unsigned  yr1  : 16;
  unsigned  yr2  : 16;
  unsigned  yr3  : 16;
  unsigned  yr4  : 16;
};

struct CanId731
{
  unsigned  yr5  : 16;
  unsigned  yr6  : 16;
  unsigned  yr7  : 16;
  unsigned  yr8  : 16;
};

struct CanId732
{
  unsigned  yr9  : 16;
  unsigned  yr10  : 16;
  unsigned  delay  : 12;
  unsigned  Reserve1  : 4;
  unsigned  confidence  : 8;
  unsigned  msg_cnt  : 4;
};

struct CanId733
{
  unsigned  xr1  : 16;
  unsigned  xr2  : 16;
  unsigned  xr3  : 16;
  unsigned  xr4  : 16;
};

struct CanId734
{
  unsigned  xr5  : 16;
  unsigned  xr6  : 16;
  unsigned  xr7  : 16;
  unsigned  xr8  : 16;
};

struct CanId735
{
  unsigned  xr9  : 16;
  unsigned  xr10  : 16;
};

union CAN_DATA_FORM
{
  unsigned char  bData[8];
  CAN_DATASET DataSet;  
  
  //-------------------------------------------------//
  //  Local-CAN 2                                     //
  //-------------------------------------------------//
  CanId730 ID_730;
  CanId731 ID_731;
  CanId732 ID_732;
  CanId733 ID_733;
  CanId734 ID_734;
  CanId735 ID_735;

};

#endif
// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
