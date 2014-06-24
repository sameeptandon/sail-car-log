//--------------------------------------------------------------//
// CAN Communication class for USB CAN  made by Kvaser corp. //
//--------------------------------------------------------------//
#include "RXTX_CANCom.h"

#include <canlib.h>

#include <iostream>
#include <cctype>
#include <cstring>


#define Deg2Rad 3.141592653578/180

//=====================================//
// Constructor             //
//=====================================//
CCANCom::CCANCom() :
  m_channel(-1),
  m_can_handle(canERR_NOTINITIALIZED),
  m_rx_id(-1),
  m_tx_flg(canMSG_STD)
{
  canInitializeLibrary();
}

//=====================================//
// Destructor                   //
//=====================================//
CCANCom::~CCANCom()
{
  disconnect();
}

// CAN connection
bool CCANCom::connect(int chNum)
{
  bool FLG = false;
  // Port initialize
  //CANHandle = canOpenChannel(ChNum,canOPEN_EXCLUSIVE);
  //m_canHandle = canOpenChannel(ChNum, canWANT_EXCLUSIVE);
  m_can_handle = canOpenChannel(chNum, canOPEN_ACCEPT_VIRTUAL);

  if ( m_can_handle >= canOK ) {
    // Initialization
    if ( canSetBusParams(m_can_handle, BAUD_500K, 0, 0, 0, 0, 0) == canOK ) {
      if ( canBusOn(m_can_handle) == canOK ) {
        FLG = true;
      }
    }
  }

  if (FLG == false) {
    m_can_handle = canERR_NOTINITIALIZED;
  }
  
  return FLG;
}

// CAN disconnect
bool CCANCom::disconnect()
{
  bool FLG = false;
  if (m_can_handle >= canOK) {
     if ( canClose(m_can_handle) == canOK ) {
       FLG         = true;
     }
  }
  
  m_channel = -1;
  m_can_handle = canERR_NOTINITIALIZED;
  return FLG;
}



//=====================================//
// CAN receive                         //
//=====================================//
bool CCANCom::receive()
{
  bool FLG = false;
  int stat;

  // CAN message receive ( Wait until message receiving )
  stat = canReadWait(m_can_handle, &m_rx_id, RX_Msg, &RX_DLC, &RX_Flg, &m_rx_time, 1000);

  if ( stat == canOK ) {
    FLG   = true;
  }

  return FLG;
}

//=====================================//
// CAN send                             //
//=====================================//
bool CCANCom::transmit(int id, int dlc, unsigned char bData[])
{
  bool FLG = true;
 
  // Message sending
  return (canWrite(m_can_handle, id, bData, dlc, m_tx_flg) == canOK);
    

  // CAN err detect
  // ??? is this the way to detect error?
  // spec says differently
  if ( bData[0] != canOK ) {
    FLG = false;
  }

  return FLG;
}

void CCANCom::hexDump(std::ostream& ostr, const void* addr, size_t len)
{
  unsigned char const * pc = (unsigned char const*)addr;
  char buff[17];
  
  // Process every byte in the data.
  ostr << std::hex;
  size_t i;
  for (i = 0; i < len; ++i) {
    // Multiple of 16 means new line (with line offset).
    
    if ((i % 16) == 0) {
      // Just don't print ASCII for the zeroth line.
      if (i != 0) {
        ostr << " " << buff << std::endl;
      }
      // Output the offset.
      ostr.fill('0');
      ostr.width(4);
      ostr << i << "  ";
      
      // clear the char buffer
      memset(buff, 0, sizeof(buff));
    }
    
    if (((i + 8) % 16) == 0) {
      ostr << " ";
    }
    // Now the hex code for the specific character.
    ostr.fill('0');
    ostr.width(2);
    ostr << (unsigned int) pc[i] << " ";
    
    // And store a printable ASCII character for later.
    buff[i % 16] = isprint(pc[i]) && !isspace(pc[i])? pc[i] : '.';
  }
  
  // And print the final ASCII bit.
  for (; (i % 16) != 0; ++i) {
    ostr << "   ";
    if (((i + 8) % 16) == 0) {
      ostr << " ";
    }
  }
  ostr << " " << buff << std::endl;
  ostr << std::dec;
}

// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
