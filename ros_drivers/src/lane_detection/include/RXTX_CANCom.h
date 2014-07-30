#ifndef CCANCom_h
#define CCANCom_h

#include <canlib.h>

#include <iosfwd>

//--------------------------------------------------------------//
// CAN Communication class for CAN card made by Kvaser corp.    //
//--------------------------------------------------------------//
class CCANCom
{
public:
  CCANCom();
  ~CCANCom();
  
  bool connect(int chNum);    // CAN connect( 0 or 1 )
  bool disconnect();      // CAN disconnect
  int channel() const {
    return m_channel;
  }
  bool isConnected() const {
    return m_can_handle >= canOK;
  }
  
  bool receive();
  bool transmit(int id, int dlc, unsigned char bData[]);
  int getCanMsgId() const {
    return m_rx_id;
  }
  unsigned char* getCanMsgData() {
    return RX_Msg;
  }
    
  static void hexDump(std::ostream& ostr, void const * addr, size_t len);
  
  //-------------------------------------//
  // Setting                             //
  //-------------------------------------//
protected:
  int m_channel;
  CanHandle m_can_handle;
  
  //-------------------------------------//
  // RX / TX                             //
  //-------------------------------------//
  long m_rx_id;
  unsigned char RX_Msg[8];
  unsigned int RX_DLC;
  unsigned int RX_Flg, m_tx_flg;
  unsigned long m_rx_time;

};
#endif // #define CCANCom_h

// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
