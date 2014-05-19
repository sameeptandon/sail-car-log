// define GPS class
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define BufferLength 500

struct GPGGAData{
  double UTC_Time;  // UTC time
  double Latitude;  // Latitude[deg]
  bool NS_ind;     // North or South
  double Longitude; // Longitude[deg]
  bool EW_ind;     // East or West
  
  // GPS Quality indicator
  // 0: Fix not valid
  // 1: GPS fix
  // 2: Differential GPS fix
  // 4: Real-Time Kinematic, fixed integers
  // 5: Real-Time Kinematic, float integers
  int Pos_Fix_ind;
  int Num_of_Satellite; // Number of SVs in use
  double HDOP;   // HDOP
  double Altitude; // Altitude 
};

struct GPHDTData{
  double Heading;  // Heading[deg]
  bool TF_ind;    // True North or not
};

class CGPSCom{
public:
  GPGGAData GPGGA;
  GPHDTData GPHDT;

  CGPSCom();
  ~CGPSCom();
  bool Connect(char* server, short port);
  bool ReceiveGPS();


private:
  struct sockaddr_in serveraddr;
  struct hostent *hostp;
  int sd; // socket

  void decode_GPGGA(int startByte, int endByte, char* buf);
  void decode_GPHDT(int startByte, int endByte, char* buf);
  int SearchDevByte(int startByte, int endByte, char* buf);
  void CopyChar(int startByte, int endByte, char* buf, char* target);

};
