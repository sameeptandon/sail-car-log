#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "trimble_gps.hpp"

using namespace std;

CGPSCom::CGPSCom(){

  // init GPGGA
  GPGGA.Altitude = 0.0;
  GPGGA.Latitude = 0.0;
  GPGGA.NS_ind = false;
  GPGGA.Longitude = 0.0;
  GPGGA.EW_ind = false;
  GPGGA.Pos_Fix_ind = 0;
  GPGGA.Num_of_Satellite = 0;
  GPGGA.HDOP = 0.0;
  GPGGA.Altitude = 0.0;

  // init GPHDT
  GPHDT.Heading = 0.0;
  GPHDT.TF_ind = false;

}

CGPSCom::~CGPSCom(){
  close(sd); // close socket
}

bool CGPSCom::Connect(char* server, short port)
{
  // socket open
  if((sd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
    return false;
  }
  
  memset(&serveraddr, 0x00, sizeof(struct sockaddr_in));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_port = htons(port);

  if((serveraddr.sin_addr.s_addr = inet_addr(server)) == (unsigned long)INADDR_NONE){
    hostp = gethostbyname(server);
    if(hostp == (struct hostent *)NULL){
      return false;
    }
    memcpy(&serveraddr.sin_addr, hostp->h_addr, sizeof(serveraddr.sin_addr));
  }

  // connect
  if(connect(sd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0){
    close(sd);
    return false;
  }

  return true;

}

bool CGPSCom::ReceiveGPS()
{
  char buffer[BufferLength]={0};

  // read data
  int rc = read(sd, &buffer[0], BufferLength);
  if(rc < 0){
    close(sd);
    return false;
  } else if (rc == 0){   // server program has issued a close()
    close(sd);
    return false;
  }
  //cout << "data: " << buffer << endl;

  // decode data
  for (int i=0;i<BufferLength;i++){
    if(buffer[i] == 0x24){ // 0x24 = $
      // GPGGA
      if( buffer[i+1] == 0x47	// 0x47 = G
	  && buffer[i+2] == 0x50	// 0x50 = P
	  && buffer[i+3] == 0x47	// 0x47 = G
	  && buffer[i+4] == 0x47	// 0x47 = G
	  && buffer[i+5] == 0x41	// 0x41 = A
	  ){
	for(int j=i+1;j<BufferLength;j++){
	  if(buffer[j]==0x0d){ // 0X0d = [CR]
	    decode_GPGGA(i+7,j,buffer);
	    break;
	  }
	}
      }
      
      
      // GPHDT
      if( buffer[i+1] == 0x47	// 0x47 = G
	  && buffer[i+2] == 0x50	// 0x50 = P
	  && buffer[i+3] == 0x48	// 0x48 = H
	  && buffer[i+4] == 0x44	// 0x44 = D
	  && buffer[i+5] == 0x54	// 0x54 = T
	  ){
	for(int j=i+1;j<BufferLength;j++){
	  if(buffer[j]==0x0d){ // 0X0d = [CR]
	    decode_GPHDT(i+7,j,buffer);
	    break;
	  }
	}
      }
      
    }
  }
  //delete buffer;
  return true;
}

void CGPSCom::decode_GPGGA(int startByte, int endByte, char* buf)
{
  int start = startByte;
  int devByte;

  // decode
  char tmp[20]={0};

  // 1.UTC of position fix
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.UTC_Time = (double)atof(tmp);
  start = devByte+2;

  // 2.Latitude : ddmm.mmmm (d:degree, m:minute)
  devByte = start+1;
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Latitude = (double)atof(tmp); // dd
  start = devByte+1;
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Latitude += (double)atof(tmp)/60.0;
  start = devByte+2;
  
  // 3.Direction of Latitude
  if(buf[start] == 0x4e) // 0x4e = N
    GPGGA.NS_ind = true; // North
  else
    GPGGA.NS_ind = false; // South
  start = start+2;

  // 4.Longitude : dddmm.mmmm
  devByte = start+2;
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Longitude = (double)atof(tmp); // ddd
  start = devByte+1;
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Longitude += (double)atof(tmp)/60.0;
  start = devByte+2;  

  // 5.Direction of Longitude
  if(buf[start] == 0x45) // 0x45 = E
    GPGGA.EW_ind = true; // East
  else
    GPGGA.EW_ind = false; // West
  start = start+2;

  // 6.GPS Quality indicator
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Pos_Fix_ind = atoi(tmp);
  start = devByte+2;

  // 7.Number of satellites in fix
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Num_of_Satellite = atoi(tmp);
  start = devByte+2;

  // 8.HDOP
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.HDOP = (double)atof(tmp);
  start = devByte+2;

  // 9.Orthometric height
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPGGA.Altitude = (double)atof(tmp);

}

void CGPSCom::decode_GPHDT(int startByte, int endByte, char* buf)
{
  int start = startByte;
  int devByte;

  // decode
  char tmp[20]={0};

  // 1. Heading
  devByte = SearchDevByte(start,endByte,buf);
  CopyChar(start,devByte,buf,tmp);
  GPHDT.Heading = (double)atof(tmp);
  start = devByte+2;

  // 2. True North or not
  if(buf[start] == 0x54) // 0x54 = E
    GPHDT.TF_ind = true; // True North
  else
    GPHDT.TF_ind = false; // False

}

int CGPSCom::SearchDevByte(int startByte, int endByte, char* buf)
{
  int devByte = endByte;
  for(int i=startByte;i<endByte;i++){
    if(buf[i] == 0x2c){ // 0x2c = ","
      devByte = i-1;
      break;
    }
  }
  return devByte;
}

void CGPSCom::CopyChar(int startByte, int endByte, char* buf, char* target)
{
  int index = 0;
  for(int i=0;i<20;i++) target[i]=0;
  for(int j=startByte;j<=endByte;j++){
    target[index] = buf[j];
    index++;
  }

}
