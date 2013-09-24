#include <boost/algorithm/string/predicate.hpp>
#include "GPSLogger.h"

using namespace std;
using namespace serial;

#define PACKET_HEADER "#MARK1PVAA"
#define RETRY_ATTEMPTS 3

void GPSLogger::safeWrite(string cmd) {
    cout << "Command: " << cmd; 
    _port->write(cmd);
    _port->flushOutput();
    sleep(1);
    GPSPacketType ack_1 = getPacket();
    GPSPacketType ack_2 = getPacket();

    cout << boost::get<1>(ack_1); 
    cout << boost::get<1>(ack_2); // show acknowledgement 
    cout << endl; 
}

GPSLogger::GPSPacketType GPSLogger::getPacket() {
    string candidatePacket; 
    for (int i = 0; i < RETRY_ATTEMPTS; i++) {
        candidatePacket = safeRead();
        //cout << candidatePacket << endl; 
        if (boost::contains(candidatePacket, PACKET_HEADER)) {
            return GPSPacketType(1,candidatePacket);
        }
    }

    return GPSPacketType(0, candidatePacket);
}

string GPSLogger::safeRead() {
    string ret = _port->readline();
    return ret;
}

void GPSLogger::Connect(string port) {
    int baudrate = 115200;
    Timeout my_timeout(6,6,0,6,0); // 6 millisecond read and write timeout
    cout << "port = " << port << endl; 
    _port = new Serial(port, baudrate, my_timeout);

    // reset stuff
    safeWrite("unlogall\r\n");
    safeWrite("fix none\r\n");
    safeWrite("eventincontrol mark1 event\r\n");
    
    // flush stuff from previous runs
    //_port->flushInput();
    //_port->flushOutput();
    //_port->readlines();
    
    ///////// logging /////////////////
    //safeWrite("log mark1time onnew\r\n");
    safeWrite("log mark1pvaa onnew\r\n");
    //safeWrite(port, "log usb1 bestposa ontime 0.5 0 nohold\r\n");
}

void GPSLogger::Run() {
    safeWrite("eventoutcontrol mark2 enable positive 10000000 10000000\r\n");
    //safeWrite("eventoutcontrol mark2 enable positive 250000000 250000000\r\n");
}

void GPSLogger::Close() {
    safeWrite("eventoutcontrol mark2 disable\r\n");
    safeWrite("unlogall\r\n");
    cout << "flushing GPS buffers" << endl;
    _port->flush();
    sleep(1);
    // hack since flush doesn't do it:
    size_t buf_size = _port->available();
    cout << "bytes in buffer after flush: " << buf_size << endl; 
    uint8_t* buf = (uint8_t*) malloc(sizeof(uint8_t)*buf_size);
    _port->read(buf, buf_size);
    free(buf);
    buf_size = _port->available();
    cout << "bytes in buffer after force clean: " << buf_size << endl; 
    _port->close();
    delete _port; 
}

