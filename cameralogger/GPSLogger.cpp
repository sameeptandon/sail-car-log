#include <boost/algorithm/string/predicate.hpp>
#include "GPSLogger.h"

using namespace std;
using namespace serial;

#define PACKET_HEADER "#MARK1PVAA"
#define RETRY_ATTEMPTS 2

void GPSLogger::safeWrite(string cmd) {
    cout << "Command: " << cmd << endl; 
    _port->write(cmd);
    _port->flushOutput();
    sleep(1);
    cout << safeRead() << endl; 
    cout << safeRead() << endl; // show acknowledgement 
}

string GPSLogger::getPacket() {
    for (int i = 0; i < RETRY_ATTEMPTS; i++) {
        string candidatePacket = safeRead();
        if (boost::contains(candidatePacket, PACKET_HEADER)) {
            return candidatePacket;
        }
    }

    return "INVALID PACKET";
}

string GPSLogger::safeRead() {
    string buf = _port->readline();
    return buf;
}

void GPSLogger::Connect(string port) {
    int baudrate = 115200;
    Timeout my_timeout(10,10,0,10,0); // 1 second read and write timeout
    cout << "port = " << port << endl; 
    _port = new Serial(port, baudrate, my_timeout);

    // flush stuff from previous runs
    _port->flushInput();
    _port->flushOutput();
    _port->readlines();

    safeWrite("unlogall\r\n");
    safeWrite("fix none\r\n");
    //safeWrite(_port, "fix position 37.123 -123.32 20.20\r\n");
    //safeWrite(_port, "saveconfig\r\n");
    //safeWrite(_port, "eventoutcontrol mark1 enable positive 10000000 10000000\r\n");
    //safeWrite(_port, "eventoutcontrol mark1 enable positive 250000000 250000000\r\n");
    safeWrite("eventincontrol mark1 event\r\n");
    //safeWrite(_port, "setupsensor sensor1 mark1 positive 100 mark1 event positive 0 10\r\n"); 
    //safeWrite("log mark1time onnew\r\n");
    safeWrite("log mark1pvaa onnew\r\n");
    //safeWrite(port, "log usb1 bestposa ontime 0.5 0 nohold\r\n");
}

void GPSLogger::Run() {
    safeWrite("eventoutcontrol mark2 enable positive 10000000 10000000\r\n");
    //safeWrite("eventoutcontrol mark2 enable positive 250000000 250000000\r\n");
}

void GPSLogger::Close() {
    safeWrite("unlogall\r\n");
    safeWrite("eventoutcontrol mark2 disable\r\n");
    _port->flush();
    // hack since flush doesn't do it:
    size_t buf_size = _port->available();
    uint8_t* buf = (uint8_t*) malloc(sizeof(uint8_t)*buf_size);
    _port->read(buf, buf_size);
    free(buf);
    _port->close();
    delete _port; 
}

