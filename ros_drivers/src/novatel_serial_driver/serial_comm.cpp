#include <boost/algorithm/string/predicate.hpp>
#include "serial_comm.h"

using namespace std;
using namespace serial;

void serial_comm::safeWrite(string cmd) {
    cout << "Command: " << cmd; 
    _port->write(cmd);
    _port->flushOutput();
    //sleep(1);
    usleep(0.1 * 1000 * 1000);
}

string serial_comm::safeRead() {
    string ret = _port->readline();
    return ret;
}

void serial_comm::Connect(string port) {
    int baudrate = 115200;
    Timeout my_timeout(6,6,0,6,0); // 6 millisecond read and write timeout
    cout << "port = " << port << endl; 
    _port = new Serial(port, baudrate, my_timeout);

    // reset stuff
    safeWrite("unlogall\r\n");
    safeWrite("fix none\r\n");
    safeWrite("eventincontrol mark1 event\r\n");
    
    
    ///////// logging /////////////////
    //safeWrite("log mark1time onnew\r\n");
    safeWrite("log mark1pvaa onnew\r\n");
    safeWrite("log inscovs ontime 1 0 nohold\r\n");
    safeWrite("log gpgst ontime 1 0 nohold\r\n");
    //safeWrite(port, "log usb1 bestposa ontime 0.5 0 nohold\r\n");
}

/*
void serial_comm::Run() {
    safeWrite("eventoutcontrol mark2 enable positive 10000000 10000000\r\n");
    //safeWrite("eventoutcontrol mark2 enable positive 250000000 250000000\r\n");
}

*/
void serial_comm::Close() {
    safeWrite("eventoutcontrol mark2 disable\r\n");
    safeWrite("unlogall\r\n");
    cout << "flushing GPS buffers" << endl;
    _port->flush();
    //sleep(1);
    usleep(0.1 * 1000 * 1000);
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
