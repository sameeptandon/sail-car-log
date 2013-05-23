#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <serial/serial.h>
#include "fps_calc.h"

using namespace std;
using namespace serial;

void safeWrite(Serial* port, string cmd) {
    cout << "Command: " << cmd << endl; 
    port->write(cmd);
    port->flushOutput();
    sleep(1);
}

string safeRead(Serial* port) {
    string buf = port->readline();
    cout << buf << endl;
    return buf;
}

Serial* Connect(string port) {
    int baudrate = 115200;
    Timeout my_timeout(1000,1000,0,1000,0); // 50 ms read and write timeout
    Serial* serial_port = new Serial(port, baudrate, my_timeout);
    // flush stuff from previous runs
    serial_port->flushInput();
    serial_port->flushOutput();

    safeWrite(serial_port, "unlogall usb1\r\n");
    safeRead(serial_port);
    serial_port->flush();
    safeWrite(serial_port, "fix none\r\n");
    //safeWrite(serial_port, "fix position 37.123 -123.32 20.20\r\n");
    //safeWrite(serial_port, "fix velocity 37.123 -123.32 20.20\r\n");
    //safeWrite(serial_port, "fix altitude 20.20\r\n");
    //safeWrite(serial_port, "saveconfig\r\n");
    safeRead(serial_port);
    //safeWrite(serial_port, "eventoutcontrol mark2 enable positive 10000000 10000000\r\n");
    safeWrite(serial_port, "eventoutcontrol mark2 enable positive 250000000 250000000\r\n");
    safeWrite(serial_port, "eventincontrol mark1 event\r\n");
    //safeWrite(serial_port, "alignmentmode unaided_static\r\n");
    //safeWrite(serial_port, "setupsensor sensor1 mark1 positive 100 mark1 event positive 0 10\r\n"); 
    //safeWrite(serial_port, "eventoutcontrol mark1 enable\r\n");
    safeRead(serial_port);
    safeRead(serial_port);
    return serial_port;
}

void Run(Serial* port) {
    //safeWrite(port, "setmark1offset 0 0 0 0 0 0\r\n");
    //safeWrite(port, "log mark1time onnew\r\n");
    safeWrite(port, "log usb1 mark1pvaa onnew\r\n");
    //safeWrite(port, "log taggedmark1pva onnew\r\n");
    //safeWrite(port, "log usb1 bestposa ontime 0.5 0 nohold\r\n");
    while(1) {
    safeRead(port);
    FPS_CALC("loop rate");
    }

}


int main(int argc, char** argv) {
    // parse input
    std::string port(argv[1]);
    Serial* serial_port = Connect(port); 
    Run(serial_port);
    serial_port->close();
    delete serial_port; 
}


