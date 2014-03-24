#pragma once
#include <boost/tuple/tuple.hpp>
#include <serial/serial.h>

using namespace std;
using namespace serial;

class serial_comm {
    public:
        void safeWrite(string cmd);
        string safeRead();

        void Connect(string port);
        //void Run();
        void Close();

    private:
        Serial* _port; 
};
