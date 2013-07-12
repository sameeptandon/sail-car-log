#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <serial/serial.h>
#include "fps_calc.h"

using namespace std;
using namespace serial;

class GPSLogger {
    public:
        void safeWrite(Serial *port, string cmd);
        string safeRead(Serial *port);

        Serial *Connect(string port);
        void Run(Serial *port);
};
