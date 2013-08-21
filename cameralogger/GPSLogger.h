#include <boost/tuple/tuple.hpp>
#include <serial/serial.h>

using namespace std;
using namespace serial;

class GPSLogger {
    public:
        typedef boost::tuple<int,string> GPSPacketType; 
        void safeWrite(string cmd);
        string safeRead();
        GPSPacketType getPacket();

        void Connect(string port);
        void Run();
        void Close();

    private:
        Serial* _port; 
};
