#include <usbcan.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

#define BOSCH_OBJECT_STARTER 0x3f2
#define BOSCH_OBJECT_DATA_START 0x3f3
#define BOSCH_OBJECT_DATA_END 0x432
#define BOSCH_OBJECT_ENDER 0x433
#define MAX_TARGETS 32

struct ObjectADataPacket {
    double  dx;
    double  vx;
    double  dy;
    double  ax;
    double  wExist;
    double  dySigma;
    uint8_t flagHist;
    uint8_t flagMeas;
    uint8_t flagValid;
    uint8_t MessAconsistBit; 
};

struct ObjectBDataPacket {
    double vy;
    int32_t handle;
    double wObstacle;
    int32_t movingState;
    double axSigma;
    double vxSigma;
    double dxSigma; 
    uint8_t MessBconsistBit; 
};

struct ObjectDataPacket {
    ObjectADataPacket packet_a;
    ObjectBDataPacket packet_b; 
};

struct ObjectStarterPacket { 
    double psiDt;
    double vEgo;
    double aEgo;
    double slipAng;
    uint8_t MessStarterConsistBit;
};

struct ObjectEnderPacket {
    double timestamp;
    double kapCurvTraj;
    uint8_t MessEnderConsistBit;
};

ObjectDataPacket packets[MAX_TARGETS];

int32_t bitString(uint8_t buffer[8], int32_t start, int32_t num_bits) {
    int32_t value = 0, factor = 1;
    for (int32_t i = start; i < start + num_bits; i++) { 
        int32_t byte = i / 8; 
        int32_t bit = i % 8;
        if (buffer[byte] & (1 << bit)) { value += factor;}
        factor *= 2;
    }
    return value;
}

int32_t twosComplement(int32_t x, int32_t n) {
    int mask = 1;
    for (int i = 0; i < n; i++) { mask *= 2; }
    mask -= 1;

    if (x & (1 << (n - 1))) {
        x = ~x;
        x += 1;
        x &= mask;
        x *= -1;
    }
    return x;
}

ObjectADataPacket parse_object_a_data_packet(uint8_t buffer[8]) {
    ObjectADataPacket data;
    memset(&data, 0, sizeof(ObjectADataPacket));
    data.dx = bitString(buffer, 0, 12) / 16.0;
    data.vx = twosComplement(bitString(buffer,12,12),12) / 16.0;
    data.dy = twosComplement(bitString(buffer,24,14),14) / 64.0;
    data.ax = twosComplement(bitString(buffer,38,10),10) / 32.0;
    data.wExist = bitString(buffer,48,6) / 64.0;
    data.dySigma = bitString(buffer,54,6) / 16.0;
    data.flagHist = bitString(buffer,60,1);
    data.flagMeas = bitString(buffer,61,1);
    data.flagValid = bitString(buffer,62,1);
    data.MessAconsistBit = bitString(buffer,63,1);

    return data;
}

ObjectBDataPacket parse_object_b_data_packet(uint8_t buffer[8]) { 
    ObjectBDataPacket data;
    memset(&data, 0, sizeof(ObjectBDataPacket));
    data.vy = twosComplement(bitString(buffer,0,12),12) / 16.0;
    data.handle = bitString(buffer,12,7);
    data.wObstacle = bitString(buffer,19,5) / 32.0;
    data.movingState = bitString(buffer,24,3);
    data.axSigma = bitString(buffer,27,6) / 16.0;
    data.vxSigma = bitString(buffer,33,6) / 16.0;
    data.dxSigma = bitString(buffer,39,6) / 16.0;
    data.MessBconsistBit = bitString(buffer,63,1);
}

using namespace std;
void print_packet(ObjectDataPacket &packet) {
    cout << "Object: " << packet.packet_b.handle << ", dx: " << packet.packet_a.dx << ", dy: " << packet.packet_a.dy << ", vx: " << packet.packet_a.vx << ", vy: " << packet.packet_b.vy << endl; 
}

int main() {

  cout << " >> Starting USB Can Demo << " << endl;

  UsbCan device;
  device.open("");

  uint8_t message[8];
  uint32_t can_id = 0; 
  uint32_t can_length = 0; 
  memset(&packets, 0, sizeof(ObjectDataPacket)*MAX_TARGETS);

  while(1) { 

    if (device.readMessage(can_id, &message[0], can_length) && can_id != 0 ) {
        if(can_id >= BOSCH_OBJECT_DATA_START && 
                can_id <= BOSCH_OBJECT_DATA_END) {
            if (can_id % 2 == 1) { 
                ObjectADataPacket packet = parse_object_a_data_packet(message);
                packets[ (can_id - BOSCH_OBJECT_DATA_START) / 2 ].packet_a = packet;

                //cout << "Object A data packet: 0x" << can_id << ", " << packet.dx << ", " << packet.dy << ", " << packet.vx << endl ;  
            } else {
                ObjectBDataPacket packet = parse_object_b_data_packet(message);
                packets[ (can_id - BOSCH_OBJECT_DATA_START) / 2 ].packet_b = packet;
                //cout << "Object B data packet: 0x" << can_id << ", " << packet.handle << ", " << packet.axSigma << endl;
            }

        } else if (can_id == BOSCH_OBJECT_STARTER) {
            memset(&packets, 0, sizeof(ObjectDataPacket)*MAX_TARGETS);

        } else if (can_id == BOSCH_OBJECT_ENDER) {
            for (int i = 0; i < MAX_TARGETS; i++) { 
                if (packets[i].packet_a.flagValid) {
                    print_packet(packets[i]);
                }
            }
        }
            //cout << "Received Message: 0x" << std::hex << can_id << " with " << can_length << " bytes of data" << endl; 
    }
  }

  device.close();
  return 1; 

}
