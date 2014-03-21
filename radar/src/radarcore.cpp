#include <global.h>
#include <serial.h>
#include <radarcore.h>
#include <usbcan.h>

namespace vlr {

BoschRadar::BoschRadar() : targets_ready(0) {

}

BoschRadar::~BoschRadar() {

}

void BoschRadar::connect(const std::string& device) {
  usbcan_.open(device);
  targets_ready = 0;
}

void BoschRadar::disconnect() {
  usbcan_.close();
}

void BoschRadar::sendMotionData(bool received_vehicle_state, double yaw_rate, double forward_velocity, double forward_accel) {
  uint8_t message[8];

  if (!received_vehicle_state) {
    fprintf(stderr, "X");
    return;
  }

    // vehicle state has been received, wheel_speeds and yaw_rate are valid
  fprintf(stderr, "*");

  int16_t i = (int16_t) rint(yaw_rate * 16384);
  message[0] = i & 0xFF;
  message[1] = (i >> 8) & 0xFF;

  i = (int16_t) rint(forward_velocity * 256);
  message[2] = i & 0xFF;
  message[3] = (i >> 8) & 0xFF;

  i = (int16_t) rint(forward_accel * 2048);
  message[4] = i & 0xFF;
  message[5] = (i >> 8) & 0xFF;

  message[6] = 0;
  message[7] = 0;

  if (forward_velocity == 0) message[6] = 2;

  usbcan_.sendMessage(0x6B2, message, 8);
}

void BoschRadar::process() {
  uint8_t message[100];
  unsigned int can_id;
  int i, message_length, id;
  static int counter = 0;

  while (usbcan_.readMessage(&can_id, message, &message_length)) {
    if (can_id == BOSCH_RADAR_INFO1_CAN_ID) {
      measurement_number = bitString(message, 0, 16);
      cycle_time = bitString(message, 16, 16) / 2048.0;
      //num_objects = bitString(message, 32, 16);
      sensor_dirty = bitString(message, 56, 1);
      hw_failure = bitString(message, 57, 1);
      sgu_failure = bitString(message, 59, 1);
      sgu_comsurveillance = bitString(message, 60, 1);
      cu_io1_received = bitString(message, 61, 1);
      cu_io2_received = bitString(message, 62, 1);
      cu_request_received = bitString(message, 63, 1);
      for (i = 0; i < MAX_TARGETS; i++) {
        target[i].id = 0;
      }
      counter = 0;
    }
    else if (can_id == BOSCH_RADAR_INFO2_CAN_ID) {
      sync_time = bitString(message, 24, 16) / 2048.0;
      scu_temperature = twosComplement(bitString(message, 40, 8), 8);
      version = bitString(message, 56, 4);
      sensor_class = bitString(message, 60, 4);
      num_objects = counter;
      targets_ready = 1;
    }
    else if (can_id >= BOSCH_RADAR_FIRST_TARGET_CAN_ID && can_id <= BOSCH_RADAR_LAST_TARGET_CAN_ID) {
      if (counter < MAX_TARGETS) {
        id = bitString(message, 8, 6);
        if (id > 0) {
          target[counter].id = id;
          target[counter].lateral_offset_var = bitString(message, 0, 8) / 32.0;
          target[counter].historical = bitString(message, 14, 1);
          target[counter].measured = bitString(message, 15, 1);
          target[counter].relative_velocity = twosComplement(bitString(message, 16, 12), 12) / 16.0;
          target[counter].distance = bitString(message, 28, 12) / 16.0;
          target[counter].lateral_offset = twosComplement(bitString(message, 40, 14), 14) / 128.0;
          target[counter].relative_acceleration = twosComplement(bitString(message, 54, 10), 10) / 32.0;
          counter++;
        }
      }
    }
  }
}

int32_t BoschRadar::bitString(uint8_t buffer[8], int32_t start, int32_t num_bits) {
  int32_t value = 0, factor = 1;
  for (int32_t i = start; i < start + num_bits; i++) {
    int32_t byte = i / 8;
    int32_t bit = i % 8;
    if (buffer[byte] & (1 << bit)) {value += factor;}
    factor *= 2;
  }
  return value;
}

void BoschRadar::printBitString(uint8_t buffer[8], int32_t start, int32_t num_bits, const std::string& text) {
  fprintf(stderr, "%s : %d - %d : ", text.c_str(), start, start + num_bits);
  for (int32_t i = start + num_bits - 1; i >= start; i--) {
    int32_t byte = i / 8;
    int32_t bit = i % 8;
    if (buffer[byte] & (1 << bit)) fprintf(stderr, "1");
    else fprintf(stderr, "0");
  }
  fprintf(stderr, "\n");
}

int BoschRadar::twosComplement(int x, int n) {
  int mask = 1;
  for (int i = 0; i < n; i++) {mask *= 2;}

  mask -= 1;

  if (x & (1 << (n - 1))) {
    x = ~x;
    x += 1;
    x &= mask;
    x *= -1;
  }
  return x;
}

} // namespace vlr
