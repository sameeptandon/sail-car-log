#ifndef RADAR_CORE_H_
#define RADAR_CORE_H_

#include <global.h>
#include <usbcan.h>

namespace vlr {

#define     BOSCH_RADAR_INFO1_CAN_ID            0x6B5
#define     BOSCH_RADAR_FIRST_TARGET_CAN_ID     0x6B6
#define     BOSCH_RADAR_LAST_TARGET_CAN_ID      0x6D5
#define     BOSCH_RADAR_INFO2_CAN_ID            0x6D6

#define     MAX_TARGETS                         32

struct BoschRadarTarget {
  int id, measured, historical;
  double distance, lateral_offset, lateral_offset_var;
  double relative_acceleration, relative_velocity;
};

class BoschRadar {
public:
  BoschRadar();
  ~BoschRadar();
  void connect(const std::string& device);
  void disconnect();
  void sendMotionData(bool received_vehicle_state, double yaw_rate, double forward_velocity, double forward_accel);
  void process();

private:
  int32_t bitString(uint8_t buffer[8], int32_t start, int32_t num_bits);
  void printBitString(uint8_t buffer[8], int32_t start, int32_t num_bits, const std::string& text);
  int twosComplement(int x, int n);

private:
   UsbCan usbcan_;

public:
  int targets_ready;

  double cycle_time, sync_time;
  int measurement_number, num_objects;
  int sensor_dirty, hw_failure, sgu_failure, sgu_comsurveillance;
  int cu_io1_received, cu_io2_received, cu_request_received;
  int scu_temperature, version, sensor_class;

  BoschRadarTarget target[MAX_TARGETS];

};

} // namespace vlr
#endif
