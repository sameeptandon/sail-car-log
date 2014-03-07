#include <signal.h>
#include <ros/ros.h>
#include <global.h>
#include <vlrException.h>
#include <driving_common/RadarSensor.h>
#include <driving_common/CanStatus.h>
#include <applanix/ApplanixPose.h>
#include <usbfind.h>
#include <radarcore.h>

namespace vlr {

ros::NodeHandle nh_("/driving");

ros::Subscriber can_status_sub_;
ros::Subscriber applanix_sub_;

ros::Publisher radar_pub_;

std::string topic_name;

int radar_num;
std::string radar_device;

bool received_can = false;
double yaw_rate, forward_velocity, forward_accel;

bool received_applanix = false;

driving_common::RadarSensor radar_msg;
driving_common::RadarTarget target;
BoschRadar radar;

bool use_can = false;

void canStatusHandler(const driving_common::CanStatus& can) {
  forward_velocity = 0.5 * dgc::dgc_kph2ms(can.wheel_speed_rl + can.wheel_speed_rr);
  received_can = true;
}

void applanixHandler(const applanix::ApplanixPose& pose)
{
  yaw_rate = pose.rate_yaw;
  forward_accel = pose.accel_x;
  received_applanix = true;
}

void publish(const BoschRadar& r) {

  radar_msg.measurement_number = r.measurement_number;

  radar_msg.target.clear();
  for(int32_t i = 0; i < MAX_TARGETS; i++)
    if(r.target[i].id != 0) {
      target.id = r.target[i].id;
      target.measured = r.target[i].measured;
      target.historical = r.target[i].historical;
      target.distance = r.target[i].distance;
      target.lateral_offset = r.target[i].lateral_offset;
      target.lateral_offset_var = r.target[i].lateral_offset_var;
      target.relative_acceleration =  r.target[i].relative_acceleration;
      target.relative_velocity = r.target[i].relative_velocity;
      radar_msg.target.push_back(target);
    }

  radar_msg.sensor_dirty = r.sensor_dirty;
  radar_msg.hw_failure = r.hw_failure;
  radar_msg.sgu_failure = r.sgu_failure;
  radar_msg.sgu_comsurveillance = r.sgu_comsurveillance;
  radar_msg.cu_io1_received = r.cu_io1_received;
  radar_msg.cu_io2_received = r.cu_io2_received;
  radar_msg.cu_request_received = r.cu_request_received;
  radar_msg.scu_temperature = r.scu_temperature;
  radar_msg.timestamp = vlr::Time::current();

  radar_pub_.publish(radar_msg);
}

template <class T> void getParam(std::string key, T& var) {
  if(!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}

void readParameters() {

  getParam("id", radar_num);
  getParam("device", radar_device);
  getParam("use_can", use_can);
  std::stringstream s;

  s << "RadarSensor" ; //<< radar_num;
  topic_name = "sensors/" + s.str();

}

void shutdownHandler(int x) {
  if(x == SIGINT || x == SIGSEGV) {
    fprintf(stderr, "\nINFO: Shutting down RADAR%i\n", radar_num);
    radar.disconnect();
    exit(0);
  }
}

} // namespace vlr

using namespace vlr;

int main(int argc, char **argv) {

  vlr::readParameters();

   // Publishers
  radar_pub_ = nh_.advertise<driving_common::RadarSensor> (topic_name, 5);

  signal(SIGINT, vlr::shutdownHandler);

  std::string port;
  if(!usbFindLookupParamString(radar_device, port)) {
    throw Exception(std::string("ERROR: could not connect to device ") + radar_device);
  }

  if (radar_device == port) {
    fprintf(stderr, "Opening %s=%s\n", radar_device.c_str(), port.c_str());
  }
  else {
    fprintf(stderr, "Opening %s\n", port.c_str());
  }

  if(use_can) {
      // Subscribers
    applanix_sub_ = nh_.subscribe("ApplanixPose", 5, vlr::applanixHandler);
    can_status_sub_ = nh_.subscribe("CanStatus", 5, vlr::canStatusHandler);
  }

  // open connection to radar
  try {
    radar.connect(port);
  }
  catch(VLRException& e) {
    std::cout << e.what() << "\n";
    exit(-5);
  }

  static const double motion_update_rate = 50;  // radar needs motion update 50 times/s

  // To simulate vehicle
  yaw_rate = 0.0;
  forward_velocity = 5.0;
  forward_accel = 0.0;

  double motion_update_time = 1.0/motion_update_rate;
  ros::Rate publish_loop_rate_(1000); // high rate since radar is polled
  double last_time = vlr::Time::current();
  while (ros::ok()) {
    if (use_can) {
      double t = vlr::Time::current();
      if (t - last_time > motion_update_time) {
        radar.sendMotionData(received_can && received_applanix, yaw_rate, forward_velocity, forward_accel);
        last_time = t;
      }
    }
    radar.process();
    if (radar.targets_ready) {
      publish(radar);
      radar.targets_ready = 0;
      std::cout << ".";
    }

    ros::spinOnce();
    publish_loop_rate_.sleep();
  }

  return 0;
}
