/*
 * Software License Agreement(BSD License)
 *
 *  Copyright(c) 2011, The MITRE Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Keven Ring <keven@mitre.org>
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/pcd_io.h>
#include <boost/program_options.hpp>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <typeinfo>

#include "gps_hdl_grabber.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime();\
    double now = getTime(); \
    ++count; \
    if(now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LDRConverter 
{
  public:
    typedef PointCloud<PointXYZRGBA> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    LDRConverter(Grabber& grabber) 
      : grabber_(grabber) 
    {
      cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    }

    void 
    cloud_callback(const CloudConstPtr& cloud, float start, float end, bool restart)
    {
      FPS_CALC("cloud callback");
      boost::mutex::scoped_lock lock(cloud_mutex_);
      if (restart)
      {
          writeLDRFile(_dir, cloud_);
          cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
          cloud_->header.stamp = cloud->header.stamp;
          cloud_times_.clear();
      }
      (*cloud_) += *cloud;
      for (int k = 0; k < cloud->size(); k++)
          cloud_times_.push_back(cloud->header.stamp);
    }

    void 
    run(const string& folder_name)
    {
      boost::filesystem::create_directories(folder_name);
      _dir = folder_name;
      cout << folder_name << " was created" << endl;


      boost::function<void(const CloudConstPtr&, float, float, bool)> cloud_cb = boost::bind(
              &LDRConverter::cloud_callback, this, _1, _2, _3, _4);
      boost::signals2::connection cloud_connection = grabber_.registerCallback(
              cloud_cb);
      grabber_.start();


      uint64_t last_stamp = 0;
      while(grabber_.isRunning())
      {
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
      }

      grabber_.stop();

      cloud_connection.disconnect();
    }

    void
    writeLDRFile(const std::string& dir, const CloudConstPtr& cloud)
    {
      PointCloud<PointXYZRGBA> dataCloud = *cloud.get();
      uint64_t timeStamp = dataCloud.header.stamp;

      // The timestamp is in microseconds since Jan 1, 1970 (epoch time)
      stringstream ss;
      ss << dir << timeStamp << ".ldr";

      FILE *ldrFile = fopen(ss.str().c_str(), "wb");
      uint32_t k = 0;
      for(PointCloud<PointXYZRGBA>::iterator iter = dataCloud.begin(); iter != dataCloud.end(); ++iter)
      {
          //File format is little endian, 3 floats, 1 short, 1 short (16 bytes) per entry
          float intensity = (float) ( (iter->rgba & 0xffff0000) >> 16);
          float laser_num = (float) ( (iter->rgba & 0x0000ffff) );
          uint64_t cloud_time = cloud_times_[k];
          float cloud_time_delta = (float) ((uint32_t) (timeStamp - cloud_time));  // NOTE Sweep goes backwards
          float pBuffer[] = {iter->x, iter->y, iter->z, intensity, laser_num, cloud_time_delta};
          fwrite(pBuffer, 1, sizeof(pBuffer), ldrFile);
          k++;
          //uint32_t iBuffer[] = {iter->rgba};
          //fwrite(iBuffer, 1, sizeof(iBuffer), ldrFile);
      }
      fclose(ldrFile);
    }

    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex visual_cloud_mutex_;
    string _dir;

    CloudPtr cloud_;
    std::vector<uint64_t> cloud_times_;
};



void
usage(char ** argv)
{
  cout << "usage: " << argv[0]
      << " [-hdlCalibration <path-to-calibration-file>] [-pcapFile <path-to-pcap-file>] [-ip <ip-address>] [-h | --help]"
      << endl;
  cout << argv[0] << " -h | --help : shows this help" << endl;
  return;
}

int 
main(int argc, char ** argv)
{
  string hdlCalibration, pcapFile;
  namespace po = boost::program_options; 

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("hdlcalibration", po::value<string>(), "<path-to-calibration-file")
    ("p", po::value<string>(), "<path-to-pcap-file>")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm); 

  if (vm.count("help")) {
    cout << desc << endl;
    return 0;
  }

  if (vm.count("hdlcalibration")) {
    hdlCalibration = vm["hdlcalibration"].as<string>();
  }

  if (vm.count("p")) {
    pcapFile = vm["p"].as<string>();
    cout << "Pcap File: " << pcapFile << endl;
  }


  /*
  if(find_switch(argc, argv, "-h") || 
      find_switch(argc, argv, "--help"))
  {
    usage(argv);
    return(0);
  }
  */

  //parse_argument(argc, argv, "-calibrationFile", hdlCalibration);
  //parse_argument(argc, argv, "-pcapFile", pcapFile);

  GPSHDLGrabber *grabber = new GPSHDLGrabber(hdlCalibration, pcapFile);
  float minDistance = 0.00001f;
  grabber->setMinimumDistanceThreshold(minDistance);
  
  LDRConverter v(*grabber);
  boost::filesystem::path const p(pcapFile);
  string output_folder(p.branch_path().string() + "/" + p.stem().string() + "_frames/");
  v.run(output_folder);

  return(0);
}

