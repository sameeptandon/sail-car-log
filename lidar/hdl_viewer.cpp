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
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <typeinfo>

#include "gps_hdl_grabber.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

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
class SimpleHDLViewer
{
  public:
    typedef PointCloud<PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer(Grabber& grabber,
                     PointCloudColorHandler<PointXYZI> &handler)
      : cloud_viewer_(new PCLVisualizer("PCL HDL Cloud"))
      , grabber_(grabber)
      , handler_(handler)
    {
    }

    void 
    cloud_callback(const CloudConstPtr& cloud)
    {
      FPS_CALC("cloud callback");
      boost::mutex::scoped_lock lock(cloud_mutex_);
      cloud_ = cloud;
      //std::cout << cloud->points[0] << " " << cloud->size() << std::endl;
    }

    void 
    cloud_callback(const CloudConstPtr& cloud, float startAngle,
                    float endAngle)
    {
      FPS_CALC("cloud callback");
      boost::mutex::scoped_lock lock(cloud_mutex_);
      cloud_ = cloud;
    }

    void 
    keyboard_callback(const KeyboardEvent& event,
                       void* cookie)
    {
      if(event.keyUp())
      {
        return;
      }
    }

    void 
    mouse_callback(const MouseEvent& mouse_event,
                    void* cookie)
    {
      if(mouse_event.getType() == MouseEvent::MouseButtonPress && 
          mouse_event.getButton() == MouseEvent::LeftButton)
      {
        cout << mouse_event.getX() << " , " << mouse_event.getY() << endl;
      }
    }

    void 
    run()
    {
      cloud_viewer_->addCoordinateSystem(3.0);
      cloud_viewer_->setBackgroundColor(0, 0, 0);
      cloud_viewer_->initCameraParameters();
      cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances(0.0, 50.0);
      //cloud_viewer_->registerMouseCallback(&SimpleHDLViewer::mouse_callback, *this);
      //cloud_viewer_->registerKeyboardCallback(&SimpleHDLViewer::keyboard_callback, *this);

      //boost::function<void(const CloudConstPtr&, float, float)> cloud_cb = boost::bind(&SimpleHDLViewer::cloud_callback, this, _1, _2, _3);
      boost::function<void(const CloudConstPtr&)> cloud_cb = boost::bind(
          &SimpleHDLViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback(
          cloud_cb);

      grabber_.start();

      boost::posix_time::ptime ptime = boost::posix_time::second_clock::local_time();
      stringstream folder_stream;
      folder_stream << "frames/" << boost::posix_time::to_simple_string(ptime) << "/";
      std::string dir = folder_stream.str();
      boost::filesystem::create_directories(dir);
      cout << dir << " was created" << endl;

      uint64_t last_stamp = 0;
      while(!cloud_viewer_->wasStopped())
      {
        CloudConstPtr cloud;

        // See if we can get a cloud
        if(cloud_mutex_.try_lock())
        {
          cloud_.swap(cloud);
          cloud_mutex_.unlock();
        }

        if(cloud)
        {
          FPS_CALC("drawing cloud");
          handler_.setInputCloud(cloud);
          if(!cloud_viewer_->updatePointCloud(cloud, handler_, "HDL")) {
            cloud_viewer_->addPointCloud(cloud, handler_, "HDL");
          }

          cloud_viewer_->spinOnce();

          //Save the ldr files to disk
          writeLDRFile(dir, cloud);
        }

        if(!grabber_.isRunning())
          cloud_viewer_->spin();

        boost::this_thread::sleep(boost::posix_time::microseconds(100));
      }

      grabber_.stop();

      cloud_connection.disconnect();
    }

    void
    writeLDRFile(const std::string& dir, const CloudConstPtr& cloud)
    {
      PointCloud<PointXYZI> dataCloud = *cloud.get();
      uint64_t timeStamp = dataCloud.header.stamp & 0x00000000ffffffffl;

      stringstream ss;
      ss << dir << timeStamp << ".ldr";

      FILE *ldrFile = fopen(ss.str().c_str(), "wb");

      for(PointCloud<PointXYZI>::iterator iter = dataCloud.begin(); iter != dataCloud.end(); ++iter)
      {
          //File format is little endian, 3 floats and 1 short (14 bytes) per entry
          float pBuffer[] = {iter->x, iter->y, iter->z};
          fwrite(pBuffer, 1, sizeof(pBuffer), ldrFile);
          short iBuffer[] = {iter->intensity};
          fwrite(iBuffer, 1, sizeof(iBuffer), ldrFile);
      }
      fclose(ldrFile);
    }

    boost::shared_ptr<PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<ImageViewer> image_viewer_;

    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    CloudConstPtr cloud_;
    PointCloudColorHandler<PointXYZI> &handler_;
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
  string hdlCalibration, pcapFile, format("XYZI"), ip("127.0.0.1");

  if(find_switch(argc, argv, "-h") || 
      find_switch(argc, argv, "--help"))
  {
    usage(argv);
    return(0);
  }

  parse_argument(argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument(argc, argv, "-pcapFile", pcapFile);
  parse_argument(argc, argv, "-ip", ip);

  GPSHDLGrabber *grabber = NULL;

  if(pcapFile.empty()) {
    boost::asio::ip::address ipAddress =  boost::asio::ip::address::from_string(ip);
    short port = 2368;
    grabber = new GPSHDLGrabber(ipAddress, port, hdlCalibration);
    cout << "ip address: " << ipAddress << endl;
  } else {
    grabber = new GPSHDLGrabber(hdlCalibration, pcapFile);
    cout << "pcap file: " << pcapFile << endl;
  }
  float minDistance = 0.00001f;
  grabber->setMinimumDistanceThreshold(minDistance);

  PointCloudColorHandlerGenericField<PointXYZI> color_handler("intensity");

  SimpleHDLViewer v(*grabber, color_handler);
  v.run();

  return(0);
}

