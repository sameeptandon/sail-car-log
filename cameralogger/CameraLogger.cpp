//#define TWO_CAM
//#define DISPLAY
#define DEBUG_NO_SENSORS

#include <fstream>
#include "CameraLogger.h"
#include "GPSLogger.h"

#define ZMQ_CAMERALOGGER_BIND_PORT 5001 // this app will listen on this port
#define ZMQ_DIAGNOSTICS_SEND_PORT 5000 // this app will send to master on this port

#define CAMERA_DISPLAY_SKIP 3
#define NUMTHREAD_PER_BUFFER 10

SyncBuffer cam1_buff[NUMTHREAD_PER_BUFFER];
SyncBuffer cam2_buff[NUMTHREAD_PER_BUFFER];

zmq::context_t context(1);
zmq::socket_t my_commands(context, ZMQ_SUB);
zmq::socket_t send_diagnostics(context, ZMQ_PUB);

bool is_done_working = false; 
bool quit_via_user_input = false;

Time currentTime(boost::posix_time::microsec_clock::local_time());
Time lastTime(currentTime);

inline void ImageCallback(Image* pImage, const void* pCallbackData, 
    SyncBuffer* w_buff) { 
  if (!is_done_working) {
    Image* im = new Image();
    pImage->Convert(PIXEL_FORMAT_BGR, im);
    if (!w_buff->getBuffer()->pushBack(im)) {
      boost::mutex::scoped_lock io_lock ( *(w_buff->getMutex()) );
      cerr << "Warning! Buffer full, overwriting data!" << endl; 
    }
  }
}

inline void sendDiagnosticsMessage(string message) { 
  zmq::message_t diag_msg_t((void *) message.c_str(), message.length(), NULL);
  send_diagnostics.send(diag_msg_t, ZMQ_NOBLOCK);
}

void ctrlC (int)
{
#ifndef DISPLAY
  //printf("\nCtrl-C detected, exit condition set to true.\n");
  //is_done_working = true;
#endif
#ifdef DISPLAY
  printf("\nCtrl-C Disabled! Use 'q' to quit instead\n");
#endif
}

int main(int argc, char** argv)
{
  using namespace boost::program_options;
  options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("serial,s", value<string>(), "the serial location for the gps unit")
    ("maxframes,m", value<uint64_t>(), "maximum number of frames to capture")
    ("output,o", value<string>(), "the filename for data logging");

  signal (SIGINT, ctrlC);

  string diagnostics_address = "tcp://localhost:"+boost::to_string(ZMQ_DIAGNOSTICS_SEND_PORT);
  send_diagnostics.connect(diagnostics_address.c_str());
  sleep(1);
  sendDiagnosticsMessage("WARN:Connecting to Cameras...");


  variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);
  notify(vm);
  if (vm.count("help")) { 
    cout << desc << endl;
    return 1;
  }
  string fname1 = "";
  string fname2 = "";
  string fnamegps = "";
  if (vm.count("output")) { 
    fname1 = vm["output"].as<string>() + "1.avi";
    fname2 = vm["output"].as<string>() + "2.avi";
    fnamegps = vm["output"].as<string>() + "_gps.out";
  }
  else {
    cout << desc << endl;
    return 1;
  };

  uint64_t maxframes = -1; 
  if (vm.count("maxframes")) {
    maxframes = vm["maxframes"].as<uint64_t>(); 
  }

#ifdef DEBUG_NO_SENSORS
  bool useGPS = false;
#else
  bool useGPS = true; 
#endif
  GPSLogger gpsLogger;
  ofstream gps_output;
  if (useGPS) {
    gps_output.open(fnamegps.c_str());
    gpsLogger.Connect(vm["serial"].as<string>());
  }
  int imageWidth = 1280;
  int imageHeight = 960;

  cout << "Capturing for maximum of " << maxframes << " frames" << endl; 
  cout  << "Filenames: " << fname1 << " " << fname2 << endl; 

  for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++)
  {
    cam1_buff[thread_num].getBuffer()->setCapacity(1000);
    cam2_buff[thread_num].getBuffer()->setCapacity(1000);
  }

#ifndef DEBUG_NO_SENSORS
  Camera* cam1 = ConnectCamera(0); // 0 indexing
  assert(cam1 != NULL);
  RunCamera(cam1);

  Consumer<Image>* cam1_consumer[NUMTHREAD_PER_BUFFER];
  for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++) {
    stringstream sstm;
    sstm << "split_" << thread_num << "_" << fname1; 
    string thread_fname = sstm.str();
    cam1_consumer[thread_num] = new Consumer<Image>(
        cam1_buff[thread_num].getBuffer(),
        thread_fname, cam1_buff[thread_num].getMutex(), 50.0f,
        imageWidth, imageHeight);
  }
#endif

#ifdef DISPLAY
  cvNamedWindow("cam1", CV_WINDOW_AUTOSIZE);
#endif

#ifdef TWO_CAM
  Camera* cam2 = ConnectCamera(1); // 0 indexing 
  assert(cam2 != NULL);
  RunCamera(cam2); 
  Consumer<Image>* cam2_consumer[NUMTHREAD_PER_BUFFER];
  for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++) {
    stringstream sstm;
    sstm << "split_" << thread_num << "_" << fname2; 
    string thread_fname = sstm.str();
    cam2_consumer[thread_num] = new Consumer<Image>(
        cam2_buff[thread_num].getBuffer(),
        thread_fname, cam2_buff[thread_num].getMutex(), 50.0f,
        imageWidth, imageHeight);
  }
#ifdef DISPLAY
  cvNamedWindow("cam2", CV_WINDOW_AUTOSIZE);
#endif //DISPLAY
#endif //TWO_CAM

  IplImage* img = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
  string lastValidGPSPacket;

  //// setup ZMQ communication
  string bind_address = "tcp://*:"+boost::to_string(ZMQ_CAMERALOGGER_BIND_PORT);
  cout << "binding to " << bind_address << endl; 
  my_commands.bind("tcp://*:5001");
  my_commands.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);
  cout << "binding successful" << endl; 
  
#ifdef DEBUG_NO_SENSORS
  img = cvLoadImage("/home/smart/sail-car-log/cameralogger/xkcd.png");
  lastValidGPSPacket="#MARK1PVAA,USB1,0,92.0,EXACT,1753,238273.050,00000000,0000,470;1753,238273.050000025,37.856359728,-122.494864165,110.390118713,16.041177523,-24.789919285,-1.752024973,-3.954175540,4.393638450,32.292000751,INS_SOLUTION_GOOD*016857cc";
#endif
  
  // start GPS Trigger
  if (useGPS) gpsLogger.Run();
  
  uint64_t numframes = 0;
  uint64_t lastframes = 0; 
  ///////// main loop
  while (!is_done_working) {
    numframes++;
    if (numframes > maxframes) { 
      is_done_working = true;
    }
    try {
      zmq::message_t command_msg; 
      if (my_commands.recv(&command_msg, ZMQ_NOBLOCK) == true) { 
        string command((const char*)command_msg.data(), command_msg.size());
        if (command.compare("TERMINATE") == 0) {
          // properly close the application 
          is_done_working = true; 
          quit_via_user_input = true;
        } 
        else if (command.compare("LASTCAMERADATA") == 0) { 
          // transmit the camera data over
        }
        else if (command.compare("LASTGPSDATA") == 0) { 
          // transmit the last known GPS log over
        }
      }
    } catch (const zmq::error_t&e) {}

#ifndef DEBUG_NO_SENSORS
    Image image; 
    cam1->RetrieveBuffer(&image);
    ImageCallback(&image, NULL, &cam1_buff[numframes % NUMTHREAD_PER_BUFFER]);

    Image cimage; 
    if (numframes % CAMERA_DISPLAY_SKIP == 0) {
      image.Convert(PIXEL_FORMAT_BGR, &cimage);
      convertToCV(&cimage, img); 
      #ifdef DISPLAY
      show(img, "cam1");
      #endif //DISPLAY
    }
#endif //DEBUG_NO_SENSORS

#ifdef TWO_CAM
    cam2->RetrieveBuffer(&image);
    ImageCallback(&image, NULL, &cam2_buff[numframes % NUMTHREAD_PER_BUFFER]);
    if (numframes % CAMERA_DISPLAY_SKIP == 0) {
      image.Convert(PIXEL_FORMAT_BGR, &cimage);
      convertToCV(&cimage, img); 
      #ifdef DISPLAY
      show(img, "cam2");
      #endif //DISPLAY
    }
#endif // TWO_CAM

#ifdef DISPLAY
    char r = cvWaitKey(1);
    if (r == 'q') {
      is_done_working = true;
      quit_via_user_input = true;
    }
#endif //DISPLAY

    if (useGPS) {
      GPSLogger::GPSPacketType packet = gpsLogger.getPacket();
      int GPSPacketSuccess = boost::get<0>(packet); 
      if (GPSPacketSuccess > 0) { 
        lastValidGPSPacket = boost::get<1>(packet);
        gps_output << lastValidGPSPacket << endl;
      } else {
        sendDiagnosticsMessage("WARN:Bad GPS Packet"); 
      }

    }

    currentTime = Time(boost::posix_time::microsec_clock::local_time()); 
    if ((currentTime - lastTime).total_milliseconds() > 1000) {
      string captureRateMsg = "INFOCAPTURERATE:" + boost::to_string(numframes-lastframes); 
      sendDiagnosticsMessage(captureRateMsg);

      int queue_size = 0; 
      for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++)
      {
        queue_size += cam1_buff[thread_num].getBuffer()->getSize();
        queue_size += cam2_buff[thread_num].getBuffer()->getSize(); 
      }

      string bufferSizeMsg = "INFOBUFFERSIZE:" + boost::to_string(queue_size);
      sendDiagnosticsMessage(bufferSizeMsg);

      // encode last image for transmission
      Mat lastCameraImage(img);
      vector<uchar> lastCameraImageBuf; string lastCameraImageMsg; 
      resize(lastCameraImage, lastCameraImage, Size(320,240));
      imencode(".png", lastCameraImage, lastCameraImageBuf);
      lastCameraImageMsg = string(lastCameraImageBuf.begin(), lastCameraImageBuf.end());
      lastCameraImageMsg = "CAM:" + lastCameraImageMsg; 
      sendDiagnosticsMessage(lastCameraImageMsg);

      GPSRecord record(lastValidGPSPacket);
      if (record.isValid()) {
        std::ostringstream lat;
        lat << std::fixed << std::setprecision(8);
        lat << record.latitude; 
        std::ostringstream lon;
        lon << std::fixed << std::setprecision(8);
        lon << record.longitude;
        string gpsMsg = "GPS:" + lat.str() + "," + lon.str();
        sendDiagnosticsMessage(gpsMsg);
      }


      lastTime = currentTime;
      lastframes = numframes; 
    }
#ifdef DEBUG_NO_SENSORS
    usleep(200); 
#endif
  }  
  cout << "numframes = " << numframes << endl;


  /////// cleanup
  if (useGPS) gpsLogger.Close();
#ifdef DISPLAY
  cvDestroyWindow("cam1");
  cvDestroyWindow("cam2");
#endif
#ifndef DEBUG_NO_SENSORS
  for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++) {
    cam1_consumer[thread_num]->stop();
    delete cam1_consumer[thread_num];
  }
  CloseCamera(cam1);
  delete cam1; 
#endif 
#ifdef TWO_CAM
  for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++) {
    cam2_consumer[thread_num]->stop();
    delete cam2_consumer[thread_num];
  }
  CloseCamera(cam2);
  delete cam2;
#endif
  if (quit_via_user_input) return 1;
  return 0;
}
