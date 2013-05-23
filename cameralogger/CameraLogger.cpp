#define TWO_CAM
#define DISPLAY
//#define NOSYNC

#include <fstream>
#include "CameraLogger.h"
#include "GPSLogger.h"

#define SHUTTER_PARAM (190)
#define CAMERA_DISPLAY_SKIP 2

#ifdef DISPLAY
void show (const Image* obj, IplImage* img, string name) {
    IplImage* _img = img;
    _img->height = obj->GetRows();
    _img->width = obj->GetCols();
    _img->widthStep = obj->GetStride();
    _img->nChannels = 3;
    _img->imageData = (char*)obj->GetData();
    //Mat im_out;
    //resize(Mat(_img), im_out, Size(640,480));
    imshow(name.c_str(), Mat(_img));
}
#endif

typedef void(*ImageEventCallback) ( class Image* pImage, const void* pCallbackData);

class SyncBuffer {
    private:
        boost::mutex _io_mutex; 
        SynchronizedBuffer<Image>* _buffer;
    public:
        SyncBuffer () {
            _buffer = new SynchronizedBuffer<Image>(&_io_mutex);
        }
        boost::mutex* getMutex() { return & _io_mutex; }
        SynchronizedBuffer<Image>* getBuffer() { return _buffer; } 
};


SyncBuffer buff_1; 
SyncBuffer buff_2;
SyncBuffer d_buff_1;
SyncBuffer d_buff_2;

bool is_done_working = false; 

void ImageCallback(Image* pImage, const void* pCallbackData, 
        SyncBuffer* w_buff, SyncBuffer* d_buff) { 
    if (!is_done_working) {
        Image* im = new Image();
        pImage->Convert(PIXEL_FORMAT_BGR, im);
        if (!w_buff->getBuffer()->pushBack(im)) {
            boost::mutex::scoped_lock io_lock ( *(w_buff->getMutex()) );
            cerr << "Warning! Buffer full, overwriting data!" << endl; 
        }
    }
}

void ImageCallback_1(Image* pImage, const void* pCallbackData) {
    ImageCallback(pImage, pCallbackData, &buff_1, &d_buff_1); 
    FPS_CALC("image_callback_1", buff_1.getBuffer());
}

void ImageCallback_2(Image* pImage, const void* pCallbackData) {
    ImageCallback(pImage, pCallbackData, &buff_2, &d_buff_2); 
    FPS_CALC("image_callback_2", buff_2.getBuffer());
}

Camera* ConnectCamera( int index ) {
    Error error;
    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return NULL;
    }

    if ( numCameras < 1 )
    {
       printf( "No camera detected.\n" );
       return NULL;
    }
    else
    {
       printf( "Number of cameras detected: %u\n", numCameras );
    }

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(index, &guid);
    if (error != PGRERROR_OK)
    {
       PrintError(error);
       return NULL;
    }

    Camera *cam = new Camera();
    error = cam->Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return NULL;
    }
    return cam; 
}

int RunCamera( Camera* cam, ImageEventCallback callback) { 
    Error error;

    // Get the camera information
    CameraInfo camInfo;
    error = cam->GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    PrintCameraInfo(&camInfo);
  
    error = cam->SetVideoModeAndFrameRate(VIDEOMODE_1280x960YUV422,
            FRAMERATE_60); 
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }

    unsigned int bytes = readRegister(cam, 0x1098);
    bytes = bytes & (-1 << 12);
    bytes = bytes | (SHUTTER_PARAM);
    writeRegister(cam, 0x1098, bytes);

    setWhiteBalance(cam, 511, 815);

    FC2Config pConfig;
    cam->GetConfiguration(&pConfig);
    pConfig.grabMode = BUFFER_FRAMES;
    pConfig.numBuffers = 100;
    pConfig.isochBusSpeed = BUSSPEED_S5000;
    pConfig.asyncBusSpeed = BUSSPEED_S5000;
    pConfig.highPerformanceRetrieveBuffer = true;
    cam->SetConfiguration(&pConfig);

#ifndef NOSYNC
    //enable triggering mode
    TriggerMode mTrigger;
    mTrigger.mode = 0; 
    mTrigger.source = 0; 
    mTrigger.parameter = 0; 
    mTrigger.onOff = true; 
    mTrigger.polarity = 1; 
    error = cam->SetTriggerMode(&mTrigger); 
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }
#endif
    
    // Start capturing images
    printf( "Starting capture... \n" );
    //error = cam->StartCapture(callback);
    error = cam->StartCapture();

    return 0;
}

int CloseCamera( Camera* cam) {
    Error error; 

    printf( "Stopping capture... \n" );
    error = cam->StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
    
    error = cam->Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    return 0;
}


void ctrlC (int)
{
  boost::mutex::scoped_lock io_lock (*buff_1.getMutex());
#ifndef DISPLAY
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
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
        ("output,o", value<string>(), "the filename for data logging");

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
        fname1 = "data1.avi";
        fname2 = "data2.avi";
        fnamegps = "data_gps.out";
    };

    bool useGPS = vm.count("serial");
    GPSLogger gpsLogger;
    ofstream gps_output;
    if (useGPS) {
        gps_output.open(fnamegps.c_str());
        gpsLogger.Connect(vm["serial"].as<string>());
    }

    cout  << "Filenames: " << fname1 << " " << fname2 << endl; 

    signal (SIGINT, ctrlC);

    buff_1.getBuffer()->setCapacity(1000);
    buff_2.getBuffer()->setCapacity(1000);
    d_buff_1.getBuffer()->setCapacity(1000);
    d_buff_2.getBuffer()->setCapacity(1000);

    int imageWidth = 1280;
    int imageHeight = 960;

#ifdef NOSYNC
    for (int i =0 ; i  < 100; i ++) 
        cout << "WARNING: SYNC IS DISABLED" << endl;
#endif
    Camera* cam1 = ConnectCamera(0); // 0 indexing
    assert(cam1 != NULL);
    ImageEventCallback c1 = &ImageCallback_1;
    RunCamera( cam1, c1);
    Consumer<Image> consumer_1(buff_1.getBuffer(), fname1,
            buff_1.getMutex(), 50.0f, imageWidth, imageHeight );
#ifdef DISPLAY
    cvNamedWindow("cam1", CV_WINDOW_AUTOSIZE);
#endif

#ifdef TWO_CAM
    Camera* cam2 = ConnectCamera(1); // 0 indexing 
    assert(cam2 != NULL);
    ImageEventCallback c2 = &ImageCallback_2;
    RunCamera( cam2, c2); 
    Consumer<Image> consumer_2(buff_2.getBuffer(), fname2,
            buff_2.getMutex(), 50.0f, imageWidth, imageHeight );
#ifdef DISPLAY
    cvNamedWindow("cam2", CV_WINDOW_AUTOSIZE);
#endif
#endif

#ifdef DISPLAY
    IplImage* img = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
    int counter = 0;
#endif

///////// main loop

    // start GPS Trigger
#ifndef NOSYNC
    if (useGPS) gpsLogger.Run();
#endif
    int numframes = 0; 
    while (!is_done_working) {
        numframes++; 
        //usleep(1000);
        Image image; 
        cam1->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &buff_1, &d_buff_1);

        if (useGPS) {
            gps_output << gpsLogger.getPacket() << endl;
        }
#ifdef DISPLAY
        counter = (counter + 1) % CAMERA_DISPLAY_SKIP;
        Image cimage; 
        if (counter == 0) {
            image.Convert(PIXEL_FORMAT_BGR, &cimage);
            show(&cimage, img, "cam1");
        }
#endif
#ifdef TWO_CAM
        cam2->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &buff_2, &d_buff_2);
#ifdef DISPLAY
        if (counter == 0) {
            image.Convert(PIXEL_FORMAT_BGR, &cimage);
            show(&cimage, img, "cam2");
        }
        char r = cvWaitKey(1);
        if (r == 'q') is_done_working = true;
#endif // DISPLAY
#endif // TWO_CAM
    }  
    cout << "numframes = " << numframes << endl; 
/////// cleanup
    if (useGPS) gpsLogger.Close();
#ifdef DISPLAY
    cvDestroyWindow("cam1");
    cvDestroyWindow("cam2");
#endif
    consumer_1.stop();
    CloseCamera(cam1);
    delete cam1; 
#ifdef TWO_CAM
    consumer_2.stop();
    CloseCamera(cam2);
    delete cam2;
#endif
    return 0;
}
