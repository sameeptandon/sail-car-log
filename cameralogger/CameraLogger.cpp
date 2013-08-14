#define TWO_CAM
#define DISPLAY
//#define NOSYNC

#include <fstream>
#include "CameraLogger.h"
#include "GPSLogger.h"

#define SHUTTER_PARAM (190)
#define CAMERA_DISPLAY_SKIP 3
#define NUMTHREAD_PER_BUFFER 10

#ifdef DISPLAY
void show (const Image* obj, IplImage* img, string name) {
    IplImage* _img = img;
    _img->height = obj->GetRows();
    _img->width = obj->GetCols();
    _img->widthStep = obj->GetStride();
    _img->nChannels = 3;
    _img->imageData = (char*)obj->GetData();
    Mat im_out;
    resize(Mat(_img), im_out, Size(320,240));
    imshow(name.c_str(), im_out);
    //imshow(name.c_str(), Mat(_img));
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

SyncBuffer cam1_buff[NUMTHREAD_PER_BUFFER];
SyncBuffer cam2_buff[NUMTHREAD_PER_BUFFER];

bool is_done_working = false; 

void ImageCallback(Image* pImage, const void* pCallbackData, 
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

void ImageCallback_1(Image* pImage, const void* pCallbackData) {
    assert(false);
}

void ImageCallback_2(Image* pImage, const void* pCallbackData) {
    assert(false);
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
#ifndef DISPLAY
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
#endif
#ifdef DISPLAY
  //printf("\nCtrl-C Disabled! Use 'q' to quit instead\n");
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
#endif
}

void stopSig (int)
{
  is_done_working = true;
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

    uint64_t maxframes = -1; 
    if (vm.count("maxframes")) {
        maxframes = vm["maxframes"].as<uint64_t>(); 
    }
    cout << "Capturing for maximum of " << maxframes << " frames" << endl; 

    bool useGPS = vm.count("serial");
    GPSLogger gpsLogger;
    ofstream gps_output;
    if (useGPS) {
        gps_output.open(fnamegps.c_str());
        gpsLogger.Connect(vm["serial"].as<string>());
    }

    cout  << "Filenames: " << fname1 << " " << fname2 << endl; 

    signal (SIGINT, ctrlC);
    signal (SIGSTOP, stopSig);

    for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++)
    {
        cam1_buff[thread_num].getBuffer()->setCapacity(1000);
        cam2_buff[thread_num].getBuffer()->setCapacity(1000);
    }

    //buff_1.getBuffer()->setCapacity(1000);
    //buff_2.getBuffer()->setCapacity(1000);

    int imageWidth = 1280;
    int imageHeight = 960;

#ifdef NOSYNC
    cerr << "WARNING: SYNC IS DISABLED" << endl;
#endif

    Camera* cam1 = ConnectCamera(0); // 0 indexing
    assert(cam1 != NULL);
    ImageEventCallback c1 = &ImageCallback_1;
    RunCamera( cam1, c1);

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

#ifdef DISPLAY
    cvNamedWindow("cam1", CV_WINDOW_AUTOSIZE);
#endif

#ifdef TWO_CAM
    Camera* cam2 = ConnectCamera(1); // 0 indexing 
    assert(cam2 != NULL);
    ImageEventCallback c2 = &ImageCallback_2;
    RunCamera( cam2, c2); 
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
    uint64_t numframes = 0; 
    while (!is_done_working) {
        numframes++;
        if (numframes > maxframes) { 
            is_done_working = true;
        }
        Image image; 
        cam1->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &cam1_buff[numframes % NUMTHREAD_PER_BUFFER]);

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
        ImageCallback(&image, NULL, &cam2_buff[numframes % NUMTHREAD_PER_BUFFER]);
#ifdef DISPLAY
        if (counter == 0) {
            image.Convert(PIXEL_FORMAT_BGR, &cimage);
            show(&cimage, img, "cam2");
        }
        char r = cvWaitKey(1);
        if (r == 'q') is_done_working = true;
#endif // DISPLAY
#endif // TWO_CAM

        if (useGPS) {
            gps_output << gpsLogger.getPacket() << endl;
        }
    }  
    cout << "numframes = " << numframes << endl;


/////// cleanup
    if (useGPS) gpsLogger.Close();
#ifdef DISPLAY
    cvDestroyWindow("cam1");
    cvDestroyWindow("cam2");
#endif
    for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++) {
        cam1_consumer[thread_num]->stop();
        delete cam1_consumer[thread_num];
    }
    CloseCamera(cam1);
    delete cam1; 
#ifdef TWO_CAM
    for (int thread_num = 0; thread_num < NUMTHREAD_PER_BUFFER; thread_num++) {
        cam2_consumer[thread_num]->stop();
        delete cam2_consumer[thread_num];
    }
    CloseCamera(cam2);
    delete cam2;
#endif
    return 0;
}
