#define TWO_CAM
//#define DISPLAY
#define DEVDISPLAY

#include "SaveImageToAviEx.h"

#define SHUTTER_PARAM (190)

#ifdef DEVDISPLAY
void show (const Image* obj, IplImage* img, string name) {
    IplImage* _img = img;
    //_img = cvCreateImage(cvSize(obj->GetCols(), obj->GetRows()), IPL_DEPTH_8U, 3);
    _img->height = obj->GetRows();
    _img->width = obj->GetCols();
    _img->widthStep = obj->GetStride();
    _img->nChannels = 3;
    _img->imageData = (char*)obj->GetData();
    imshow(name.c_str(), Mat(_img));

    //cvShowImage(_name.c_str(), _img);
    //cvWaitKey(1);
    //if (k == 'r')
    //    imshow(_name.c_str(), Mat(_img));
    //	k = cvWaitKey(15);
    //sleep(0.001);
    //cvReleaseImage(&_img);
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
#ifdef DISPLAY
        im = new Image();
        pImage->Convert(PIXEL_FORMAT_BGR, im);
        if (!d_buff->getBuffer()->pushBack(im)) {
            boost::mutex::scoped_lock io_lock ( *(d_buff->getMutex()) );
            cerr << "Warning! Buffer full, overwriting data!" << endl; 
        }
#endif
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

Camera* CreateCamera( PGRGuid* guid ) {
    Error error;  
    Camera *cam = new Camera();
    
    error = cam->Connect(guid);
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
    pConfig.numBuffers = 10;
    pConfig.isochBusSpeed = BUSSPEED_S5000;
    pConfig.asyncBusSpeed = BUSSPEED_S5000;
    pConfig.highPerformanceRetrieveBuffer = true;
    cam->SetConfiguration(&pConfig);

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
    
    /* 
    TriggerMode mTrigger;
    mTrigger.mode = 15; 
    mTrigger.source = 0; 
    mTrigger.parameter = 60; 
    mTrigger.onOff = true; 
    mTrigger.polarity = 0; 
    error = cam->SetTriggerMode(&mTrigger); 
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }
    */
    

    float p = getProperty(cam, SHUTTER);
    cout << " shutter = "  << p << endl; 
    // Start capturing images
    printf( "Starting capture... \n" );
    //error = cam->StartCapture(callback);
   
    error = cam->StartCapture();
    /*
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
    for (int i = 0; i < 10; i++) { 
        Image image; 
        cam->RetrieveBuffer(&image); 
        Image* im = new Image();
        im->DeepCopy(&image);
        buff_1.getBuffer()->pushBack(im);
        cout << " go yo frame " << endl; 
    }
    */


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
#ifndef DEVDISPLAY
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
#endif
#ifdef DEVDISPLAY
  printf("\nCtrl-C Disabled! Use 'q' to quit instead\n");
#endif
}

int main(int argc, char** argv)
{
    using namespace boost::program_options;
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
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
    if (vm.count("output")) { 
        fname1 = vm["output"].as<string>() + "1.avi";
        fname2 = vm["output"].as<string>() + "2.avi";
    }
    else {
        fname1 = "data1.avi";
        fname2 = "data2.avi";
    }

    cout  << "Filenames: " << fname1 << " " << fname2 << endl; 

    signal (SIGINT, ctrlC);
    //PrintBuildInfo();

    buff_1.getBuffer()->setCapacity(1000);
    buff_2.getBuffer()->setCapacity(1000);
    d_buff_1.getBuffer()->setCapacity(1000);
    d_buff_2.getBuffer()->setCapacity(1000);

    int imageWidth = 1280;
    int imageHeight = 960;

    Error error;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    if ( numCameras < 1 )
    {
       printf( "No camera detected.\n" );
       return -1;
    }
    else
    {
       printf( "Number of cameras detected: %u\n", numCameras );
    }

    PGRGuid guid;
    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
       PrintError(error);
       return -1;
    }
    Camera* cam1 = CreateCamera(&guid); 
    ImageEventCallback c1 = &ImageCallback_1;
    RunCamera( cam1, c1);

    //printf("%x, %x\n", readRegister(cam1, 0x830), readRegister(cam1, 0x834));
    //return 0;

    Consumer<Image> consumer_1(buff_1.getBuffer(), fname1,
            buff_1.getMutex(), 50.0f, imageWidth, imageHeight ); 
#ifdef DISPLAY
    Display<Image> display_1(d_buff_1.getBuffer(), "cam1", 
            d_buff_1.getMutex());
#endif
#ifdef DEVDISPLAY
    cvNamedWindow("cam1", CV_WINDOW_AUTOSIZE);
#endif

#ifdef TWO_CAM
    error = busMgr.GetCameraFromIndex(1, &guid);
    if (error != PGRERROR_OK)
    {
       PrintError(error);
       return -1;
    }
    Camera* cam2 = CreateCamera(&guid); 
    ImageEventCallback c2 = &ImageCallback_2;
    RunCamera( cam2, c2); 
    Consumer<Image> consumer_2(buff_2.getBuffer(), fname2,
            buff_2.getMutex(), 50.0f, imageWidth, imageHeight );

#ifdef DEVDISPLAY
    cvNamedWindow("cam2", CV_WINDOW_AUTOSIZE);
#endif
#ifdef DISPLAY
    Display<Image> display_2(d_buff_2.getBuffer(), "cam2", 
            d_buff_2.getMutex());
#endif // DISPLAY 
#endif

#ifdef DEVDISPLAY
    IplImage* img = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
    int counter = 0;
#endif
    while (!is_done_working) {
        //usleep(1000);
        Image image; 
        cam1->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &buff_1, &d_buff_1);
#ifdef DEVDISPLAY
        counter = (counter + 1) % 2;
        Image cimage; 
        if (counter == 0) {
            image.Convert(PIXEL_FORMAT_BGR, &cimage);
            show(&cimage, img, "cam1");
        }
#endif
#ifdef TWO_CAM
        cam2->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &buff_2, &d_buff_2);
#ifdef DEVDISPLAY
        if (counter == 0) {
            image.Convert(PIXEL_FORMAT_BGR, &cimage);
            show(&cimage, img, "cam2");
	}
	char r = cvWaitKey(1);
	if (r == 'q') is_done_working = true;
#endif
#endif
    }   
#ifdef DEV_DISPLAY
    cvDestroyWindow("cam1");
    cvDestroyWindow("cam2");
#endif
    consumer_1.stop();
#ifdef DISPLAY
    display_1.stop();
#endif
    CloseCamera(cam1);
    delete cam1; 
#ifdef TWO_CAM
    consumer_2.stop();
#ifdef DISPLAY
    display_2.stop();
#endif
    CloseCamera(cam2);
    delete cam2;
#endif
    return 0;
}
