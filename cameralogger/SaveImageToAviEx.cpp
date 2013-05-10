#define TWO_CAM

#include "SaveImageToAviEx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
//boost::mutex io_mutex; 
//SynchronizedBuffer<Image> _buffer(&io_mutex);
bool is_done_working = false; 

void ImageCallback(Image* pImage, const void* pCallbackData, SyncBuffer* buff) { 
    if (!is_done_working) {
        Image* im = new Image();
        pImage->Convert(PIXEL_FORMAT_BGR, im);
        if (!buff->getBuffer()->pushBack(im)) {
            boost::mutex::scoped_lock io_lock ( *(buff->getMutex()) );
            cerr << "Warning! Buffer full, overwriting data!" << endl; 
        }
    }
}

void ImageCallback_1(Image* pImage, const void* pCallbackData) {
    ImageCallback(pImage, pCallbackData, &buff_1); 
    FPS_CALC("image_callback_1", buff_1.getBuffer());
}

void ImageCallback_2(Image* pImage, const void* pCallbackData) {
    ImageCallback(pImage, pCallbackData, &buff_2); 
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

    setProperty(cam, BRIGHTNESS, 0.0f);
    setProperty(cam, AUTO_EXPOSURE, 0.565f);
    setProperty(cam, SHARPNESS, 1024);
    setProperty(cam, SATURATION, 100.0f);
    setProperty(cam, GAMMA, 1.0f);
    //setProperty(cam, SHUTTER, 7.0f);
    setProperty(cam, SHUTTER, 3.0f);
    setProperty(cam, GAIN, 2.25f);

    
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
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
}

int main(int /*argc*/, char** /*argv*/)
{

    signal (SIGINT, ctrlC);
    PrintBuildInfo();

    buff_1.getBuffer()->setCapacity(1000);
    buff_2.getBuffer()->setCapacity(1000);
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
    Consumer<Image> consumer_1(buff_1.getBuffer(), "uncompressed1.avi",
            buff_1.getMutex(), 50.0f, imageWidth, imageHeight ); 
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
    Consumer<Image> consumer_2(buff_2.getBuffer(), "uncompressed2.avi",
            buff_2.getMutex(), getFrameRate(cam2), imageWidth, imageHeight );
#endif

    while (!is_done_working) {
        //usleep(1000);
        Image image; 
        cam1->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &buff_1);
#ifdef TWO_CAM
        cam2->RetrieveBuffer(&image);
        ImageCallback(&image, NULL, &buff_2);
#endif
    }   

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
