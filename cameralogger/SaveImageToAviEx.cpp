#include "SaveImageToAviEx.h"

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
        im->DeepCopy(pImage);
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
            FRAMERATE_30); 
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }

    // Start capturing images
    printf( "Starting capture... \n" );
    error = cam->StartCapture(callback);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

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

float getFrameRate(Camera* cam) { 

    Error error;

    // Check if the camera supports the FRAME_RATE property
    printf( "Detecting frame rate from camera... \n" );
    PropertyInfo propInfo;
    propInfo.type = FRAME_RATE;
    error = cam->GetPropertyInfo( &propInfo );
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    } 


    float frameRateToUse = 15.0f;
    if ( propInfo.present == true )
    {
        // Get the frame rate
        Property prop;
        prop.type = FRAME_RATE;
        error = cam->GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
        frameRateToUse = prop.absValue;
    }

    return frameRateToUse; 

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
    
    error = busMgr.GetCameraFromIndex(1, &guid);
    if (error != PGRERROR_OK)
    {
       PrintError(error);
       return -1;
    }
    Camera* cam2 = CreateCamera(&guid); 
    ImageEventCallback c1 = &ImageCallback_1;
    ImageEventCallback c2 = &ImageCallback_2;
    RunCamera( cam1, c1);
    RunCamera( cam2, c2); 

    Consumer<Image> consumer_1(buff_1.getBuffer(), "uncompressed1", buff_1.getMutex(),
            getFrameRate(cam1), imageWidth, imageHeight ); 
    Consumer<Image> consumer_2(buff_2.getBuffer(), "uncompressed2", buff_2.getMutex(),
            getFrameRate(cam1), imageWidth, imageHeight );

    while (!is_done_working)
        usleep(1000);

    consumer_1.stop();
    consumer_2.stop();
    CloseCamera(cam1);
    CloseCamera(cam2);
    delete cam1; 
    delete cam2; 
    return 0;
}
