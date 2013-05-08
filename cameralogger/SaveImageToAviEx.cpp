#include "SaveImageToAviEx.h"

boost::mutex io_mutex; 
SynchronizedBuffer<Image> _buffer(&io_mutex);
bool is_done_working = false; 

void ImageCallback(Image* pImage, const void* pCallbackData) {
    if (!is_done_working) {
        Image* im = new Image();
        im->DeepCopy(pImage);
        if (!_buffer.pushBack(im)) {
            boost::mutex::scoped_lock io_lock (io_mutex);
            cerr << "Warning! Buffer full, overwriting data!" << endl; 
        }

        FPS_CALC("image_callback", _buffer);
    }
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

int RunCamera( Camera* cam) { 
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
    error = cam->StartCapture(ImageCallback);
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
  boost::mutex::scoped_lock io_lock (io_mutex);
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
}

int main(int /*argc*/, char** /*argv*/)
{

    signal (SIGINT, ctrlC);
    PrintBuildInfo();

    _buffer.setCapacity(1000);
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

    RunCamera( cam1 );
    RunCamera( cam2 ); 

    Consumer<Image> consumer(_buffer, "uncompressed", &io_mutex, getFrameRate(cam1), imageWidth, imageHeight ); 

    while (!is_done_working)
        usleep(1000);

    consumer.stop();
    CloseCamera(cam1);
    CloseCamera(cam2);
    delete cam1; 
    delete cam2; 
    return 0;
}
