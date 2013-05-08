#include "SaveImageToAviEx.h"

int numImages = 180;
vector<Image> buffer(numImages);
int imageCount = 0;
boost::mutex io_mutex; 
SynchronizedBuffer<Image> _buffer(&io_mutex);
bool is_done_working = false; 



void ImageCallback(Image* pImage, const void* pCallbackData) {
    //cout << "Image Callback" << imageCount << endl;
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

int RunCamera( PGRGuid guid )
{
    Error error;
    Camera cam;

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    PrintCameraInfo(&camInfo);
    
    int imageWidth = 1280;
    int imageHeight = 960;

    error = cam.SetVideoModeAndFrameRate(VIDEOMODE_1280x960YUV422,
            FRAMERATE_30); 
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }

    _buffer.setCapacity(numImages+20);

    // Start capturing images
    printf( "Starting capture... \n" );
    error = cam.StartCapture(ImageCallback);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }


    // Check if the camera supports the FRAME_RATE property
    printf( "Detecting frame rate from camera... \n" );
    PropertyInfo propInfo;
    propInfo.type = FRAME_RATE;
    error = cam.GetPropertyInfo( &propInfo );
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
        error = cam.GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
        frameRateToUse = prop.absValue;
    }

    usleep(2000);

    Consumer<Image> consumer(_buffer, "uncompressed", &io_mutex, frameRateToUse, imageWidth, imageHeight ); 

    while (!is_done_working) { 
        usleep(100); 
    }

    // Stop capturing images
    printf( "Stopping capture... \n" );
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }



    /*
    printf("Using frame rate of %3.1f\n", frameRateToUse);

    char aviFileName[512] = {0};

    sprintf(aviFileName, "Uncompressed");
    SaveAviHelper(UNCOMPRESSED, buffer, aviFileName, frameRateToUse);
    */
    /*
    sprintf(aviFileName, "SaveImageToAviEx-Mjpg-%u", camInfo.serialNumber);
    SaveAviHelper(MJPG, buffer, aviFileName, frameRateToUse);

    sprintf(aviFileName, "SaveImageToAviEx-h264-%u", camInfo.serialNumber);
    SaveAviHelper(H264, buffer, aviFileName, frameRateToUse);
    */

    consumer.stop();
    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    return 0;
}

void 
ctrlC (int)
{
  boost::mutex::scoped_lock io_lock (io_mutex);
  printf("\nCtrl-C detected, exit condition set to true.\n");
  is_done_working = true;
}

int main(int /*argc*/, char** /*argv*/)
{

    signal (SIGINT, ctrlC);
    PrintBuildInfo();

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

    printf( "Running the first camera.\n" );
    RunCamera( guid );

    /*
    printf( "Done! Press Enter to exit...\n" );
    getchar();
    */

    return 0;
}
