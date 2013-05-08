#include "SaveImageToAviEx.h"

int numImages = 180;
vector<Image> buffer(numImages);
int imageCount = 0;
boost::mutex io_mutex; 
SynchronizedBuffer<Image> _buffer(&io_mutex);
bool is_done = false; 


//////////////////////////////////////////////////////////////////////////////////////////
// Consumer thread class
template <typename T>
class Consumer
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////
    void 
    writeToDisk (const T* obj)
    {
      FPS_CALC ("cloud write.", buf_);
      Error error;
      error = aviRecorder_.AVIAppend((T*)obj);
      if (error != PGRERROR_OK) 
          PrintError(error);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Consumer thread function
    void 
    receiveAndProcess ()
    {
      while (true)
      {
        if (is_done) { 
          break;
        }
        writeToDisk (buf_.getFront ());
      }

      {
        boost::mutex::scoped_lock io_lock (io_mutex);
        printf("Writing remaing %d clouds in the buffer to disk...\n", buf_.getSize ());
      }
      while (!buf_.isEmpty ())
        writeToDisk (buf_.getFront ());
    }

  public:
    Consumer (SynchronizedBuffer<T> &buf, std::string aviFileName, float frameRate, int imWidth, int imHeight)
      : buf_ (buf), aviFileName_(aviFileName), frameRate_(frameRate)
    {
      thread_.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));

      Error error;

      // Uncompressed Options
      AVIOption option;

      //MJPG Compressed Options
      //MJPGOption option;
      //option.quality = 100;
      
      //H264 Compressed Options
      //H264Option option;
      //option.bitrate = 1000000;
      //option.height = imHeight;
      //option.width = imWidth;
      
      cout << "frameRate = " << frameRate_ << endl; 
      option.frameRate = frameRate_;
      error = aviRecorder_.AVIOpen(aviFileName_.c_str(), &option); 
      if (error != PGRERROR_OK) { 
          PrintError(error);
      }
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      printf("stop called\n");
      thread_->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);
      aviRecorder_.AVIClose();
      printf("Consumer done.\n");
    }

  private:
    SynchronizedBuffer<T> &buf_;
    boost::shared_ptr<boost::thread> thread_;
    string aviFileName_; 
    float frameRate_;
    AVIRecorder aviRecorder_; 
};

//////////////////////////////////////////////////////////////////////////////////////////

void ImageCallback(Image* pImage, const void* pCallbackData) {
    //cout << "Image Callback" << imageCount << endl;
    Image* im = new Image();
    im->DeepCopy(pImage);
    if (!_buffer.pushBack(im)) {
        boost::mutex::scoped_lock io_lock (io_mutex);
        cerr << "Warning! Buffer full, overwriting data!" << endl; 
    }
    FPS_CALC("image_callback", _buffer);
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
            FRAMERATE_60); 
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

    Consumer<Image> consumer(_buffer, "uncompressed", frameRateToUse, imageWidth, imageHeight ); 

    while (!is_done) { 
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
  is_done = true;
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
