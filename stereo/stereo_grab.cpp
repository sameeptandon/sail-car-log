#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "FlyCapture2.h"

using namespace FlyCapture2;

void PrintBuildInfo()
{
  FC2Version fc2Version;
  Utilities::GetLibraryVersion( &fc2Version );
  char version[128];
  sprintf( 
	  version, 
	  "FlyCapture2 library version: %d.%d.%d.%d\n", 
	  fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

  printf( version );

  char timeStamp[512];
  sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

  printf( timeStamp );
}

void PrintImageInfo( Image* image ) {
  
  printf(
	 "Image size bytes %d, rows %d, cols %d, stride %d, bayer format %d , pix format %x\n",
	 image->GetDataSize(), image->GetRows(),
	 image->GetCols(), image->GetStride(),
	 image->GetBayerTileFormat(), image->GetPixelFormat() ); 
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
  printf(
	 "\n*** CAMERA INFORMATION ***\n"
	 "Serial number - %u\n"
	 "Camera model - %s\n"
	 "Camera vendor - %s\n"
	 "Sensor - %s\n"
	 "Resolution - %s\n"
	 "Firmware version - %s\n"
	 "Firmware build time - %s\n\n",
	 pCamInfo->serialNumber,
	 pCamInfo->modelName,
	 pCamInfo->vendorName,
	 pCamInfo->sensorInfo,
	 pCamInfo->sensorResolution,
	 pCamInfo->firmwareVersion,
	 pCamInfo->firmwareBuildTime );
}

void PrintError( Error error )
{
  error.PrintErrorTrace();
}

void Deinterlace(Image rawImage, Image* right, Image* center, Image* left) {
  unsigned int cols = rawImage.GetCols();
  unsigned int rows = rawImage.GetRows();
  unsigned int stride = rawImage.GetStride();
  BayerTileFormat btf = rawImage.GetBayerTileFormat();
  unsigned int rawDataSize = rawImage.GetDataSize();
  unsigned char* rawData = rawImage.GetData();
  
  // sanity checks
  if (stride % cols != 0 or stride/cols != 3 or (rows*stride) != rawDataSize) {
    printf("Error: raw image does not have three bayer tiles");
    exit(-1);
  }

  if (rawImage.GetPixelFormat() != PIXEL_FORMAT_RGB8) {
    printf("Raw image should be PIXEL_FORMAT_RGB8");
    exit(-1);
  }

  // allocate our data for our new buffers
  std::vector<unsigned char*> buffers(3,NULL);
  for (unsigned int b = 0; b < buffers.size(); b++)
    buffers[b] = new unsigned char[rows*cols];

  for (unsigned int p = 0; p < rawDataSize; p++)
    buffers[p%3][p/3] = rawData[p];
  
  // transfer data to provided images
  std::vector<Image*> imgs(3,NULL);  
  imgs[0] = left; imgs[1] = center; imgs[2] = right;  
  Error error;
  for (int i = 0; i < imgs.size(); i++) {
    // destory anything previously held
    error = imgs[i]->ReleaseBuffer();
    if (error != PGRERROR_OK)
    {
      PrintError( error );
      exit(-1);
    }
    
    // set new dimensions
    error = imgs[i]->SetDimensions(rows, cols, cols, PIXEL_FORMAT_RAW8, btf);
    if (error != PGRERROR_OK)
    {
      PrintError( error );
      exit(-1);
    }

    // set new buffer
    error = imgs[i]->SetData(buffers[i], rows * cols);
    if (error != PGRERROR_OK)
    {
      PrintError( error );
      exit(-1);
    }    
  }
}

int RunSingleCamera( PGRGuid guid )
{
  const int k_numImages = 5;

  Error error;
  Camera cam;

  // Connect to a camera
  error = cam.Connect(&guid);
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }

  // Get the camera information
  CameraInfo camInfo;
  error = cam.GetCameraInfo(&camInfo);
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }

  PrintCameraInfo(&camInfo);  

  error = cam.SetVideoModeAndFrameRate( 
  				       VIDEOMODE_1280x960Y8, 
  				       FRAMERATE_1_875 );
  
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      printf( 
	     "Error2 starting cameras. \n"
	     "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
	     "If your camera does not support this mode, please edit the source code and recompile the application. \n"
	     "Press Enter to exit. \n");
      getchar();
      return -1;
    }


  Format7ImageSettings fis;
  fis.mode = MODE_3;
  fis.offsetX = 0;
  fis.offsetY = 0;
  fis.width = 1280;
  fis.height = 960;
  fis.pixelFormat = PIXEL_FORMAT_RGB8;
  
  error = cam.SetFormat7Configuration(&fis, 100.0f);

  if (error != PGRERROR_OK)
    {
      PrintError( error );
      printf( 
	     "SetFormat7Configuration failed. \n"
	     "Press Enter to exit. \n");
      getchar();
      return -1;
    }    

  
  // Start capturing images
  error = cam.StartCapture();
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }

  Image rawImage;

  for ( int imageCnt=0; imageCnt < k_numImages; imageCnt++ )
    {                
      // Retrieve an image
      error = cam.RetrieveBuffer( &rawImage );
      if (error != PGRERROR_OK)
        {
	  PrintError( error );
	  continue;
        }

      printf( "Grabbed image %d\n", imageCnt );
      
      // get other images
      Image right, center, left;
      Deinterlace(rawImage, &right, &center, &left);

      // Create a unique filename
      char lfilename[512];
      char cfilename[512];
      char rfilename[512];      
      sprintf( lfilename, "left-%d.ppm", imageCnt );
      sprintf( cfilename, "center-%d.ppm", imageCnt );
      sprintf( rfilename, "right-%d.ppm", imageCnt );

      // Save the image. If a file format is not passed in, then the file
      // extension is parsed to attempt to determine the file format.      
      error = left.Save( lfilename );
      if (error != PGRERROR_OK)
        {
	  PrintError( error );
	  return -1;
        }
      
      error = center.Save( cfilename );
      if (error != PGRERROR_OK)
        {
	  PrintError( error );
	  return -1;
        }
      
      error = right.Save( rfilename );
      if (error != PGRERROR_OK)
        {
	  PrintError( error );
	  return -1;
        }        
    }            

  // Stop capturing images
  error = cam.StopCapture();
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }      

  // Disconnect the camera
  error = cam.Disconnect();
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }

  return 0;
}

int main(int /*argc*/, char** /*argv*/)
{    
  PrintBuildInfo();

  Error error;

  // Since this application saves images in the current folder
  // we must ensure that we have permission to write to this folder.
  // If we do not have permission, fail right away.
  FILE* tempFile = fopen("test.txt", "w+");
  if (tempFile == NULL)
    {
      printf("Failed to create file in current folder.  Please check permissions.\n");
      return -1;
    }
  fclose(tempFile);
  remove("test.txt");

  BusManager busMgr;
  unsigned int numCameras;
  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }

  printf( "Number of cameras detected: %u\n", numCameras );

  for (unsigned int i=0; i < numCameras; i++)
    {
      PGRGuid guid;
      error = busMgr.GetCameraFromIndex(i, &guid);
      if (error != PGRERROR_OK)
        {
	  PrintError( error );
	  return -1;
        }

      RunSingleCamera( guid );
    }

  printf( "Done! Press Enter to exit...\n" );
  getchar();

  return 0;
}
