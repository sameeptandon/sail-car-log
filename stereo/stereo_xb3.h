#include "FlyCapture2.h"
#pragma once

using namespace FlyCapture2;


void setXb3Configuration(Camera* cam) {
  Format7ImageSettings fis;
  fis.mode = MODE_3;
  fis.offsetX = 0;
  fis.offsetY = 0;
  fis.width = 1280;
  fis.height = 960;
  fis.pixelFormat = PIXEL_FORMAT_RGB8;
  
  Error error = cam->SetFormat7Configuration(&fis, 100.0f);

  if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      printf( 
	     "SetFormat7Configuration failed. \n"
	     "Press Enter to exit. \n");
      getchar();
      exit(-1);
    }   
}

PGRGuid getOnlyGuid() {
  BusManager busMgr;
  unsigned int numCameras;
  Error error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK or numCameras > 1)
    {
      printf( "Error deteced %u > 1 cameras\n", numCameras );      
      error.PrintErrorTrace();
      exit(-1);
    }

  PGRGuid guid;
  error = busMgr.GetCameraFromIndex(0, &guid);
  if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      exit(-1);
    }
  return guid;
}


void deinterlace(Image rawImage, Image* right, Image* center, Image* left) {
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
      error.PrintErrorTrace();
      exit(-1);
    }
    
    // set new dimensions
    error = imgs[i]->SetDimensions(rows, cols, cols, PIXEL_FORMAT_RAW8, btf);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      exit(-1);
    }

    // set new buffer
    error = imgs[i]->SetData(buffers[i], rows * cols);
    if (error != PGRERROR_OK)
    {
      error.PrintErrorTrace();
      exit(-1);
    }    
  }
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
