#pragma once

#include "FlyCapture2.h"
#include <string>
#include <vector>
#include <iostream>
#define SHUTTER_PARAM (190)

using namespace FlyCapture2;
using namespace std; 

void PrintError( Error error )
{
    error.PrintErrorTrace();
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

    printf( "%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( "%s", timeStamp );
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



float getProperty(Camera* cam, PropertyType p) {

    Error error;

    // Check if the camera supports the FRAME_RATE property
    printf( "Detecting frame rate from camera... \n" );
    PropertyInfo propInfo;
    propInfo.type = p;
    error = cam->GetPropertyInfo( &propInfo );
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    } 

    if ( propInfo.present == true )
    {
        // Get the frame rate
        Property prop;
        prop.type = p;
        error = cam->GetProperty( &prop );
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
        return prop.absValue;
    }
    return -1; 

}

void setProperty(Camera* cam, PropertyType p, float val) {
    Error error;
    Property prop;
    prop.type = p;
    prop.absControl = true;
    prop.present = true; 
    prop.onOff = true;
    prop.autoManualMode = false; 
    prop.absValue = val;
    error = cam->SetProperty( &prop );
    if ( error != PGRERROR_OK ) {
        PrintError(error);
    }
}

void setWhiteBalance(Camera* cam, int red, int blue) {
    Error error;
    Property prop;
    prop.type = WHITE_BALANCE;
    prop.absControl = false;
    prop.present = true; 
    prop.onOff = false;
    prop.autoManualMode = false; 
    prop.valueA = red;
    prop.valueB = blue;
    error = cam->SetProperty( &prop );
    if ( error != PGRERROR_OK ) {
        PrintError(error);
    }
}

unsigned int readRegister(Camera* cam, unsigned int address) {
    unsigned int pValue;
    cam->ReadRegister(address, &pValue);
    return pValue;
}

void writeRegister(Camera* cam, unsigned int address, unsigned int value) {
    cam->WriteRegister(address, value);
}

float getFrameRate(Camera *cam) {
    return getProperty(cam, FRAME_RATE);
}

int RunCamera(Camera* cam) { 
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
    //setProperty(cam, GAIN, 0.0);
    //setProperty(cam, SHUTTER, 3.0);

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
    error = cam->StartCapture();

    return 0;
}

int CloseCamera( Camera* cam) {
    Error error; 

    printf( "Stopping capture... \n" );
    error = cam->StopCapture();
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }
    
    error = cam->Disconnect();
    if (error != PGRERROR_OK) {
        PrintError(error);
        return -1;
    }

    return 0;
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


