#pragma once

#include "FlyCapture2.h"
#include <string>
#include <vector>
#include <iostream>

using namespace FlyCapture2;
using namespace std; 

enum AviType
{
    UNCOMPRESSED,
    MJPG,
    H264
};


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

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

void SaveAviHelper(
    AviType aviType,
    std::vector<Image>& vecImages,
    std::string aviFileName,
    float frameRate)
{
    Error error;
    AVIRecorder aviRecorder;

    // Open the AVI file for appending images

    switch (aviType)
    {
    case UNCOMPRESSED:
        {
            AVIOption option;
            option.frameRate = frameRate;
            error = aviRecorder.AVIOpen(aviFileName.c_str(), &option);
        }
        break;
    case MJPG:
        {
            MJPGOption option;
            option.frameRate = frameRate;
            option.quality = 75;
            error = aviRecorder.AVIOpen(aviFileName.c_str(), &option);
        }
        break;
    case H264:
        {
            H264Option option;
            option.frameRate = frameRate;
            option.bitrate = 1000000;
            option.height = vecImages[0].GetRows();
            option.width = vecImages[0].GetCols();
            error = aviRecorder.AVIOpen(aviFileName.c_str(), &option);
        }
        break;
    }

    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    printf( "\nAppending %d images to AVI file: %s ... \n", (int) vecImages.size(), aviFileName.c_str() );
    for (int imageCnt = 0; imageCnt < (int) vecImages.size(); imageCnt++)
    {
        // Append the image to AVI file
        error = aviRecorder.AVIAppend(&vecImages[imageCnt]);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            continue;
        }

        //printf("Appended image %d...\n", imageCnt);
    }

    // Close the AVI file
    error = aviRecorder.AVIClose( );
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }
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
    prop.onOff = false;
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
    prop.absControl = true;
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
