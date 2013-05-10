#include "FlyCapture2.h"
#include <string>
#include <vector>
#include <iostream>
#include "../util/synchronized_buffer.h"
#include "../util/time.h"
#include "../util/fps_calc.h"
#include "Consumer_CV.h"

using namespace FlyCapture2;
using namespace std; 

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

float getFrameRate(Camera *cam) {
    return getProperty(cam, FRAME_RATE);
}
