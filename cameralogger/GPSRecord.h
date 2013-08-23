#pragma once
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

using namespace std;

class GPSRecord {
  public:
    GPSRecord(string record); // parse out a string version of the GPS record

    bool isValid () { return valid; } 
    double seconds;
    double latitude;
    double longitude;
    double height;
    double north_velocity;
    double east_velocity;
    double up_velocity;
    double rot_x;
    double rot_y;
    double azimuth; 
    string status;
  protected:
      bool valid; 
};
