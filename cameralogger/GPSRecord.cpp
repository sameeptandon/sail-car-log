#include "GPSRecord.h"

using namespace std;
using namespace boost::algorithm;
using boost::lexical_cast;

GPSRecord::GPSRecord(string record) { 
  valid = false;

  vector<string> tokens;
  split(tokens, record, is_any_of(","));
  if (tokens.size() == 21) {
      seconds          = lexical_cast<double>(tokens[10]);
      latitude         = lexical_cast<double>(tokens[11]);
      longitude        = lexical_cast<double>(tokens[12]);
      height           = lexical_cast<double>(tokens[13]);
      north_velocity   = lexical_cast<double>(tokens[14]);
      east_velocity    = lexical_cast<double>(tokens[15]);
      up_velocity      = lexical_cast<double>(tokens[16]);
      rot_x            = lexical_cast<double>(tokens[17]);
      rot_y            = lexical_cast<double>(tokens[18]);
      azimuth          = lexical_cast<double>(tokens[19]);
      status           = tokens[20];
      valid            = true;
  }

  
}

