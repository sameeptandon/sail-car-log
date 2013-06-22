#pragma once

#include "GPSRecord.h"
#include <iostream>
#include <string>

vector<GPSRecord> getAllGPSRecords(string logfilename) {

  ifstream file (logfilename.c_str());
  string line;
  vector<GPSRecord> records;
  while (file.good()) { 
    getline(file, line);
    GPSRecord g(line);
    if (g.isValid()) { 
      records.push_back(g);
    }
  }
  return records; 
}


