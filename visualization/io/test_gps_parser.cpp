#include "GPSRecord.h"
#include <vector>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "Parser.h"

using namespace std; 

int main(int argc, char** argv) {

  using namespace boost::program_options;
  options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("input,i", value<string>(), "the gps log filename");

  variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);
  notify(vm);
  if (vm.count("help")) { 
    cout << desc << endl;
    return 1;
  }

  string input_name;  
  if (vm.count("input")) {
    input_name = vm["input"].as<string>(); 
  } else {
    cout << desc << endl; 
    return 1; 
  }

  vector<GPSRecord> records = getAllGPSRecords(input_name);
  
  cout << setprecision(15); 
  for (int i = 0; i < records.size(); i++) {
    cout << records[i].seconds << endl;
  }

  return 1;
}
