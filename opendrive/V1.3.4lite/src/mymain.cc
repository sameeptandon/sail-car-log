#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "OdrManagerLite.hh"
using namespace std;

int main(int argc, char ** argv){
	OpenDrive::OdrManagerLite myManager;

	if(!myManager.loadFile(argv[1])){
		cerr << "couldn't load file" << endl;
		exit(1);
	}

	cerr << "main: printing data contents" << endl;
	myManager.printData();

	OpenDrive::Position* myPos = myManager.createPosition();
	myManager.activatePosition(myPos);

	for(double s = 0.0; s < 2300.00; s+= 100.0){
		
		myManager.setTrackPos(1,s,1.0);
		
		bool result = myManager.track2inertial();

		cout << myManager.getInertialPos().getX() << " " << myManager.getInertialPos().getY() << " " << myManager.getInertialPos().getZ();
	}
	return 0;
}
