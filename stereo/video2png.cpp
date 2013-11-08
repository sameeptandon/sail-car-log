
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void save2pngs(string videoName, string outdir, string postFix) {
  mkdir(outdir.c_str(), 0755);  
  CvCapture* capture = cvCreateFileCapture(videoName.c_str());
  IplImage* frame; 
  unsigned int ind = 0;  
  while (true) {
    cout << "processing: " << ind << endl;
    frame = cvQueryFrame(capture);
    if (!frame) break;

    stringstream ss;
    ss << outdir << ind << postFix;
    cvSaveImage(ss.str().c_str(), frame);
    ind++;
  }
}

void save2avi(string infile, string outfile) {

  VideoWriter writer = VideoWriter(outfile.c_str(),
				   CV_FOURCC('F','M','P','4'),
				   3.75f,
				   cvSize(1280, 960));

  if (!writer.isOpened()) {
    cerr << "File not opened!" << endl; 
  }  

  
  CvCapture* capture = cvCreateFileCapture(infile.c_str());
  if (!capture) {
    std::cerr << "File: " << infile << " not found" << std::endl;
    exit(-1);
  }
    
  IplImage* frame; 
  unsigned int ind = 0;  
  //while (true) {
  for (int i = 0; i < 4900; i++) {
    cout << "processing: " << ind << endl;
    frame = cvQueryFrame(capture);
    if (!frame) break;

    // possible memory leak
    Mat imgMat(frame);
    writer << imgMat;
    ind++;
  }
};


int main(int argc, char* argv[]) {
  vector<string> names(1);
  names[0] = "left"; //names[1] = "center"; names[2] = "right";
  string prefix = "./recordings/2013-09-13_11:23_";
  // string outprefix = "./recordings/2013-09-13_11:23_2_";
  
  
  for (int i = 0; i < names.size(); i++)
    save2pngs(prefix + names[i] + ".avi", "./images/", "_"+names[i]+".png");
  //save2avi(inprefix + names[i] + ".avi", outprefix+ names[i]+".avi");
  

  
  return 0;
}
