#include "viewer.h"
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include "viewer_keyboard.h"
#include <boost/program_options.hpp>
using namespace std; 

int record_num = 1;
double cx, cy, cz = 0.0;
vector<GPSRecord> records;
osg::Geometry* lines;
osg::MatrixTransform* box;
osg::MatrixTransform* box_axis;

inline double radians(double deg) {
  return deg * M_PI / 180; 
}

void resetRecord() {
  record_num = 1;
  cx = 0;
  cy = 0; 
  cz = 0; 
}

void nextRecord( bool addLines ) {
  if (record_num < records.size()) { 
    double dx = records[record_num].north_velocity; 
    double dy = records[record_num].east_velocity;
    double dz = records[record_num].up_velocity;
    double dt = records[record_num].seconds - records[record_num-1].seconds;
    cx += dx*dt;
    cy += dy*dt;
    cz += dz*dt;
    if (addLines) 
      addLineSegment(lines, cx, cy, cz);

    double yaw =  M_PI/2 + radians(records[record_num].azimuth);
    double pitch = radians(records[record_num].rot_x);
    double roll = radians(records[record_num].rot_y);

    pitch = radians(records[record_num].rot_y);
    roll = radians(records[record_num].rot_x);
    osg::Matrixd rot;
    rot.makeRotate(yaw  ,osg::Vec3d(0.0,0.0,1.0),
                   pitch,osg::Vec3d(1.0,0.0,0.0),
                   roll ,osg::Vec3d(0.0,1.0,0.0));

    cout << "yaw = " << yaw << endl; 
    cout << "pit = " << pitch << endl;
    cout << "rol = " << roll << endl; 

    cout << "altitude = " << cz;




    osg::Matrixd trans;
    trans.makeTranslate(cx,cy,cz);
    
    moveBox(box, rot*trans);
    record_num++;
    cout << "record num: " << record_num << "/" << records.size() << endl;
  } else { 
    resetRecord();
    nextRecord();
  }
}

osg::Geode* createGeode() {
  osg::Geode* geode = new osg::Geode();
  osg::StateSet* stateset = new osg::StateSet();
  stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
  geode->setStateSet( stateset );
  return geode;
}

osg::MatrixTransform* createBox(float lx = 0, float ly = 0, float lz = 0, 
    osg::Vec4 color = osg::Vec4(1.0,1.0,1.0,1.0)) {
  osg::Box* box; 
  if (lx == 0 && ly == 0 && lz == 0) {
    float radius = 20.f;
    box =  new osg::Box(osg::Vec3(0,0,0),2*radius);
  } else {
    assert(lx > 0 && ly > 0 && lz > 0);
    box = new osg::Box(osg::Vec3(0,0,0), lx,ly,lz);
  }
  osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box); 
  boxDrawable->setColor(color);
  osg::Geode *geode = new osg::Geode;
  geode->addDrawable(boxDrawable); 
  osg::MatrixTransform *mt = new osg::MatrixTransform();
  mt->addChild(geode);
  return mt; 
}

osg::MatrixTransform* createAxis() { 

  float l= 200.0;
  float w= 10.0;

  osg::MatrixTransform* axis = new osg::MatrixTransform; 
  osg::MatrixTransform* xaxis = createBox(l,w,w,osg::Vec4(1.0,0.0,0.0,1.0));
  osg::MatrixTransform* yaxis = createBox(w,l,w,osg::Vec4(0.0,1.0,0.0,1.0));
  osg::MatrixTransform* zaxis = createBox(w,w,l,osg::Vec4(0.0,0.0,1.0,1.0));

  osg::Matrix xt = osg::Matrix::translate(l/2,0,0);
  xaxis->setMatrix(xt);
  osg::Matrix yt = osg::Matrix::translate(0,l/2,0);
  yaxis->setMatrix(yt);
  osg::Matrix zt = osg::Matrix::translate(0,0,l/2);
  zaxis->setMatrix(zt);

  axis->addChild(xaxis);
  axis->addChild(yaxis);
  axis->addChild(zaxis); 

  return axis;
}


osg::Geometry* createLines() { 
  osg::Geometry* geo = new osg::Geometry();
  osg::Vec3Array* v = new osg::Vec3Array;
  geo->setVertexArray(v);
  geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,0));
  geo->setUseDisplayList(false);
  return geo;
}

osg::LineWidth* createLineWidth() { 
  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(10.0f);
  return linewidth;
}

int run(string gpsFileName) {


  // construct the viewer.
  osgViewer::Viewer viewer;

  // add model to viewer.
  osg::Group* root = new osg::Group(); 
  osg::Geode* geode = createGeode();
  box = createBox(20.0, 40.0, 10.0);
  box_axis = createAxis();
  lines = createLines(); 
  osg::LineWidth* linewidth = createLineWidth(); 
  geode->addDrawable(lines); 
  geode->getOrCreateStateSet()->setAttributeAndModes(linewidth,
      osg::StateAttribute::ON);

  box->addChild(box_axis);
  root->addChild(box);
  root->addChild(geode);
  root->addChild(createAxis());

  viewer.setSceneData( root );
  
  // get the records
  records = getAllGPSRecords(gpsFileName);
  for (int t = 1; t < records.size(); t++) {
    nextRecord(true); 
  }
  resetRecord();

  // add keyboard handler to viewer
  KeyboardEventHandler *keh = new KeyboardEventHandler();
  viewer.addEventHandler(keh);
  //osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
  //viewer.setCameraManipulator(tb);
  osgGA::NodeTrackerManipulator* ntm = new osgGA::NodeTrackerManipulator;
  ntm->setTrackNode(box->getChild(0));
  ntm->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
  viewer.setCameraManipulator(ntm);

  viewer.setUpViewInWindow(0,0,800,600);

  viewer.realize();
  while (!viewer.done()) {
    viewer.frame();
  } 
  return 1; 
}

int main(int argc, char ** argv)
{
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

  run(input_name); 
}
