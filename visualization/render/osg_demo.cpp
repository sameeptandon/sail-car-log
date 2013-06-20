#include "osg_demo.h"
#include <osgGA/TrackballManipulator>
#include "keyboard_handler.h"
using namespace std; 

osg::Geode* createGeode()
{
    osg::Geode* geode = new osg::Geode();

    // ---------------------------------------
    // Set up a StateSet to texture the objects
    // ---------------------------------------
    osg::StateSet* stateset = new osg::StateSet();

    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    
    geode->setStateSet( stateset );
    
    //osg::TessellationHints* hints = new osg::TessellationHints;
    //hints->setDetailRatio(0.5f);

    /*
    int num_pts = 100; 

    for (int i = 0; i < num_pts; i++) {
      float x = (float)rand() / RAND_MAX;
      float y = (float)rand() / RAND_MAX;
      float z = (float)rand() / RAND_MAX;
      v->push_back(osg::Vec3(x,y,z));

    }
    */
    
    /*
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius),hints));
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(4.0f,0.0f,0.0f),radius,height),hints));
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(6.0f,0.0f,0.0f),radius,height),hints));
    geode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(8.0f,0.0f,0.0f),radius,height),hints));
    */
    return geode;
}

osg::MatrixTransform* createBox() { 
    float radius = 0.1f;
    osg::Box* box =  new osg::Box(osg::Vec3(0,0,0),2*radius);
    osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box); 
    osg::Geode *geode = new osg::Geode;
    geode->addDrawable(boxDrawable); 
    osg::MatrixTransform *mt = new osg::MatrixTransform();
    mt->addChild(geode);
    return mt; 
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
    linewidth->setWidth(3.0f);
    return linewidth;
}

int main(int, char **)
{
    // construct the viewer.
    osgViewer::Viewer viewer;


    // add model to viewer.
    osg::Group* root = new osg::Group(); 
    osg::Geode* geode = createGeode();
    osg::MatrixTransform* box = createBox();
    osg::Geometry* lines = createLines(); 
    osg::LineWidth* linewidth = createLineWidth(); 
    geode->addDrawable(lines); 
    geode->getOrCreateStateSet()->setAttributeAndModes(linewidth,
        osg::StateAttribute::ON);

    osg::MatrixTransform* box2 = createBox();
    moveBox(box2, 0.2, 0.2, 0.2);

    root->addChild(box);
    root->addChild(box2);
    root->addChild(geode);

    viewer.setSceneData( root );

    // add keyboard handler to viewer
    KeyboardEventHandler *keh = new KeyboardEventHandler(box, lines);
    viewer.addEventHandler(keh);
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    viewer.setCameraManipulator(tb);
    viewer.setUpViewInWindow(0,0,800,600);

    viewer.realize();
    while (!viewer.done()) {
      viewer.frame();
    }
}
