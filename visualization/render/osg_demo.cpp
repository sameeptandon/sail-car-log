#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/LineWidth>
#include <osgUtil/ShaderGen>

#include <osgViewer/Viewer>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osg/Math>
#include <cstdlib>
#include <iostream>
#include "keyboard_handler.h"

using namespace std; 

osg::Geode* createShapes()
{
    osg::Geode* geode = new osg::Geode();

    cout << "Hello there" << endl; 
    
    
    // ---------------------------------------
    // Set up a StateSet to texture the objects
    // ---------------------------------------
    osg::StateSet* stateset = new osg::StateSet();

    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    
    geode->setStateSet( stateset );

    
    float radius = 0.1f;
    float height = 1.0f;
    
    osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(0.5f);

    osg::Geometry* geo = new osg::Geometry();
    osg::Vec3Array* v = new osg::Vec3Array;
   
    int num_pts = 100; 

    for (int i = 0; i < num_pts; i++) {
      float x = (float)rand() / RAND_MAX;
      float y = (float)rand() / RAND_MAX;
      float z = (float)rand() / RAND_MAX;
      v->push_back(osg::Vec3(x,y,z));

      if ( i == num_pts - 1 ) 
        geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(x,y,z),2*radius),hints));
    }

    geo->setVertexArray(v);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,num_pts));
    geode->addDrawable(geo);
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(3.0f);
    geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON); 

    /*
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius),hints));
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(4.0f,0.0f,0.0f),radius,height),hints));
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(6.0f,0.0f,0.0f),radius,height),hints));
    geode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(8.0f,0.0f,0.0f),radius,height),hints));
    */
    return geode;
}

int main(int, char **)
{
    // construct the viewer.
    osgViewer::Viewer viewer;

    KeyboardEventHandler *keh = new KeyboardEventHandler();

    // add model to viewer.
    viewer.setSceneData( createShapes() );

    // add keyboard handler to viewer
    viewer.addEventHandler(keh);
    
    viewer.setUpViewInWindow(0,0,800,600);

    return viewer.run();
}
