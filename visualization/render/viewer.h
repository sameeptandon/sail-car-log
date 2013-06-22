#pragma once
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/LineWidth>
#include <osgUtil/ShaderGen>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Math>
#include <cstdlib>
#include <iostream>
#include "io/Parser.h"

using namespace std; 

void printMatrix(osg::Matrix m) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      cout << m(j,i) << " "; 
    }
    cout << endl; 
  }
}

void moveBox (osg::MatrixTransform* box, float x, float y, float z) { 
  osg::Matrix m = box->getMatrix();

  m(3,0) = x;
  m(3,1) = y;
  m(3,2) = z;

  box->setMatrix(m);
}

void moveBox(osg::MatrixTransform* box, osg::Matrix m) {
  box->setMatrix(m);
}

void addLineSegment (osg::Geometry* lines, float x, float y, float z) { 

    osg::Vec3Array* v = (osg::Vec3Array*) lines->getVertexArray();
    v->push_back(osg::Vec3(x,y,z));
    osg::DrawArrays* arr = (osg::DrawArrays*) lines->getPrimitiveSet(0); 
    arr->setFirst(0);
    arr->setCount(v->size());

}

void nextRecord( bool addLines = false ); 
