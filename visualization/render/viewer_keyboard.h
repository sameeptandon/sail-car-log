#pragma once 

#include <osgGA/GUIEventHandler>
class KeyboardEventHandler : public osgGA::GUIEventHandler
{
  public:

    /*
    KeyboardEventHandler(osg::MatrixTransform* car, osg::Geometry* lines) {
      _car = car;
      _lines = lines; 
    };
    */

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };

  protected:
    //osg::MatrixTransform* _car;
    //osg::Geometry* _lines; 
};

bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
  switch(ea.getEventType())
  {
    case(osgGA::GUIEventAdapter::KEYDOWN):
      {
        switch(ea.getKey())
        {
          case 'n':
            std::cout << " n key pressed" << std::endl;
            for (int p = 0; p  < 10; p++) 
              nextRecord();
            return false;
            break;
          default:
            return false;
        } 
      }
    default:
      return false;
  }
}
