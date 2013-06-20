#ifndef KEYBOARD_HANDLER_H
#define KEYBOARD_HANDLER_H
#include <osgGA/GUIEventHandler>
class KeyboardEventHandler : public osgGA::GUIEventHandler
{
  public:

    KeyboardEventHandler(osg::MatrixTransform* car, osg::Geometry* lines) {
      _car = car;
      _lines = lines; 
    };

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    virtual void accept(osgGA::GUIEventHandlerVisitor& v)   { v.visit(*this); };

  protected:
    osg::MatrixTransform* _car;
    osg::Geometry* _lines; 
};

bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
  float x = (float)rand() / RAND_MAX;
  float y = (float)rand() / RAND_MAX;
  float z = (float)rand() / RAND_MAX;
  switch(ea.getEventType())
  {
    case(osgGA::GUIEventAdapter::KEYDOWN):
      {
        switch(ea.getKey())
        {
          case 'n':
            std::cout << " n key pressed" << std::endl;
            for (int p = 0 ; p < 100; p ++ ) {
              x = (float)rand() / RAND_MAX;
              y = (float)rand() / RAND_MAX;
              z = (float)rand() / RAND_MAX;

              moveBox(_car, x,y,z);
              addLineSegment(_lines, x,y,z);
            }

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
#endif
