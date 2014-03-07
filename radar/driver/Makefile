# List of demos to compile and link
DEVLIST=10A 

CXXFLAGS=-I. -I./include
CLFLAGS=$(DEVLIST:%=-leci%DriverLinux)
CXX=g++

default: LinuxEciDemo UsbCanDemo

LinuxEciDemo: $(DEVLIST:%=EciDemo%.o) EciDemoCommon.o LinuxEciDemo.o
	$(CXX) -o LinuxEciDemo LinuxEciDemo.o EciDemoCommon.o $(DEVLIST:%=EciDemo%.o)  $(CLFLAGS)

LinuxEciDemo.o: LinuxEciDemo.c
	$(CXX) -c $(CXXFLAGS) LinuxEciDemo.c
  
EciDemoCommon.o: EciDemoCommon.c EciDemoCommon.h
	$(CXX) -c $(CXXFLAGS) EciDemoCommon.c

#usbcan.o: usbcan.cpp 
#	$(CXX) -c $(CXXFLAGS) usbcan.cpp

UsbCanDemo.o: UsbCanDemo.c
	$(CXX) -c $(CXXFLAGS) UsbCanDemo.c usbcan.cpp 

UsbCanDemo: EciDemo10A.o usbcan.o UsbCanDemo.o
	$(CXX) -o UsbCanDemo usbcan.o UsbCanDemo.o EciDemoCommon.o $(CLFLAGS)

$(DEVLIST:%=EciDemo%.o): $(DEVLIST:%=EciDemo%.c) $(DEVLIST:%=EciDemo%.h)
	$(CXX) -c $(CXXFLAGS) $(DEVLIST:%=EciDemo%.c)
  
clean:
	rm -f *.o LinuxEciDemo UsbCanDemo
