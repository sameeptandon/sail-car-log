#include <iostream>
//#include <global.h>
//#include <vlrException.h>
#include <usbcan.h>

int main(int argc, char **argv) {
  vlr::UsbCan usbcan;
  unsigned char message[100];
  int i, message_length = 0;
  unsigned int can_id;

  if(argc < 2) {
    std::cout << "Error: not enough arguments.\n" << "Usage: " << argv[0] << "<device>\n";
  }

  try {
    usbcan.open(argv[1]);
  }
  catch(VLRException& e) {
    std::cout << e.what();
  }

  while(1) {
    if(usbcan.readMessage(&can_id, message, &message_length)) {
      fprintf(stderr, "MSG 0x%04x : (%d) ", can_id, message_length);
      for(i = 0; i < message_length; i++) 
	fprintf(stderr, "%02x ", message[i]);
      fprintf(stderr, "\n");
    }
    else
      usleep(10000);
  }
  usbcan.close();
  return 0;
}
