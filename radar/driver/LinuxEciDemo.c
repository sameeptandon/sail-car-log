#include <EciDemo10A.h>
int   main( int        argc,
            char**     argv,
            char**     envp )
{
  ECI_RESULT hResult = ECI_OK;

  OS_Printf(">> Linux ECI API Demo program <<\n\n");

  //*** ECI Demo for USB-to-CAN II
  hResult = EciDemo10A();

  OS_Printf("-> Closing Linux ECI API Demo program <-\n");

  return hResult;
}
