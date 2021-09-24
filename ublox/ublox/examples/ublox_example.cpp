#include <iostream>
#include "ublox/ublox.h"
using namespace ublox;
using namespace std;

// Global Variables
Almanac   stored_almanac;
NavStatus cur_nav_status;
AidIni    cur_aid_ini;
double    aid_ini_timestamp;

int main(int argc, char** argv)
{
  Ublox my_gps;
  // double ttff_unassisted;
  // double ttff_assisted;

  std::cout << "sizeof navpvt: " << sizeof(ublox::NavPvt) << std::endl;
  std::cout << "sizeof NavSigBlock: " << sizeof(ublox::NavSigBlock)
            << std::endl;
  std::cout << "sizeof NavSig: " << sizeof(ublox::NavSig) << std::endl;
  std::cout << "sizeof NavSigFlags: " << sizeof(ublox::NavSigFlags)
            << std::endl;

  std::cout << "sizeof MonRfBlock: " << sizeof(ublox::MonRfBlock) << std::endl;
  std::cout << "sizeof MonRfBlock: " << sizeof(ublox::MonRf) << std::endl;

  ublox::NavSigFlags flags;

  memset(&flags, 0, 2);

  flags.prUsed = 1;

  uint16_t flagsInt;
  memcpy(&flagsInt, &flags, 2);

  std::cout << "Flags: " << flagsInt << std::endl;

  if (argc < 3)
  {
    std::cerr << "Usage: ublox_example <serial port address> <baud rate>"
              << std::endl;
    return 0;
  }
  std::string port(argv[1]);
  int         baudrate = 115200;
  istringstream(argv[2]) >> baudrate;

  // Connect to Receiver
  bool result = my_gps.connect(port, baudrate);
  if (result)
  {
    cout << "Successfully connected." << endl;
  }
  else
  {
    cout << "Failed to connect." << endl;
    return -1;
  }

  // request position message
  // my_gps.configureMessageRate(0x01,0x03,1);
  my_gps.configureMessageRate(0x01, 0x07, 1);
  // my_gps.configureMessageRate(0x01,0x35,1);
  my_gps.configureMessageRate(0x0A, 0x38, 1);

  // loop forever
  while (1)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  my_gps.disconnect();

  return 0;
}
