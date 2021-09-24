#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "ublox/ublox.h"
using namespace ublox;
using namespace std;

// Global Variables
NavSol   cur_nav_solution;
NavClock cur_nav_clock;
double   aid_ini_timestamp;

ofstream log_file;  //!< file stream for writing logs

void LogData()
{
  boost::posix_time::ptime present_time(
    boost::posix_time::microsec_clock::universal_time());

  boost::posix_time::time_duration duration(present_time.time_of_day());

  double cpu_time = ((double)duration.total_microseconds()) / 1.0e6;

  std::cout << duration.total_microseconds() << "," << cur_nav_solution.week
            << "," << (double)cur_nav_solution.iTOW / 1000.0 << ","
            << cur_nav_solution.fTOW << "," << (int)cur_nav_solution.gpsFix
            << "," << (int)cur_nav_solution.numSV << ","
            << cur_nav_solution.ecefX << "," << cur_nav_solution.ecefY << ","
            << cur_nav_solution.ecefZ << ","
            << (double)cur_nav_clock.clkbias / 1000.0 << ","
            << cur_nav_clock.clkdrift << std::endl;

  log_file << cpu_time << "," << cur_nav_solution.week << ","
           << (double)cur_nav_solution.iTOW / 1000.0 << ","
           << cur_nav_solution.fTOW << "," << (int)cur_nav_solution.gpsFix
           << "," << (int)cur_nav_solution.numSV << ","
           << cur_nav_solution.ecefX << "," << cur_nav_solution.ecefY << ","
           << cur_nav_solution.ecefZ << ","
           << (double)cur_nav_clock.clkbias / 1000.0 << ","
           << cur_nav_clock.clkdrift << std::endl;
}

void NavSolCbk(ublox::NavSol nav_sol, SysClkTimePoint time_stamp)
{
  //  std::cout << "NAV-SOL: " << (int)nav_sol.iTOW << endl;
  //  std::cout << "   Fix Type: " << (int)nav_sol.gpsFix
  //            << " # SVs: " << (int)nav_sol.numSV << endl;
  //  std::cout << "   X: " << nav_sol.ecefX << " Y:" << nav_sol.ecefY
  //            << " Z:" << nav_sol.ecefZ << endl;
  cur_nav_solution = nav_sol;
}

void NavClockCbk(ublox::NavClock nav_clock, SysClkTimePoint time_stamp)
{
  //  std::cout << "NAV-CLK: " << (int)nav_clock.iTOW <<  std::endl;
  cur_nav_clock = nav_clock;
  LogData();
}

// Get current date/time, format is YYYY-MM-DD_HH:mm:ss
const std::string currentDateTime()
{
  char buf[80];
  try
  {
    time_t    now = time(0);
    struct tm tstruct;
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);
  }
  catch (std::exception& e)
  {
    std::cout << "Error in currentDateTime: " << e.what() << std::endl;
  }
  return buf;
}

void StartLogFile()
{
  try
  {
    string date_time = currentDateTime();

    std::string log_path = boost::filesystem::current_path().string();

    std::string log_filename = "UbloxLog_" + date_time + ".log";
    log_file.open(log_filename.c_str());
    std::cout << "Started log file: " << log_path << "/" << log_filename
              << std::endl;
  }
  catch (std::exception& e)
  {
    std::cout << "Error opening log file: " << e.what() << std::endl;

    if (log_file.is_open())
      log_file.close();
    return;
  }
}

int main(int argc, char** argv)
{
  Ublox my_gps;

  if (argc < 3)
  {
    std::cerr << "Usage: ublox_logger <serial port address> <baud rate>"
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

  my_gps.resetToWarmStart();

  // request nav solution
  my_gps.configureMessageRate(0x01, 0x06, 1);
  // request nav clock for clock bias
  my_gps.configureMessageRate(0x01, 0x22, 1);

  my_gps.set_nav_clock_Handler(NavClockCbk);
  my_gps.set_nav_solution_Handler(NavSolCbk);

  // open file for logging
  StartLogFile();

  // loop forever
  while (1)
    sleep(10);  // sleep for 10 sec

  log_file.close();

  my_gps.disconnect();

  return 0;
}
