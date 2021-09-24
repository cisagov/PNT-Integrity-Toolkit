#include <bitset>
#include <fstream>
#include <iostream>
#include "ublox/ublox.h"
using namespace ublox;
using namespace std;

// global variables
AidIni      cur_aid_ini;
Ephemerides stored_ephems;
Almanac     stored_almanac;
AidHui      cur_aid_hui;
double      aid_ini_timestamp;
NavStatus   cur_nav_status;

std::chrono::system_clock::time_point start;

inline void printHex(char* data, int length)
{
  for (int i = 0; i < length; ++i)
  {
    printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
  }
  printf("\n");
}

void EphemerisCallback(const EphemSV&        ephemeris,
                       const SysClkTimePoint time_stamp)
{
  stored_ephems.ephemsv[ephemeris.svprn] = ephemeris;

  //    if (ephemeris.header.payload_length==104)
  //        cout << "[" << time_stamp <<  "]" <<  "Received ephemeris for sv "
  //        << ephemeris.svprn << endl;
  //    else
  //        cout << "[" << time_stamp <<  "]" <<  "No ephemeris available for sv
  //        " << ephemeris.svprn << endl;
}

void PositionTimeCallback(const AidIni&         init_position,
                          const SysClkTimePoint time_stamp)
{
  cur_aid_ini = init_position;
  std::chrono::duration<double> cur_time_sec =
    std::chrono::system_clock::now() - start;
  aid_ini_timestamp = cur_time_sec.count();
  cout << dec << "[" << aid_ini_timestamp << "]"
       << "Received aid_ini." << endl;
  cout << "Pos X: " << init_position.ecefXorLat << endl;
  cout << "Pos Y: " << init_position.ecefYorLon << endl;
  cout << "Pos Z: " << init_position.ecefZorAlt << endl;
  cout << "TOW1 = " << init_position.time_of_week << " ms" << endl;
  cout << "TOW2 = " << init_position.time_of_week_ns << " ns" << endl;
  cout << "Time accuracy1 = " << init_position.time_accuracy_ms << " ms"
       << endl;
  cout << "Time accuracy2 = " << init_position.time_accuracy_ns << " ns"
       << endl;
  cout << "Flags: 0x" << hex << (int)init_position.flags << dec << endl << endl;
}

void NavigationStatusCallback(const NavStatus&      status,
                              const SysClkTimePoint time_stamp)
{
  cur_nav_status = status;
}

void AlmanacCallback(const AlmSV& almanac, const SysClkTimePoint time_stamp)
{
  stored_almanac.almsv[almanac.svprn] = almanac;

  //    if (almanac.header.payload_length==40)
  //        cout << "[" << time_stamp <<  "]" <<  "Received almanac for sv " <<
  //        almanac.svprn << endl;
  //    else
  //        cout << "[" << time_stamp <<  "]" <<  "No almanac available for sv "
  //        << almanac.svprn << endl;
}

void HuiCallback(const AidHui& hui, const SysClkTimePoint time_stamp)
{
  cur_aid_hui = hui;
  std::chrono::duration<double> cur_time_sec =
    std::chrono::system_clock::now() - start;
  std::bitset<32> svhealth(hui.health);
  std::cout << dec << "[" << cur_time_sec.count() << "]"
            << "Received aid_hui." << std::endl;
  std::cout << "SV health flags: " << svhealth << std::endl;
  std::cout << "TOW: " << hui.tow << std::endl;
  std::cout << "Week #: " << hui.week << std::endl;
  std::cout << "Leap Seconds: " << hui.beforeleapsecs << std::endl;
}

int main(int argc, char** argv)
{
  start = std::chrono::system_clock::now();

  Ublox  my_gps;
  double ttff_unassisted;
  double ttff_assisted;
  bool   postime_onoff;
  bool   ephem_onoff;
  bool   alm_onoff;
  bool   hui_onoff;

  if (argc < 3)
  {
    std::cerr << "Usage: Full_AGPS_assist <serial port address> <baud rate>"
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

  // setup callbacks
  my_gps.set_aid_eph_Handler(EphemerisCallback);
  my_gps.set_aid_alm_Handler(AlmanacCallback);
  my_gps.set_aid_hui_Handler(HuiCallback);
  my_gps.set_aid_ini_Handler(PositionTimeCallback);
  my_gps.set_nav_status_Handler(NavigationStatusCallback);
  // my_gps.set_nav_solution_callback(NULL);
  // my_gps.set_nav_sv_info_callback(NULL);

  // turn off nmea messages
  // my_gps.SetPortConfiguration();

  // request nav status data and wait for fix
  my_gps.configureMessageRate(0x01, 0x03, 1);  // nav status at 1 Hz
  // turn off nav-sol and nav-svinfo in case netassist has turned them on
  my_gps.configureMessageRate(0x01, 0x06, 0);  // nav sol at 0 Hz
  my_gps.configureMessageRate(0x01, 0x30, 0);  // nav svinfo at 0 Hz

  // Loop 10 times to get average TTFF
  const uint8_t iterations = 10;
  double  un_ttff[iterations];
  double  as_ttff[iterations];

  // Turn AGPS data on/off
  postime_onoff = 1;
  ephem_onoff   = 1;
  alm_onoff     = 0;
  hui_onoff     = 0;

  for (uint8_t i = 0; i < iterations; i++)
  {
    ///////////////////////////////////////////////////////////////////////////
    // Perform Unassisted Cold Start                                         //
    ///////////////////////////////////////////////////////////////////////////

    // reset receiver
    cout << "Resetting receiver." << endl;
    my_gps.resetToColdStart((ublox::ResetMode)0x02);

    while (cur_nav_status.fixtype != 0x00)  // wait for receiver to reset
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

    cout << "Receiver reset. Waiting for unassisted fix" << endl;

    // clear stored assist data
    memset(&cur_aid_ini, 0, sizeof(cur_aid_ini));
    memset(&stored_ephems, 0, sizeof(stored_ephems));
    memset(&stored_almanac, 0, sizeof(stored_almanac));
    memset(&cur_aid_hui, 0, sizeof(cur_aid_hui));

    while (cur_nav_status.fixtype != 0x03)  // wait for 3D fix
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    cout << "3D fix obtained." << endl;
    cout << " TTFF: " << (cur_nav_status.ttff / 1000.) << " sec" << endl;
    cout << " Time since startup: " << (cur_nav_status.msss / 1000.) << endl
         << endl;
    ttff_unassisted = cur_nav_status.ttff / 1000.;

    // All AGPS data present, request aiding data from receiver
    if (postime_onoff == 1)
    {
      my_gps.pollAidIni();  // poll position and time
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (ephem_onoff == 1)
    {
      my_gps.pollEphem();  // poll ephemeris for all satellites
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (alm_onoff == 1)
    {
      my_gps.pollAlmanac();
    }
    if (hui_onoff == 1)
    {
      my_gps.pollHui();
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // make sure we got the aid_ini data
    if (postime_onoff == 1)
    {
      while (cur_aid_ini.header.sync1 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // make sure we got all of the ephemeris
    if (ephem_onoff == 1)
    {
      while (stored_ephems.ephemsv[32].header.sync1 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // Wait to get almanac data for all SVs (assuming #32 is sent last)
    if (alm_onoff == 1)
    {
      while (stored_almanac.almsv[32].header.sync1 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // make sure we get HUI
    if (hui_onoff == 1)
    {
      while (cur_aid_hui.header.sync1 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    ///////////////////////////////////////////////////////////////////////////
    // RESET RECEIVER AND PERFORM ASSISTED COLD START                        //
    ///////////////////////////////////////////////////////////////////////////

    // reset receiver
    cout << "Aiding data stored. Resetting receiver." << endl;
    my_gps.resetToColdStart((ublox::ResetMode)0x02);

    while (cur_nav_status.fixtype != 0x00)  // wait for receiver to reset
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cout << "Receiver reset. Waiting for assisted fix" << endl;

    ///////////////////////////////////////////////////////////////////
    // SEND AIDING DATA
    // update time in AidIni message and send
    if (postime_onoff == 1)
    {
      cur_aid_ini.flags = cur_aid_ini.flags & 0xF7;  // clear time pulse flag
      cur_aid_ini.flags = cur_aid_ini.flags & 0xFB;  // clear clock drift flag
      cur_aid_ini.flags = cur_aid_ini.flags & 0xFE;  // clear clock freq flag
      cur_aid_ini.clock_drift_or_freq = 0;

      cur_aid_ini.time_accuracy_ms = 3000;
      std::chrono::duration<double> cur_time_sec =
        std::chrono::system_clock::now() - start;
      double cur_time        = cur_time_sec.count();
      int    time_correction = (cur_time - aid_ini_timestamp) * 1000;
      std::cout << "cur_time = " << cur_time << "sec" << std::endl;
      std::cout << "aid_ini_timestamp = " << aid_ini_timestamp << "sec"
                << std::endl;
      cout << "Correct time by " << (cur_time - aid_ini_timestamp) << " sec."
           << endl;
      cout << "Correct time by " << time_correction << " ms." << endl;
      cur_aid_ini.time_of_week = cur_aid_ini.time_of_week + time_correction;
      cout << "Initialize receiver position and time." << endl;
      my_gps.sendAidIni(cur_aid_ini);
    }
    // send ephemeris
    if (ephem_onoff == 1)
    {
      cout << "Send ephemeris back to receiver." << endl;
      my_gps.sendAidEphem(stored_ephems);
    }

    // send almanac
    if (alm_onoff == 1)
    {
      cout << "Send almanac back to receiver." << endl;
      my_gps.sendAidAlm(stored_almanac);
    }
    // send HUI
    if (hui_onoff == 1)
    {
      cout << "Send HUI back to receiver." << endl;
      my_gps.sendAidHui(cur_aid_hui);
    }

    ///////////////////////////////////////////////////////////////////
    // WAIT FOR FIX
    cout << "Wait for assisted fix." << endl;

    while (cur_nav_status.fixtype != 0x03)  // wait for 3D fix
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

    cout << "3D fix obtained." << endl;
    cout << " TTFF: " << (cur_nav_status.ttff / 1000.) << " sec" << endl;
    cout << " Time since startup: " << (cur_nav_status.msss / 1000.) << endl
         << endl;
    ttff_assisted = cur_nav_status.ttff / 1000.;

    if (alm_onoff == 1)
    {
      std::cout << "Verify that Almanac was stored on receiver" << endl;
      // Poll Almanac to see if SendAlm worked
      memset(&stored_almanac, 0, sizeof(stored_almanac));
      my_gps.pollAlmanac();
      while (stored_almanac.almsv[32].header.sync1 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (hui_onoff == 1)
    {
      std::cout << "Verify that HUI was stored on receiver" << endl;
      // Poll Aid-HUI to see if SendAidHui worked
      memset(&cur_aid_hui, 0, sizeof(cur_aid_hui));
      my_gps.pollHui();
      while (cur_aid_hui.header.sync1 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    ///////////////////////////////////////////////////////////////////
    // DISPLAY RESULTS

    cout << endl << endl << "Results:" << endl;
    cout << " Unassisted TTFF (sec): " << ttff_unassisted << endl;
    cout << " Assisted TTFF (sec): " << ttff_assisted << endl;

    // Store Unassisted and Assisted TTFF Iterations
    un_ttff[i] = ttff_unassisted;
    as_ttff[i] = ttff_assisted;
  }

  // Average TTFF Iterations
  double un_total = 0;
  double as_total = 0;

  for (uint8_t j = 0; j < iterations; j++)
  {
    un_total = un_total + un_ttff[j];
    as_total = as_total + as_ttff[j];
    // std::cout << "un_total = " << un_total << endl;
    // std::cout << "as_total = " << as_total << endl;
    std::cout << "un_ttff = " << un_ttff << endl;
    std::cout << "as_ttff = " << as_ttff << endl;
  }

  double un_ave_ttff = un_total / iterations;
  double as_ave_ttff = as_total / iterations;
  // std::cout << "Results for Complete AGPS assist" << endl;
  // std::cout << "Results for Complete AGPS with degraded AGPS time accuracy"
  // << endl;
  std::cout << "Location: Stadium Parking Deck" << endl;
  std::cout << "Scenario: Under Overhang" << endl;
  std::cout << "Aiding used: AID-INI" << endl;
  // std::cout << "Results for INI and ephem AGPS with degraded AGPS time
  // accuracy" << endl;
  std::cout << "Average Unassisted TTFF = " << un_ave_ttff << endl;
  std::cout << "Average Assisted TTFF = " << as_ave_ttff << endl;

  my_gps.disconnect();

  return 0;
}
