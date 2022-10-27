//============================================================================//
//------------ usrp_utilities/surrpasRecordToFile.cpp -----*- C++ -*--//
//============================================================================//
//
// This file is part of the usrp_utilities library.
//
// The usrp_utilities library is free software: you can redistribute it
// and/or modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation, either version 3 of the License,
// or (at your option) any later version.
//
// The usrp_utilities library is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with The usrp_utilities library.  If not, see
// <https://www.gnu.org/licenses/>.
//
//===----------------------------------------------------------------------===//
// Modified from rx_samples_to_file
#include <uhd/types/tune_request.hpp>
#include <uhd/types/time_spec.hpp>

#ifdef WITH_UHD_VERSION_LESS_3_11
  #include <uhd/utils/thread_priority.hpp>
#else
  #include <uhd/utils/thread.hpp>
#endif

#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/transport/bounded_buffer.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <thread>
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>
#include <ctime>
#include "logutils/logutils.hpp"

//pthread lib
#include <pthread.h>
//for utc time converting
#include <time.h>
//ptime lib
#include <boost/date_time/posix_time/posix_time.hpp>

#include "usrp_utilities/SurrpasUsrpRecord.hpp"

using namespace logutils;
using namespace usrp_utilities;

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int)
{
//  usrp_utilities::writeIsDone = true;
  stop_signal_called = true;
}

int UHD_SAFE_MAIN(int argc, char *argv[])
{
  logutils::LogCallback log = printLogToStdOut;

	uhd::set_thread_priority_safe();
  
	//---------------variables to be set by po------------------
	std::string args, file, type, subdev, wirefmt, start_time;
	size_t total_num_samps, cbcapacity;
	double total_time, setup_time;
  std::vector<double> rates,freqs, gains, bws;
  std::vector<std::string> clockRefs, ants;
  double nanDefault = std::numeric_limits<double>::quiet_NaN();
	bool detachhdr;
  
  //---------------------default values-------------------------
  std::pair<std::vector<double>,std::string> defRate,defFreq,defGain,defBw;
  defRate.first.push_back(25e6);
  defRate.first.push_back(25e6);
  defRate.second = "25e6,25e6";
  
  defFreq.first.push_back(1575.42e6);
  defFreq.first.push_back(1227.6e6);
  defFreq.second = "1575.42e6, 1227.6e6";
  
  defGain.first.push_back(20.0);
  defGain.first.push_back(20.0);
  defGain.second = "20, 20";
  
  defBw.first.push_back(nanDefault);
  defBw.first.push_back(nanDefault);
  defBw.second = "nan, nan";
  
  std::pair<std::vector<std::string>, std::string> defAnt,defClockRef;
  defAnt.first.push_back("RX2");
  defAnt.first.push_back("RX2");
  defAnt.second = "RX2, RX2";
  
  defClockRef.first.push_back("internal");
  defClockRef.first.push_back("internal");
  defClockRef.second = "internal, internal";
  
  std::string defFile = "usrp_samples.dat";
  std::string defType = "short";
  size_t defTotalSamps = 0;
  double defTotalTime = 30.0;
  size_t defBuffCap = 100000;
  std::string defWireFmt = "sc16";
  double defSetupTime = 1.0;
  
	//setup the program options
  po::options_description desc("Allowed options");
	desc.add_options()
  ("help,h", "help message")
  
  ("args", po::value<std::string>(&args)->required(),
   "multi uhd device address args")
  
  ("file", po::value<std::string>(&file)->default_value(defFile),
   "name of the file to write binary samples to")
  
  ("type", po::value<std::string>(&type)->default_value(defType),
   "sample type: double (FC64), float (FC32), short (SC16), or byte (SC8)")
  
  ("nsamps", po::value<size_t>(&total_num_samps)->default_value(defTotalSamps),
   "total number of samples to receive (< 268435455)")
  
  ("time", po::value<double>(&total_time)->default_value(defTotalTime),
   "total number of seconds to receive")
  
  ("cbcapacity", po::value<size_t>(&cbcapacity)->default_value(defBuffCap),
   "Elements per Circular Buffer")
  
  ("rate", po::value<std::vector<double> >(&rates)
   ->multitoken()
   ->default_value(defRate.first, defRate.second),
   "rate of incoming samples")
  
  ("freq", po::value<std::vector<double> >(&freqs)
   ->multitoken()->default_value(defFreq.first,defFreq.second),
   "RF center frequency in Hz")
  
  ("gain", po::value<std::vector<double> >(&gains)
   ->multitoken()->default_value(defGain.first, defGain.second),
   "gain for the RF chain")
  
  ("ant", po::value<std::vector<std::string> >(&ants)
   ->multitoken()->default_value(defAnt.first,defAnt.second),
   "daughterboard antenna selection")
  
  ("subdev", po::value<std::string>(&subdev)->required(),
   "daughterboard subdevice specification")
  
  ("bw", po::value<std::vector<double> >(&bws)
   ->multitoken()->default_value(defBw.first,defBw.second),
   "daughterboard IF filter bandwidth in Hz")
  
  ("clockref", po::value<std::vector<std::string> >(&clockRefs)
   ->multitoken()->default_value(defClockRef.first,defClockRef.second),
   "reference source (internal, external, mimo)")
  
  ("wirefmt", po::value<std::string>(&wirefmt)->default_value(defWireFmt),
   "wire format (sc8 or sc16)")
  
  ("setup", po::value<double>(&setup_time)->default_value(defSetupTime),
   "seconds of setup time")

  ("qcal", "Run quantization calibration in place of recording")
  
  ("progress", "periodically display short-term bw and circbuff capacity")
  
  ("stats", "show average bandwidth on exit")
  
  ("sizemap", "track packet size and display breakdown on exit")
  
  ("null", "run without writing to file")
  
  ("continue", "don't abort on a bad packet")
  
  ("check-lo", "check LO lock status")
  
  ("int-n", "tune USRP with integer-N tuning")
  
  ("detachhdr", po::value<bool>(&detachhdr)->default_value(true),
   "enable detachhdr, true by default, set false should write = false")
  
  ("starttime", po::value<std::string>(&start_time)->default_value("0"),
   "set up start streaming time (YYYY-MM-DD H:M:S)")
		;
  
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	
	//print the help message
	if (vm.count("help"))
  {
		std::cout << boost::format("surrpasRecordToFile %s") % desc << std::endl;
		return ~0;
	}

  po::notify(vm);

	bool bw_summary = vm.count("progress") > 0;
	bool stats = vm.count("stats") > 0;
	bool null = vm.count("null") > 0;
	bool enable_size_map = vm.count("sizemap") > 0;
	bool continue_on_bad_packet = vm.count("continue") > 0;
  bool checkLo = vm.count("check-lo") > 0;
  bool intn = vm.count("int-n") > 0;
  bool qcal = vm.count("qcal") > 0;
  
	if (enable_size_map)
  {
		log("Packet size tracking enabled - will only recv one packet at a time!",
         LogLevel::Warn);
  }
	// Check number of samples
	if( total_num_samps >= 268435455 )
  {
    log("Error: --nsamps < 268435455",LogLevel::Error);
		return ~0;
	}

  std::vector<UsrpClockRef> usrpClockRefs;
  for (size_t ii = 0; ii < clockRefs.size(); ii++)
  {
    usrpClockRefs.push_back(UsrpClockRef(clockRefs[ii]));
  }
  
  usrp_utilities::SurrpasUsrpArgs recordArgs(args,
                                             UsrpDeviceMode::RECEIVE,
                                             total_time,
                                             freqs,
                                             rates,
                                             gains,
                                             ants,
                                             bws,
                                             usrpClockRefs,
                                             subdev,
                                             {file},
                                             UsrpSampleType(type),
                                             UsrpWireFormat(wirefmt),
                                             cbcapacity,
                                             start_time,
                                             bw_summary,
                                             setup_time,
                                             total_num_samps,
                                             qcal,
                                             false,
                                             checkLo,
                                             intn,
                                             stats,
                                             enable_size_map,
                                             null,
                                             continue_on_bad_packet
                                             );

  log("making recorder....", LogLevel::Info);

  usrp_utilities::SurrpasUsrpRecord recorder(recordArgs);

  log("...done",LogLevel::Info);
  if (total_num_samps == 0)
  {
    std::signal(SIGINT, &sig_int_handler);
    log("Press Ctrl + C to stop streaming...",LogLevel::Info);
  }
  std::vector<size_t> bufferStatus;
//  recorder.bufferedTransfer(); // start the record process
  recorder.startRecordThread();
  while (recorder.checkRecordThreadStatus())
  {
    //waiting on thread to finish
    // a timed join is in the check function so no need for additional wait time
    if (stop_signal_called)
    {
      recorder.killRecording();
    }
    else if (qcal)
    {
      std::stringstream qcalStr;
      for (auto ii = 0; ii < recordArgs.numChannels_; ++ii)
      {
        qcalStr << "Channel [" << ii << "] :";
        qcalStr << " MaxBufferValue = " << recorder.getQcalVals(ii) << ", ";
      }
      log(qcalStr.str(),logutils::LogLevel::Info);
    }
    else // recording, not qcal-ing
    {
      recorder.getBufferStatus(bufferStatus);
      std::stringstream bufferMsg;
      bufferMsg << "Buffer Status: ";
      for(auto ii=0; ii < bufferStatus.size(); ++ii)
      {
        bufferMsg << "Channel " << ii << ": " << bufferStatus[ii] << ", ";
      }
      log(bufferMsg.str(),logutils::LogLevel::Info);
    }
  }
  log("Done!",LogLevel::Info);;

  std::this_thread::sleep_for(std::chrono::seconds(3));

  return EXIT_SUCCESS;
}
