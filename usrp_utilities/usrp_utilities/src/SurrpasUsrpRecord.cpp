//============================================================================//
//----------------- usrp_utilities/SurrpasUsrpRecord.cpp -------*- C++ -*-----//
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
#include "usrp_utilities/SurrpasUsrpRecord.hpp"
#include "if_data_utils/IFDataFileWriter.hpp"
#include "if_data_utils/IfData.hpp"

#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

using namespace usrp_utilities;
using namespace if_data_utils;
using namespace logutils;

SurrpasUsrpRecord::SurrpasUsrpRecord(const SurrpasUsrpArgs& recordArgs,
                                     const logutils::LogCallback& log)
  : recordUsrp_(recordArgs, log)
  , futureObj_(exitSignal_.get_future())
  , transferComplete_(false)
  , log_(log)
{
  size_t numChannels = recordUsrp_.getNumChannels();
  numBlocksInBufferPtr_.reset(
    new std::vector<std::atomic<size_t> >(numChannels));

  circBufferPtrs_.resize(numChannels);

  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    // initialize block counts
    numBlocksInBufferPtr_->operator[](ii) = 0;

    // initialize the buffers
    circBufferPtrs_[ii].reset(
      new uhd::transport::bounded_buffer<circbuff_element_t>(
        recordArgs.numElementsInBuff_));
  }
}

void SurrpasUsrpRecord::startRecordThread()
{
  recordThread_ = std::thread(&SurrpasUsrpRecord::bufferedTransfer, this);
}

bool SurrpasUsrpRecord::checkRecordThreadStatus()
{
    return futureObj_.wait_for(std::chrono::milliseconds(1000))
           == std::future_status::timeout;
}

void SurrpasUsrpRecord::bufferedTransfer()
{
  SurrpasUsrpArgs recordArgs = recordUsrp_.getUsrpArgs();
  switch (recordArgs.sampleType_.type_)
  {
    case IFSampleType::SC8: {
      streamToBuffer<std::complex<int8_t> >();

      break;
    }
    case IFSampleType::SC16: {
      streamToBuffer<std::complex<short> >();

      break;
    }
    case IFSampleType::FC32:
      streamToBuffer<std::complex<float> >();
      break;

    case IFSampleType::FC64:
      streamToBuffer<std::complex<double> >();
      break;

    default:
      // throw error
      break;
  }
}

void SurrpasUsrpRecord::getBufferStatus(std::vector<size_t>& bufferStatus)
{
  bufferStatus.clear();
  bufferStatus.resize(recordUsrp_.getNumChannels());
  for (size_t ii = 0; ii < recordUsrp_.getNumChannels(); ++ii)
  {
    bufferStatus[ii] = numBlocksInBufferPtr_->operator[](ii);
  }
}

double SurrpasUsrpRecord::getQcalVals(const uint16_t& channelNum)
{
  std::lock_guard<std::mutex> lock(qcalMtx_);
  return qcalVals_[channelNum];
}

void SurrpasUsrpRecord::writeMetaDataFile(const std::string& filename,
                                          const SurrpasUsrpArgs& recordArgs,
                                          const uhd::time_spec_t& startTimeIn)
{
  log_("Writing meta-data file...", logutils::LogLevel::Info);
  std::ofstream varLog(filename, std::ofstream::out);

  // First record the start time
  time_t startTime_t = startTimeIn.get_full_secs();
  struct tm* gmTimePtr;
  gmTimePtr = gmtime(&startTime_t);

  varLog << "========= Record start time (GMT): ===========" << std::endl;
  varLog << "year = " << gmTimePtr->tm_year << std::endl;
  varLog << "month = " << gmTimePtr->tm_mon << std::endl;
  varLog << "day = " << gmTimePtr->tm_mday << std::endl;
  varLog << "hour = " << gmTimePtr->tm_hour << std::endl;
  varLog << "min = " << gmTimePtr->tm_min << std::endl;
  varLog << "sec = " << gmTimePtr->tm_sec << std::endl;

  std::stringstream argStr;
  recordArgs.printArgs(argStr);

  varLog << argStr.str() << std::endl;
  std::stringstream msg;

  msg << "Meta-data file written to: " << filename;
  log_(msg.str(), LogLevel::Info);
}
