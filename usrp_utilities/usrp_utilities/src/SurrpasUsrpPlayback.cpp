//============================================================================//
//----------------- usrp_utilities/SurrpasUsrpPlayback.cpp -------*- C++ -*-----//
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
//
//
//  This file contains the declaration of the SurrpasUsrpPlayback class.
//
//  Josh Clanton <josh.clanton@is4s.com>
//  September 11, 2019
//
//===----------------------------------------------------------------------===//
#include "usrp_utilities/SurrpasUsrpPlayback.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

using namespace usrp_utilities;
using namespace if_data_utils;
using namespace logutils;

SurrpasUsrpPlayback::SurrpasUsrpPlayback(const SurrpasUsrpArgs& playbackArgs,
                                         const logutils::LogCallback& log)
  : playbackUsrp_(playbackArgs, log)
  , futureObj_(exitSignal_.get_future())
  , transferComplete_(false)
  , filesAreInvalid_(false)
  , log_(log)
{
  size_t numChannels = playbackUsrp_.getNumChannels();
  size_t numFiles    = playbackArgs.filenames_.size();

  // if user gave too many files, only playback first n files
  // where n is numChannels
  if (numFiles > numChannels)
  {
    std::stringstream fileErr;
    fileErr << "Too many files given! Only using: ";
    for (size_t jj = 0; jj < numChannels; jj++)
    {
      fileErr << playbackArgs.filenames_[jj] << std::endl;
    }
    log_(fileErr.str(), logutils::LogLevel::Warn);
  }
  // if user gave too few files, exit
  else if (numFiles < numChannels)
  {
    log_("Not enough files given!", logutils::LogLevel::Error);
    filesAreInvalid_ = true;
  }

  numBlocksInBufferPtr_.reset(
    new std::vector<std::atomic<size_t> >(numChannels));

  circBufferPtrs_.resize(numChannels);
  // readThreads_.resize(numChannels);

  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    // initialize block counts
    numBlocksInBufferPtr_->operator[](ii) = 0;

    // initialize the buffers
    circBufferPtrs_[ii].reset(
      new uhd::transport::bounded_buffer<circbuff_element_t>(
        playbackArgs.numElementsInBuff_));
  }
}

void SurrpasUsrpPlayback::startPlaybackThread()
{
  playbackThread_ = std::thread(&SurrpasUsrpPlayback::bufferedTransfer, this);
}

bool SurrpasUsrpPlayback::checkPlaybackThreadStatus()
{
 // true if still running
  return futureObj_.wait_for(std::chrono::milliseconds(1000))
           == std::future_status::timeout;
}

void SurrpasUsrpPlayback::bufferedTransfer()
{
  SurrpasUsrpArgs playbackArgs = playbackUsrp_.getUsrpArgs();
  switch (playbackArgs.sampleType_.type_)
  {
    case IFSampleType::SC8: {
      bufferToStream<std::complex<int8_t> >();

      break;
    }
    case IFSampleType::SC16: {
      bufferToStream<std::complex<short> >();

      break;
    }
    case IFSampleType::FC32:
      bufferToStream<std::complex<float> >();
      break;

    case IFSampleType::FC64:
      bufferToStream<std::complex<double> >();
      break;

    default:
      // throw error
      break;
  }
}

void SurrpasUsrpPlayback::getBufferStatus(std::vector<size_t>& bufferStatus)
{
  bufferStatus.clear();
  bufferStatus.resize(playbackUsrp_.getNumChannels());
  for (size_t ii = 0; ii < playbackUsrp_.getNumChannels(); ++ii)
  {
    bufferStatus[ii] = numBlocksInBufferPtr_->operator[](ii);
  }
}