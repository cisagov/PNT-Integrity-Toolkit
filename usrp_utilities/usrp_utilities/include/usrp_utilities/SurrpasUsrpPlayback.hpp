//============================================================================//
//----------------- usrp_utilities/SurrpasUsrpPlayback.hpp -------*- C++ -*-----//
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
///
/// \file
/// \brief    This file holds the declaration of the SurrpasUsrpPlayback class. 
/// \details 
/// \author   Josh Clanton <josh.clanton@is4s.com> 
/// \date    September 11, 2019
///
//===----------------------------------------------------------------------===//
#ifndef SURRPAS_USRP_PLAYBACK_HPP
#define SURRPAS_USRP_PLAYBACK_HPP

#include <algorithm>
#include <atomic>
#include <memory>
#include <future>
#include <cstdint>
#include <uhd/transport/bounded_buffer.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include "if_data_utils/IFDataFileReader.hpp"
#include "logutils/logutils.hpp"
#include "usrp_utilities/SurrpasUsrpDevice.hpp"

namespace usrp_utilities
{
/// \brief A class to encapsulate the IF data recording process with the
/// SURRPAS USRP
class SurrpasUsrpPlayback
{
public:
  /// \brief Constructor for the class object
  ///
  /// \param argsIn The provided device arguments
  /// \param log    A log callback for message display (defaults to std::cout)
  SurrpasUsrpPlayback(
    const SurrpasUsrpArgs& argsIn,
    const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Terminates the playback process
  void killPlayback() 
  { 
    exitSignal_.set_value();
    transferComplete_ = true; 
    playbackThread_.join();
  };

  /// \brief Checks if enough files were given
  bool checkForInvalidFiles() { return filesAreInvalid_; };

  /// \brief Starts the playback process
  void startPlaybackThread();

  /// \brief Checks the status of the playback thread
  ///
  /// Checks the status of the playback thread by attempting a timed join.
  /// If the join times out, then the thread is still running
  bool checkPlaybackThreadStatus();

  /// \brief Checks the number of blocks in each buffer
  ///
  /// This function takes in a vector and fills each element with the number of
  /// blocks in each buffer
  ///
  /// \param bufferStatus The number of blocks in each buffer
  void getBufferStatus(std::vector<size_t>& bufferStatus);

private:
  // The top level producer thread function. Calls the correct version of the
  // "bufferToStream" based on sample type.
  void bufferedTransfer();

  // The stored usrp device class
  SurrpasUsrpDevice playbackUsrp_;

  // The producer thread
  std::thread playbackThread_;

  // The consumer threads
  std::vector<std::thread> readThreads_;

  std::promise<void> exitSignal_;
  std::future<void> futureObj_;

  // Template function for pulling samples buffer(s) and writing them to the
  // usrp stream
  // Sets up streamer, kicks off fileToBuffer threads, then writes from buffer
  // to usrp stream
  template <typename samp_type>
  void bufferToStream();

  // The consumer thread function. Pulls samples from a file and writes to
  // the buffer for the given channel. 1 function/thread per channel.
  template <typename samp_type>
  void readSamplesFromFile(
    if_data_utils::IFDataFileReader<samp_type>* fileReaderPtr,
    const uint16_t& channelNum);

  // a vector of buffer pointers, 1 buffer per receive channel
  std::vector<
    std::shared_ptr<uhd::transport::bounded_buffer<circbuff_element_t> > >
    circBufferPtrs_;

  // flag to indicate playback has ceased (either by operator-kill or
  // end-condition)
  std::atomic<bool> transferComplete_;

  // true if files given are not valid (either too few or can't be opened)
  std::atomic<bool> filesAreInvalid_;

  // The number of blocks currently in each buffer
  std::shared_ptr<std::vector<std::atomic<size_t> > > numBlocksInBufferPtr_;

  // File handles for each channel's recording file
  std::vector<int> fd_;  // vector of file descriptors

  // local storage of the log callback
  logutils::LogCallback log_;
};

//--------------inline and template functions-----------
template <typename samp_type>
void SurrpasUsrpPlayback::bufferToStream()
{
  SurrpasUsrpArgs playbackArgs = playbackUsrp_.getUsrpArgs();

  std::string cpu_format         = playbackArgs.sampleType_.getTypeStr();
  std::string wire_format        = playbackArgs.wireFmt_.getTypeStr();
  std::vector<std::string> files = playbackArgs.filenames_;
  std::string start_time         = playbackArgs.startTime_;
  size_t cbcapacity              = playbackArgs.numElementsInBuff_;
  size_t num_requested_samples   = playbackArgs.numSamples_;
  double time_requested          = playbackArgs.recordTimeSec_;

  // setup streamer, kick off fileToBuffer threads for each channel, write
  // samples from buffer to stream
  unsigned long long num_total_samps = 0;

  size_t numChannels = playbackUsrp_.getNumChannels();

  // create a receive streamer
  uhd::stream_args_t stream_args(cpu_format, wire_format);

  std::vector<int> channels;
  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    channels.push_back(ii);
  }
  stream_args.channels.insert(
    stream_args.channels.end(), channels.begin(), channels.end());

  uhd::tx_streamer::sptr tx_stream =
    playbackUsrp_.getUsrp()->get_tx_stream(stream_args);
  uhd::tx_metadata_t md;

  std::vector<if_data_utils::IFDataFileReader<samp_type> > fileReaders;

  // Set and check circular buffer element and size
  const size_t samps_per_element = CB_ELEMENT_SIZE / sizeof(samp_type);

  std::stringstream sizeMsg;
  sizeMsg << cbcapacity << " elements in circular buffer";
  log_(sizeMsg.str(), logutils::LogLevel::Info);

  // create a buffer not full flag for each channel
  std::vector<bool> circBuffNotFull;
  fileReaders.resize(numChannels);
  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    circBuffNotFull.push_back(true);

    fileReaders[ii].setLogHandler(log_);
    fileReaders[ii].setReadBufferSize(CB_ELEMENT_SIZE);
    fileReaders[ii].openFile(playbackUsrp_.getUsrpArgs().filenames_[ii].c_str());
  }

  // CHECK THIS
  unsigned long long num_samps_to_get = 0;

  // TODO: Throw messages here about numSamps vs Time requested
  if (num_requested_samples == 0)
  {
    if (time_requested != 0)
    {
      num_samps_to_get =
        time_requested * playbackUsrp_.getUsrp()->get_tx_rate();
    }
  }
  else
  {
    num_samps_to_get = num_requested_samples;
  }

  // If no future start time is specified on the command line
  if (start_time.compare("0") == 0)
  {
    // Default starting time is now + 10 seconds
    md.time_spec = playbackUsrp_.getUsrp()->get_time_now()+ uhd::time_spec_t(10.0);
  }
  else
  {
    // Set up future streaming time
    std::time_t std_start_time = utcToSpecT(start_time.data());

    // check if the future time is valid
    if (std_start_time < std::time(NULL))
    {
      log_("Invalid future streaming-time setup", logutils::LogLevel::Error);
      transferComplete_ = true;
    }
    else
    {
      // time_t to time_spec_t
      md.time_spec = uhd::time_spec_t(std_start_time,0);
    }
  }

  for (unsigned int ii = 0; ii < numChannels; ++ii)
  {
    readThreads_.push_back(std::thread(
      &SurrpasUsrpPlayback::readSamplesFromFile<samp_type>,
                  this,
                  &(fileReaders[ii]),
                  ii));
  }
  // When reach future stream time, it will be set to be true
  bool start_stream = false;

  // create a vector of read elements, 1 for each channel
  std::vector<circbuff_element_t> readElements(numChannels);

  // create a vector of buffer pointers for the read operation
  std::vector<samp_type*> bufferPtrs;
  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    bufferPtrs.push_back((samp_type*)&(readElements[ii]));
  }

  // Main Loop
  while (not transferComplete_ and
         (num_total_samps < num_samps_to_get or num_samps_to_get == 0))
  {
    // pop elements from each circBuffer
    for (size_t ii = 0; ii < numChannels; ++ii)
    {
      circBufferPtrs_[ii]->pop_with_timed_wait(readElements[ii], 1.0);
      numBlocksInBufferPtr_->operator[](ii)--;
    }
    // send these elements with tx stream
    size_t num_tx_samps =
      tx_stream->send(bufferPtrs, samps_per_element, md, 3.0);

    if (start_stream == false)
    {
      start_stream        = true;
      time_t sstream_time = md.time_spec.get_full_secs();
      std::stringstream streamTimeMsg;
      streamTimeMsg << "Start streaming at USRP Time: " << ctime(&sstream_time);
      log_(streamTimeMsg.str(), logutils::LogLevel::Info);
    }

    // Make sure sent enough samps
    num_total_samps += num_tx_samps;
    if (num_tx_samps != samps_per_element)
    {
      if (num_total_samps < num_samps_to_get)
      {
        std::stringstream sampleErr;
        sampleErr << "Only sent " << num_tx_samps << "/" << samps_per_element
                  << "samples";
        log_(sampleErr.str(), logutils::LogLevel::Error);
      }
    }
  }
  transferComplete_ = true;

  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    readThreads_[ii].join();
  }
}

template <typename samp_type>
void SurrpasUsrpPlayback::readSamplesFromFile(
  if_data_utils::IFDataFileReader<samp_type>* fileReaderPtr,
  const uint16_t& channelNum)
{
  circbuff_element_t read_elem;

  if_data_utils::read_element* read_elem_ptr =
    (if_data_utils::read_element*)&(read_elem);

  bool spaceInBuffer = true;
  while (!transferComplete_)
  {
    // if there is space in the buffer, pull in a new element from the file
    // if there is not, that means the that previous iteration's attempt to
    // add data to the buffer was unsucessful, so this iteration we will
    // retry the push.
    if (spaceInBuffer)
    {
      // read a new element from the file
      if (!fileReaderPtr->readSamplesFromFile(*read_elem_ptr))
      {
        transferComplete_ =  true;
      }
    }

    // push the read element into the buffer
    spaceInBuffer = circBufferPtrs_[channelNum]->push_with_haste(read_elem);

    if (spaceInBuffer)
    {
      // push was successfule so increase the num blocks in buffer count
      numBlocksInBufferPtr_->operator[](channelNum)++;
    }
  }
}
}  // namespace usrp_utilities
#endif