//============================================================================//
//----------------- usrp_utilities/SurrpasUsrpRecord.hpp -------*- C++ -*-----//
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
/// \brief    This file contains the declaration of the SurrpasUsrpRecord class.
/// \details
/// \author   Josh Clanton <josh.clanton@is4s.com>
/// \date     April 9, 2018
///
//===----------------------------------------------------------------------===//
#ifndef SURRPAS_USRP_RECORD_HPP
#define SURRPAS_USRP_RECORD_HPP

#include <algorithm>
#include <atomic>
#include <memory>
#include <cstdint>
#include <mutex>
#include <future>
#include <thread>
#include <vector>
#include <iomanip>

#include <uhd/transport/bounded_buffer.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include "if_data_utils/IFDataFileWriter.hpp"
#include "logutils/logutils.hpp"
#include "usrp_utilities/SurrpasUsrpDevice.hpp"

namespace usrp_utilities
{
/// \brief A class to encapsulate the IF data recording process with the
/// SURRPAS USRP
class SurrpasUsrpRecord
{
public:
  /// \brief Constructor for the class object
  ///
  /// \param argsIn The provided device arguments
  /// \param log    A log callback for message display (defaults to std::cout)
  SurrpasUsrpRecord(
    const SurrpasUsrpArgs& argsIn,
    const logutils::LogCallback& log = logutils::printLogToStdOut);

  /// \brief Terminates the record process
  void killRecording() 
  { 
    exitSignal_.set_value();
    transferComplete_ = true; 
    recordThread_.join();    
  };

  /// \brief Starts the recording process
  ///
  /// The function kicks off a "producer" thread that fectches samples from the
  /// usrp and places in buffer(s). At startup, this thread also kicks off a
  /// consumer thread that reads samples from the buffer and writes them to a
  /// file. In this setup, there is a single producer and multiple consumer
  /// threads, 1 consumer thread per channel / buffer.
  void startRecordThread();

  /// \brief Checks the status of the record thread
  ///
  /// Checks the status of the record thread by attempting a timed join.
  /// If the join times out, then the thread is still running
  bool checkRecordThreadStatus();

  /// \brief Checks the number of blocks in each buffer
  ///
  /// This function takes in a vector and fills each element with the number of
  /// blocks in each buffer
  ///
  /// \param bufferStatus The number of blocks in each buffer
  void getBufferStatus(std::vector<size_t>& bufferStatus);

  /// \brief Returns the qcal value for the specified channel
  ///
  /// \param channelNum Desired channel number
  double getQcalVals(const uint16_t& channelNum);

  /// \brief Sample publish connector
  ///
  /// Connects the internal sample publishing function to an external
  /// publisher
  void setPublishSamples(
    std::function<size_t(uhd::transport::bounded_buffer<circbuff_element_t>&,
                         const SurrpasUsrpArgs&,
                         const size_t&)> handler)
  {
    publishSampleData = handler;
  };

private:

  // The top level producer thread function. Calls the correct version of the
  // "receiveToFile" based on sample type.
  void bufferedTransfer();

  // The stored usrp device class
  SurrpasUsrpDevice recordUsrp_;

  // The producer thread
  std::thread recordThread_;

  // The consumer threads
  std::vector<std::thread> writeThreads_;

  std::promise<void> exitSignal_;
  std::future<void> futureObj_;
  
  // Template function for pulling samples from the usrp stream and writing them
  // to buffer(s)
  template <typename samp_type>
  void streamToBuffer();

  // Template function for pulling samples from the usrp stream and running
  // the quantization calibration routine
  template <typename samp_type>
  void quantizationCalibration(const uint16_t& channelNum);

  template <typename samp_type>
  void publishSamples(const uint16_t& channelNum);

  template <typename samp_type>
  void writeSamplesToFile(
    if_data_utils::IFDataFileWriter<samp_type>* fileWriterPtr,
    const uint16_t& channelNum);

  // a vector of buffer pointers, 1 buffer per receive channel
  std::vector<
    std::shared_ptr<uhd::transport::bounded_buffer<circbuff_element_t>>>
    circBufferPtrs_;

  // flag to indicate recording has ceased (either by operator-kill or
  // end-condition)
  std::atomic<bool> transferComplete_;

  // The number of blocks currently in each buffer
  std::shared_ptr<std::vector<std::atomic<size_t>>> numBlocksInBufferPtr_;

  std::map<uint16_t, double> qcalVals_;
  std::mutex qcalMtx_;

  // local storage of the log callback
  logutils::LogCallback log_;

  void writeMetaDataFile(const std::string& filename,
                         const SurrpasUsrpArgs& recordArgs,
                         const uhd::time_spec_t& startTime);

  std::function<size_t(uhd::transport::bounded_buffer<circbuff_element_t>&,
                       const SurrpasUsrpArgs&,
                       const size_t&)>
    publishSampleData;

  size_t samplesPerBufferElement_;
};

//--------------inline and template functions-----------
template <typename samp_type>
void SurrpasUsrpRecord::streamToBuffer()
{
  SurrpasUsrpArgs recordArgs = recordUsrp_.getUsrpArgs();

  std::string cpu_format       = recordArgs.sampleType_.getTypeStr();
  std::string wire_format      = recordArgs.wireFmt_.getTypeStr();
  std::string file             = recordArgs.filenames_[0];
  std::string start_time       = recordArgs.startTime_;
  size_t cbcapacity            = recordArgs.numElementsInBuff_;
  size_t num_requested_samples = recordArgs.numSamples_;
  double time_requested        = recordArgs.recordTimeSec_;
  bool qcal                    = recordArgs.qcal_;
  bool publish                 = recordArgs.publish_;
  bool enable_size_map         = recordArgs.sizemap_;
  bool continue_on_bad_packet  = recordArgs.continue_;

  unsigned long long num_total_samps = 0;

  size_t numChannels = recordUsrp_.getNumChannels();

  // create a receive streamer
  uhd::stream_args_t stream_args(cpu_format, wire_format);

  std::vector<int> channels;
  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    channels.push_back(ii);
  }
  stream_args.channels.insert(
    stream_args.channels.end(), channels.begin(), channels.end());

  uhd::rx_streamer::sptr rx_stream =
    recordUsrp_.getUsrp()->get_rx_stream(stream_args);
  uhd::rx_metadata_t md;

  // extract the filename and extension so individual channel files can be
  // created
  std::string sfile = file.substr(0, file.find_last_of("."));
  std::string sext  = file.substr(sfile.size(), file.size());

  // create the binary data files, 1 per channel
  std::time_t t = std::time(nullptr);
  std::stringstream timeStamp;
  timeStamp << std::put_time(std::gmtime(&t), "%F %T %Z");

  std::string timeStampStr = timeStamp.str();
  std::replace(timeStampStr.begin(), timeStampStr.end(), ' ', '_');
  std::replace(timeStampStr.begin(), timeStampStr.end(), ':', '_');

  // create the file writers to pass to the writer threads
  // making an individual channel writer for each channel so each thread can
  // have its own writer
  std::vector<if_data_utils::IFDataFileWriter<samp_type>> fileWriters;

  const size_t samps_per_element = CB_ELEMENT_SIZE / sizeof(samp_type);
  samplesPerBufferElement_       = samps_per_element;

  std::stringstream sizeMsg;
  sizeMsg << cbcapacity << " elements in circular buffer";
  log_(sizeMsg.str(), logutils::LogLevel::Info);

  // create a buffer not full flag for each channel
  std::vector<bool> circBuffNotFull;
  std::stringstream baseFilename;
  baseFilename << sfile << "_" << timeStampStr;
  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    circBuffNotFull.push_back(true);

    // only create binary files if not running qcal
    if ((!qcal) and (!publish))
    {
      std::stringstream channelFile;
      channelFile << baseFilename.str() << "_ch" << ii << sext;

      if_data_utils::IFDataFileWriter<samp_type> newFileWriter(CB_ELEMENT_SIZE,
                                                               log_);
      if (newFileWriter.createFile(channelFile.str()))
      {
        fileWriters.push_back(newFileWriter);
      }
      else
      {
        return;
      }
    }
  }
  unsigned long long num_samps_to_get = 0;

  bool overflow_message = true;

  // Setup streaming
  // STREAM_MODE_NUM_SAMPS_AND_DONE has a limit of 10.7s @ 25MSPS
  // so num_requested_samples must be < 268435455 to use this mode
  //
  // For --time: calculate num_samps_to_get
  // For --nsamps: num_samps_to_get = num_samples_requested
  //
  // However, we also want a guaranteed execution time in case the USRP drops
  // samples and we don't get num_samps_to_get so we bound the execution time
  // based on the CPU time as a sanity check.
  // CPU time should still not be required to be correct, just tick at a
  // reasonably close rate to true time for accurate record length
  uhd::stream_cmd_t stream_cmd(
    (num_requested_samples == 0)
      ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
      : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);

  // TODO: Throw messages here about numSamps vs Time requested
  if (num_requested_samples == 0)
  {
    if (time_requested != 0)
    {
      num_samps_to_get = time_requested * recordUsrp_.getUsrp()->get_rx_rate();
    }
  }
  else
  {
    stream_cmd.num_samps = num_requested_samples;
    num_samps_to_get     = num_requested_samples;
  }
  stream_cmd.stream_now = false;

  std::time_t std_start_time = std::time(NULL);
  // If no future start time is specified on the command line
  if (start_time.compare("0") == 0)
  {
    // Default starting time is now
    // NOTE: "stream_now" cannot be set to true for MIMO streams
    stream_cmd.stream_now = false;
    stream_cmd.time_spec =
      recordUsrp_.getUsrp()->get_time_now() + uhd::time_spec_t(5.0);
  }
  else
  {
    // Set up future streaming time
    std_start_time = utcToSpecT(start_time.data());

    // check if the future time is valid
    if (std_start_time < std::time(NULL))
    {
      log_("Invalid future streaming-time setup", logutils::LogLevel::Error);
      transferComplete_ = true;
    }
    else
    {
      // time_t to time_spec_t
      uhd::time_spec_t usrp_time = uhd::time_spec_t(std_start_time, 0);
      stream_cmd.time_spec       = usrp_time;
    }
  }
  // only create metadata files if not running qcal
  if ((!qcal) and (!publish))
  {
    // Dump all of the relevant data to the log file
    std::stringstream metaFile;
    metaFile << baseFilename.str() << ".txt";
    writeMetaDataFile(metaFile.str(), recordArgs, stream_cmd.time_spec);
  }

  // Configure the USRP
  rx_stream->issue_stream_cmd(stream_cmd);

  for (unsigned int ii = 0; ii < numChannels; ++ii)
  {
    if (qcal)
    {
      if (recordArgs.sampleType_.type_ == if_data_utils::IFSampleType::FC64)
      {
          log_("Q-cal for sample type 'fc64' is untested.",
               logutils::LogLevel::Warn);
          break;
      }
      else if  (recordArgs.sampleType_.type_ == if_data_utils::IFSampleType::FC32)
      {
          log_("Q-cal for sample type 'fc32' is untested.",
               logutils::LogLevel::Warn);
          break;
      }
      writeThreads_.push_back(std::thread(
        &SurrpasUsrpRecord::quantizationCalibration<samp_type>, this, ii));
    }
    else if (publish)
    {
      writeThreads_.push_back(std::thread(
        &SurrpasUsrpRecord::publishSamples<samp_type>, this, ii));
    }
    else
    {
      writeThreads_.push_back(std::thread(
        &SurrpasUsrpRecord::writeSamplesToFile<samp_type>,
                    this,
                    &(fileWriters[ii]),
                    ii));
    }
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
    size_t num_rx_samps =
      rx_stream->recv(bufferPtrs, samps_per_element, md, 3.0, enable_size_map);

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
    {
      log_("uhd::rx_metadata_t::ERROR_CODE_TIMEOUT", logutils::LogLevel::Error);
      continue;
    }
    else
    {
      // push each read element onto it's repsective buffer
      for (size_t ii = 0; ii < numChannels; ++ii)
      {
        circBuffNotFull[ii] =
          circBufferPtrs_[ii]->push_with_haste(readElements[ii]);

        numBlocksInBufferPtr_->operator[](ii)++;
      }

      if (start_stream == false)
      {
        start_stream        = true;
        time_t sstream_time = md.time_spec.get_full_secs();
        std::stringstream streamTimeMsg;
        streamTimeMsg << "Start streaming at USRP Time: "
                      << ctime(&sstream_time);
        log_(streamTimeMsg.str(), logutils::LogLevel::Info);
      }
    }

    // check to see if all of the buffers have space
    bool allHaveSpace = std::all_of(circBuffNotFull.begin(),
                                    circBuffNotFull.end(),
                                    [](bool ii) { return ii; });

    // if any of them do not have space, end recording
    if (!allHaveSpace and !qcal)
    {
      log_("At least one of the circular buffers is FULL! Stopping Record",
           logutils::LogLevel::Error);
      transferComplete_ = true;
    }

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
    {
      if (overflow_message)
      {
        double writeSpeed =
          recordUsrp_.getUsrp()->get_rx_rate() * sizeof(samp_type) / 1e6;

        overflow_message = false;
        std::stringstream overflowErr;

        overflowErr << "Got an overflow indication. "
                    << "Please consider the following:" << std::endl
                    << "  Your write medium must sustain a rate of "
                    << writeSpeed << "MB/s." << std::endl
                    << "  Dropped samples will not be written to the file."
                    << std::endl
                    << "  This message will not appear again.";

        log_(overflowErr.str(), logutils::LogLevel::Error);
      }
      continue;
    }
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
    {
      std::stringstream errorStr;
      errorStr << "Receiver error: " << md.strerror();
      if (continue_on_bad_packet)
      {
        log_(errorStr.str(), logutils::LogLevel::Error);
        continue;
      }
      else
        throw std::runtime_error(errorStr.str());
    }

    num_total_samps += num_rx_samps;
    if (num_rx_samps != samps_per_element)
    {
      if (num_total_samps < num_samps_to_get)
      {
        std::stringstream sampleErr;
        sampleErr << "Only got " << num_rx_samps << "/" << samps_per_element
                  << "samples";
        log_(sampleErr.str(), logutils::LogLevel::Error);
      }
    }
  }
  transferComplete_ = true;

  // Wait for threads to exit
  for (size_t ii = 0; ii < numChannels; ++ii)
  {
    // writeThreadPtrs_[ii]->join();
    writeThreads_[ii].join();
    // close(fd_[ii]);
  }
}

template <typename samp_type>
void SurrpasUsrpRecord::quantizationCalibration(const uint16_t& channelNum)
{
  SurrpasUsrpArgs recordArgs = recordUsrp_.getUsrpArgs();

  std::chrono::milliseconds qcalPeriod(100);
  auto lastQcal = std::chrono::system_clock::now();

  circbuff_element_t cal_ele;
  while (!transferComplete_)
  {
    circBufferPtrs_[channelNum]->pop_with_timed_wait(cal_ele, 1.0);
    numBlocksInBufferPtr_->operator[](channelNum)--;

    // run a qcal at every display period
    auto now                           = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now - lastQcal;
    if (diff > qcalPeriod)
    {
      lastQcal = std::chrono::system_clock::now();

      samp_type sample;
      size_t buffSizeBytes    = sizeof(cal_ele.a);
      size_t sampSizeBytes    = sizeof(sample);
      size_t numSampsInBuffer = buffSizeBytes / sampSizeBytes;

      std::vector<double> normVec(numSampsInBuffer);

      size_t buffIdx = 0;
      for (size_t ii = 0; ii < numSampsInBuffer; ++ii)
      {
        samp_type* tmpValPtr = (samp_type*)&(cal_ele.a[buffIdx]);
        buffIdx              = buffIdx + sampSizeBytes;

        normVec[ii] = pow((double)tmpValPtr->real(), 2.0) +
                      pow((double)tmpValPtr->imag(), 2.0);
      }
      double maxValue = *std::max_element(normVec.begin(), normVec.end());
      std::lock_guard<std::mutex> qcalLock(qcalMtx_);
      qcalVals_[channelNum] = sqrt(maxValue);
    }
  }
}

template <typename samp_type>
void SurrpasUsrpRecord::writeSamplesToFile(
  if_data_utils::IFDataFileWriter<samp_type>* fileWriterPtr,
  const uint16_t& channelNum)
{
  circbuff_element_t write_elem;
  if_data_utils::write_element* write_elem_ptr =
    (if_data_utils::write_element*)&(write_elem);

  while (!transferComplete_)
  {
    // Block until we have some data to write
    // get the pointer to the file writers storage element and convert to the
    // necessary type for the pop function
    circBufferPtrs_[channelNum]->pop_with_timed_wait(write_elem, 1.0);
    numBlocksInBufferPtr_->operator[](channelNum)--;

    // write_elem_ptr = (if_data_utils::write_element*)&(write_elem);
    fileWriterPtr->writeSamplesToFile(*write_elem_ptr);
  }
}

template <typename samp_type>
void SurrpasUsrpRecord::publishSamples(const uint16_t& channelNum)
{
  SurrpasUsrpArgs recordArgs = recordUsrp_.getUsrpArgs();

  std::chrono::milliseconds publishPeriod(1000);
  auto lastPublish = std::chrono::system_clock::now();

  circbuff_element_t publish_element;
  while (!transferComplete_)
  {
    // run a qcal at every display period
    auto now                           = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now - lastPublish;
    if (diff > publishPeriod)
    {
      // pass the buffer to the publisher to the data can be popped directly
      // the publishing structure
      size_t numRead = publishSampleData(*(circBufferPtrs_[channelNum].get()),
                                         recordArgs,
                                         samplesPerBufferElement_);

      numBlocksInBufferPtr_->operator[](channelNum) =
        numBlocksInBufferPtr_->operator[](channelNum) - numRead;

      lastPublish = std::chrono::system_clock::now();
    }
    else
    {
      // continue to pop data out of the buffer to keep it emptying
      circBufferPtrs_[channelNum]->pop_with_timed_wait(publish_element, 1.0);
      numBlocksInBufferPtr_->operator[](channelNum)--;
    }
  }
}

}  // namespace usrp_utilities
#endif
