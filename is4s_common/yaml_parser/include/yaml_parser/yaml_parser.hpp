//============================================================================//
//-------------- yaml_parser.hpp --------*- C++ -*----------//
//============================================================================//
// BSD 3-Clause License
//
// Copyright (C) 2020 Integrated Solutions for Systems, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//----------------------------------------------------------------------------//
/// \file
/// \brief    Wrapper class for yaml-cpp
/// \author   Chris Collins <chris.collins@is4s.com>
/// \date     January 23, 2020
//============================================================================//
#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>

#include "logutils/logutils.hpp"

namespace yaml_parser
{
class YamlParser
{
public:
  explicit YamlParser(
    const std::string&              yamlFileName,
    const logutils::LogCallback&    log        = logutils::printLogToStdOut,
    const std::vector<std::string>& searchPath = std::vector<std::string>());

  explicit YamlParser(
    const YAML::Node&               yamlNode,
    const logutils::LogCallback&    log        = logutils::printLogToStdOut,
    const std::vector<std::string>& searchPath = std::vector<std::string>());

  ~YamlParser();

  bool exists(const std::string& variableName) const;

  template <typename DataType>
  bool readVariable(DataType& value, const std::string& variableName) const;

  template <typename DataType>
  DataType readVariable(const std::string& variableName) const;

  template <typename DataType>
  DataType readVariable(const std::string& variableName,
                        const DataType&    defaultValue) const;

private:
  static YAML::Node loadYaml(const std::string&       filename,
                      const std::vector<std::string>& searchPath,
                      const logutils::LogCallback&    log);

  logutils::LogCallback log_;

  std::vector<std::string> searchPath_; // This is never used

  YAML::Node yamlNode_;

};  // end class YamlParser

inline YamlParser::YamlParser(const std::string&              yamlFileName,
                              const logutils::LogCallback&    log,
                              const std::vector<std::string>& searchPath)
  : log_(log)
  , searchPath_(searchPath)
  , yamlNode_(loadYaml(yamlFileName, searchPath, log_))
{
}

inline YamlParser::YamlParser(const YAML::Node&               yamlNode,
                              const logutils::LogCallback&    log,
                              const std::vector<std::string>& searchPath)
  : log_(log), searchPath_(searchPath), yamlNode_(yamlNode)
{
}

inline YamlParser::~YamlParser()
{
}

inline bool YamlParser::exists(const std::string& variableName) const
{
  return yamlNode_[variableName].IsDefined();
}

template <typename DataType>
inline bool YamlParser::readVariable(DataType&          value,
                                     const std::string& variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    value = yamlNode_[variableName].as<DataType>();

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file: " << value;

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(std::vector<double>& value,
                                     const std::string&   variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    value = yamlNode_[variableName].as<std::vector<double> >();

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file:";

      for (auto& localVal : value)
      {
        logStream << " " << localVal;
      }

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(std::vector<int>&  value,
                                     const std::string& variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    value = yamlNode_[variableName].as<std::vector<int> >();

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file:";

      for (auto& localVal : value)
      {
        logStream << " " << localVal;
      }

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(std::vector<uint8_t>& value,
                                     const std::string&    variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    value = yamlNode_[variableName].as<std::vector<uint8_t> >();

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file:";

      for (auto& localVal : value)
      {
        logStream << " " << localVal;
      }

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(std::vector<std::string>& value,
                                     const std::string& variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    value = yamlNode_[variableName].as<std::vector<std::string> >();

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file:\n";

      for (auto& localVal : value)
      {
        logStream << "  " << localVal << "\n";
      }

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(Eigen::VectorXd&   value,
                                     const std::string& variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    const std::vector<double> vecValue =
      yamlNode_[variableName].as<std::vector<double> >();

    value.resize(vecValue.size());
    for (Eigen::Index idx = 0; idx < value.size(); ++idx)
    {
      value(idx) = vecValue[idx];
    }

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file: ";
      logStream << value.transpose();

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(Eigen::Vector3d&   value,
                                     const std::string& variableName) const
{
  Eigen::VectorXd vectorXdValue;
  const bool      available = readVariable(vectorXdValue, variableName);
  if (available)
  {
    if (vectorXdValue.size() != 3)
    {
      const std::string errorMsg = variableName + " must be length 3";

      if (log_)
        log_(errorMsg, logutils::LogLevel::Error);

      throw std::runtime_error(errorMsg);
    }

    value = vectorXdValue;
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(Eigen::Matrix2d&   value,
                                     const std::string& variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    const std::vector<double> vecValue =
      yamlNode_[variableName].as<std::vector<double> >();

    if (vecValue.size() != 4)
    {
      const std::string errorMsg = variableName + " must be 2 x 2";

      if (log_)
        log_(errorMsg, logutils::LogLevel::Error);

      throw std::runtime_error(errorMsg);
    }

    std::vector<double>::size_type vecIdx = 0;
    for (Eigen::Index rowIdx = 0; rowIdx < 2; ++rowIdx)
    {
      for (Eigen::Index colIdx = 0; colIdx < 2; ++colIdx)
      {
        value(rowIdx, colIdx) = vecValue[vecIdx++];
      }
    }

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file:\n";
      logStream << value;

      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <>
inline bool YamlParser::readVariable(Eigen::Matrix3d&   value,
                                     const std::string& variableName) const
{
  const bool available = exists(variableName);

  if (available)
  {
    const std::vector<double> vecValue =
      yamlNode_[variableName].as<std::vector<double> >();

    if (vecValue.size() != 9)
    {
      const std::string errorMsg = variableName + " must be 3 x 3";

      if (log_)
        log_(errorMsg, logutils::LogLevel::Error);

      throw std::runtime_error(errorMsg);
    }

    std::vector<double>::size_type vecIdx = 0;
    for (Eigen::Index rowIdx = 0; rowIdx < 3; ++rowIdx)
    {
      for (Eigen::Index colIdx = 0; colIdx < 3; ++colIdx)
      {
        value(rowIdx, colIdx) = vecValue[vecIdx++];
      }
    }

    if (log_)
    {
      std::stringstream logStream;
      logStream << "Read " << variableName << " from config file:\n";
      logStream << value;
      log_(logStream.str(), logutils::LogLevel::Debug3);
    }
  }

  return available;
}

template <typename DataType>
inline DataType YamlParser::readVariable(const std::string& variableName) const
{
  DataType value;

  if (!readVariable<DataType>(value, variableName))
  {
    const std::string errorMsg =
      variableName + " must be set in the config file";

    if (log_)
      log_(errorMsg, logutils::LogLevel::Error);

    throw std::runtime_error(errorMsg);
  }

  return value;
}

template <typename DataType>
inline DataType YamlParser::readVariable(const std::string& variableName,
                                         const DataType&    defaultValue) const
{
  DataType value = defaultValue;

  readVariable<DataType>(value, variableName);

  return value;
}

}  // end namespace yaml_parser
#endif
