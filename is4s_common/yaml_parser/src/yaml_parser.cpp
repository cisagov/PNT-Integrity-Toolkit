//============================================================================//
//-------------------- yaml_parser.cpp --------*- C++ -*----------------------//
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

#include "yaml_parser/yaml_parser.hpp"

namespace yaml_parser
{
YAML::Node YamlParser::loadYaml(const std::string&              filename,
                                const std::vector<std::string>& searchPath,
                                const logutils::LogCallback&    log)
{
  // Build a list of files, first successufully loaded file will be returned
  std::vector<std::string> filesToTry;
  if (searchPath.empty())
  {
    filesToTry.push_back(filename);
  }
  else
  {
    for (auto path : searchPath)
    {
      // make sure the path ends in a trailing /
      char lastChar = path.back();
      if (lastChar != '/')
      {
        path = path + "/";
      }
      filesToTry.push_back(path + filename);
    }
  }

  YAML::Node node;
  for (auto yamlFile : filesToTry)
  {
    try
    {
      node = YAML::LoadFile(yamlFile);
      if (log)
        log("YamlParser::loadYaml(): File found", logutils::LogLevel::Debug);
      return node;
    }
    catch (YAML::BadFile& exc)
    {
      if (log)
        log("YamlParser::loadYaml(): File not found: " + yamlFile,
            logutils::LogLevel::Warn);
      continue;
    }
  }

  if (log)
    log("YamlParser::loadYaml(): Unable to find " + filename,
        logutils::LogLevel::Error);

  throw std::runtime_error("YamlParser::loadYaml(): Unable to find file");
}

}  // namespace yaml_parser