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
                                const std::vector<std::string>& searchPath)
{
  if (searchPath.size() > 0)
  {
    YAML::Node node;
    for (auto path : searchPath)
    {
      // make sure the path ends in a trailing /
      char lastChar = path.back();
      if (lastChar != '/')
      {
        path = path + "/";
      }
      try
      {
        std::cout << "Load file: " << path + filename << std::endl;
        node = YAML::LoadFile(path + filename);
        std::cout << "Found" << std::endl;
        return node;
      }
      catch (YAML::BadFile& exc)
      {
        std::cout << "Not found" << std::endl;
      }
    }

    throw YAML::BadFile();
  }
  else
  {
    try
    {
      std::cout << "Load file: " << filename << std::endl;

      const YAML::Node node = YAML::LoadFile(filename);

      std::cout << "Found" << std::endl;

      return node;
    }
    catch (YAML::BadFile& exc)
    {
      std::cout << "Not found" << std::endl;
    }

    throw YAML::BadFile();
  }
}

}  // namespace yaml_parser