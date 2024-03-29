/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, DF Automation & Robotics Sdn Bhd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the DF Automation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Patrick Chin
 *********************************************************************/

#ifndef DIAGNOSTIC_RECORDER_STORAGES_FILESYSTEM_STORAGE_H
#define DIAGNOSTIC_RECORDER_STORAGES_FILESYSTEM_STORAGE_H

#include <boost/algorithm/string.hpp>
#include <fstream>

#include <diagnostic_recorder/storage.h>

namespace diagnostic_recorder
{

class FilesystemStorage
{
public:
  explicit FilesystemStorage(const std::string& url);
  virtual ~FilesystemStorage();

  virtual void record(const std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> >& diagnostics);

  static bool matchUrl(const std::string& url);

private:
  std::string csvEscape(std::string str)
  {
    boost::replace_all(str, "\"", "\"\"");
    return str;
  }

private:
  std::string folder_;
  std::string filename_;
  std::ofstream ofs_;
  std::string header_;

  static const std::string protocol_;
};

}  // namespace diagnostic_recorder

#endif  // DIAGNOSTIC_RECORDER_STORAGES_FILESYSTEM_STORAGE_H
