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

#include <diagnostic_recorder/storages/filesystem_storage.h>

#include <boost/filesystem.hpp>
#include <cstdlib>
#include <ctime>
#include <sstream>

namespace diagnostic_recorder
{

const std::string FilesystemStorage::protocol_ = "file://";

FilesystemStorage::FilesystemStorage(const std::string& url)
{
  folder_ = url.substr(protocol_.length());
  if (folder_[0] == '~') {
    const char* home = getenv("HOME");
    if (home == NULL) {
      ROS_ERROR("FilesystemStorage: Path cannot begin with ~ because the HOME environment variable is not set.");
      folder_.clear();
      return;
    }
    folder_ = std::string(home) + folder_.substr(1);
  }
  boost::filesystem::create_directories(folder_);
  if (!boost::filesystem::is_directory(folder_))
  {
    ROS_ERROR("FilesystemStorage: Failed to create the folder specified in storage URL.");
    folder_.clear();
  }
}

FilesystemStorage::~FilesystemStorage()
{
}

void FilesystemStorage::record(const std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> >& diagnostics)
{
  if (folder_.empty())
  {
    return;
  }
  ROS_INFO_ONCE("FilesystemStorage: Writing to file.");

  time_t now = time(NULL);
  char buffer[30];
  strftime(buffer, 30, "%Y%m%d.csv", localtime(&now));
  std::string filename(buffer);

  if (filename_ != filename)
  {
    ofs_.close();
  }

  if (!ofs_.is_open())
  {
    std::string path = (boost::filesystem::path(folder_) / filename).string();
    ofs_.open(path.c_str(), std::ios_base::app);
    filename_ = filename;
  }

  strftime(buffer, 30, "%Y-%m-%d %H:%M:%S", localtime(&now));
  std::string timestamp(buffer);

  std::vector<std::string> keys;
  std::vector<std::string> values;
  for (size_t i = 0, len = diagnostics.size(); i < len; ++i)
  {
    diagnostic_msgs::DiagnosticStatus& diag(*diagnostics[i]);

    for (std::vector<diagnostic_msgs::KeyValue>::iterator vt = diag.values.begin(), vt_end = diag.values.end();
         vt != vt_end; ++vt)
    {
      keys.push_back(vt->key);
      values.push_back(vt->value);
    }
  }

  std::ostringstream oss;
  oss << "\"Timestamp";
  for (std::vector<std::string>::iterator st = keys.begin(), st_end = keys.end(); st != st_end; ++st)
  {
    oss << "\",\"" << *st;
  }
  oss << '"' << std::endl;
  std::string header = oss.str();

  if (header_ != header)
  {
    if (ofs_.tellp() != 0) {
      ofs_ << std::endl << std::endl;
    }
    ofs_ << header;
    header_ = header;
  }

  ofs_ << '"' << timestamp;
  for (std::vector<std::string>::iterator st = values.begin(), st_end = values.end(); st != st_end; ++st)
  {
    ofs_ << "\",\"" << *st;
  }
  ofs_ << '"' << std::endl;
  ofs_.flush();
}

bool FilesystemStorage::matchUrl(const std::string& url)
{
  return url.substr(0, protocol_.length()) == protocol_;
}

}  // diagnostic_recorder
