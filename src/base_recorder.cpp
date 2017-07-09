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

#include <diagnostic_recorder/base_recorder.h>
#include <diagnostic_recorder/storages/filesystem_storage.h>
#include <diagnostic_recorder/storages/mongodb_storage.h>

namespace diagnostic_recorder
{

bool BaseRecorder::init(const std::string base_path, const ros::NodeHandle &n)
{
  std::vector<std::string> storage_urls;
  if (!n.getParam("storage_urls", storage_urls))
  {
    std::string storage_url;
    if (n.getParam("storage_url", storage_url))
    {
      storage_urls.push_back(storage_url);
    }
  }

  storages_.clear();
  for (int i = 0; i < storage_urls.size(); i++)
  {
    const std::string& url = storage_urls[i];

    if (FilesystemStorage::matchUrl(url))
    {
      storages_.push_back(boost::reinterpret_pointer_cast<Storage>(boost::make_shared<FilesystemStorage>(url)));
    }
    else if (MongodbStorage::matchUrl(url))
    {
      storages_.push_back(boost::reinterpret_pointer_cast<Storage>(boost::make_shared<MongodbStorage>(url)));
    }
    else
    {
      ROS_ERROR("Invalid storage URL: %s", url.c_str());
    }
  }

  if (storages_.empty())
  {
    ROS_ERROR("Storage URL does not match any available storage backend. Recording is not available.");
  }
  return true;
}

void BaseRecorder::record(const std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> >& diagnostics)
{
  for (int i = 0; i < storages_.size(); i++)
  {
    storages_[i]->record(diagnostics);
  }
}

}  // namespace diagnostic_recorder
