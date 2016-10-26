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

#include <sstream>

namespace diagnostic_recorder
{

bool BaseRecorder::init(const std::string base_path, const ros::NodeHandle &n)
{
  std::string storage_url;
  n.param("storage_url", storage_url, std::string(""));

  std::string protocol = "mongodb://";
  if (storage_url.substr(0, protocol.length()) != protocol)
  {
    ROS_ERROR("Storage URL does not match any available storage backend. Recording is not available.");
    return true;
  }

  storage_url = storage_url.substr(protocol.length());
  size_t db_name_start = storage_url.find('/');
  if (db_name_start == std::string::npos)
  {
    ROS_ERROR("Database name must be specified in the storage url after the host name, separated by a slash.");
    return true;
  }

  db_name_ = storage_url.substr(db_name_start + 1);
  storage_url = storage_url.substr(0, db_name_start);

  expire_after_ = 24 * 3600;
  size_t collection_name_start = db_name_.find('#');
  if (collection_name_start != std::string::npos)
  {
    collection_name_ = db_name_.substr(collection_name_start + 1);
    db_name_ = db_name_.substr(0, collection_name_start);

    size_t expire_after_start = collection_name_.find(',');
    if (expire_after_start != std::string::npos)
    {
      std::istringstream expire_after_ss(collection_name_.substr(expire_after_start + 1));
      expire_after_ss >> expire_after_;
      collection_name_ = collection_name_.substr(0, expire_after_start);
    }
  }
  else
  {
    collection_name_ = "diagnostic";
  }

  size_t db_host_start = storage_url.find('@');
  if (db_host_start != std::string::npos)
  {
    db_host_ = storage_url.substr(db_host_start + 1);
    storage_url = storage_url.substr(0, db_host_start);

    size_t db_pw_start = storage_url.find(':');
    if (db_pw_start != std::string::npos)
    {
      db_user_ = storage_url.substr(0, db_pw_start);
      db_password_ = storage_url.substr(db_pw_start + 1);
    }
    else
    {
      db_user_ = storage_url;
      db_password_ = "";
    }
  }
  else
  {
    db_host_ = storage_url;
    db_user_ = "";
    db_password_ = "";
  }

  try
  {
    std::string errmsg;
    conn_.reset(new mongo::DBClientConnection(true));
    if (!db_user_.empty())
    {
      conn_->auth(db_name_, db_user_, db_password_, errmsg);
    }
    conn_->connect(db_host_, errmsg);
    if (conn_->isFailed())
    {
      ROS_ERROR("Faild to connect to Mongo storage backend. Recording is not available.");
      conn_.reset();
      return true;
    }
  }
  catch (std::exception &ex)
  {
    ROS_ERROR("Storage exception: %s. Recording is not available.", ex.what());
    conn_.reset();
    return true;
  }

  conn_->ensureIndex(db_name_ + "." + collection_name_, BSON("timestamp" << 1),
                     true, "",  true, false, -1, expire_after_);
  return true;
}

void BaseRecorder::record(const std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> >& diagnostics) {
  if (!conn_)
  {
    return;
  }
  ROS_INFO_ONCE("Persisting to DB.");

  mongo::BSONObjBuilder bb;
  bb.appendTimeT("timestamp", ros::Time::now().sec);

  mongo::BSONArrayBuilder status(bb.subarrayStart("status"));
  for (size_t i = 0, len = diagnostics.size(); i < len; ++i)
  {
    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> diag(diagnostics[i]);

    mongo::BSONObjBuilder bb0;
    bb0.append("level", diag->level);
    bb0.append("name", diag->name);
    bb0.append("message", diag->message);
    bb0.append("hardware_id", diag->hardware_id);

    mongo::BSONArrayBuilder values(bb0.subarrayStart("values"));
    for (std::vector<diagnostic_msgs::KeyValue>::iterator vt = diag->values.begin(), vt_end = diag->values.end();
         vt != vt_end; ++vt)
    {
      mongo::BSONObjBuilder bb1;
      bb1.append("key", vt->key);
      bb1.append("value", vt->value);
      values.append(bb1.obj());
    }
    values.done();
    status.append(bb0.obj());
  }
  status.done();
  conn_->insert(db_name_ + "." + collection_name_, bb.obj());
}

}  // namespace diagnostic_recorder
