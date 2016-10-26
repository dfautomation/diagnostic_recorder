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

#include "diagnostic_recorder/generic_analyzer_recorder.h"


// register as diagnostic_aggregator::Analyzer plugin
PLUGINLIB_EXPORT_CLASS(diagnostic_recorder::GenericAnalyzerRecorder, diagnostic_aggregator::Analyzer)

namespace diagnostic_recorder
{

bool GenericAnalyzerRecorder::init(const std::string base_path, const ros::NodeHandle &n)
{
  bool init_ok = GenericAnalyzer::init(base_path, n);
  if (!init_ok)
  {
    return init_ok;
  }
  return BaseRecorder::init(base_path, n);
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > GenericAnalyzerRecorder::report()
{
  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > diagnostics = diagnostic_aggregator::GenericAnalyzer::report();
  BaseRecorder::record(diagnostics);
  return diagnostics;
}

}  // namespace diagnostic_recorder
