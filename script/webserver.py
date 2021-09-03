#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Tokyo Opensource Robotics Kyokai Association
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Mace, Jihoon Lee, Isaac Isao Saito

import argparse
import roswww
import rospy


def parse_argument(argv):
    """
    argument parser for roswww server configuration
    """
    parser = argparse.ArgumentParser(description="ROSWWW Server")
    parser.add_argument('-n', '--name', default=rospy.get_name(), help='Webserver name')
    parser.add_argument('-p', '--port', default=80, type=int, help='Webserver Port number')
    parser.add_argument('-w', '--webpath', default='www', help='package relative path to web pages')
    parser.add_argument('--cached', default='true', help='static file is cached')
    parser.add_argument('--basic', default='false', help='enable basic authentication')
    parser.add_argument('--basic-yaml', default=None, help='basic key yaml file path')
    parser.add_argument('--start_port', default=8000, type=int, help='setting up port scan range')
    parser.add_argument('--end_port', default=9000, type=int, help='setting up port scan range')

    parsed_args = parser.parse_args(argv)
    cached = False if parsed_args.cached in [0, False, 'false', 'False'] else True
    basic = True if parsed_args.basic in [1, True, 'true', 'True'] else False
    basic_yaml = parsed_args.basic_yaml
    return parsed_args.name, parsed_args.webpath, (parsed_args.port, parsed_args.start_port, parsed_args.end_port), cached, basic, basic_yaml


if __name__ == '__main__':
    rospy.init_node("webserver", disable_signals=True)
    name, webpath, port, cached, basic, basic_yaml = parse_argument(rospy.myargv()[1:])
    webserver = roswww.ROSWWWServer(name, webpath, port, cached, basic, basic_yaml)
    webserver.loginfo("Initialised")
    webserver.spin()
