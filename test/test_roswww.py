#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tokyo Opensource Robotics Kyokai Association
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
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
# Author: Isaac I.Y. Saito

import requests
from requests import ConnectionError
import unittest

import rospy
import rostest

from roswww.roswww_server import ROSWWWServer

_PKG = 'roswww'


class TestRoswww(unittest.TestCase):
    '''    '''

    @classmethod
    def setUpClass(cls):
        ''' Assume roswww server is not running'''
        True

    @classmethod
    def tearDownClass(cls):
        True

    def _is_wwwserver_running(self):
        rest = None
        url = rospy.get_param('url_roswww_testserver')
        rospy.loginfo('URL used for the server: {}'.format(url))
        print('URL used for the server: {}'.format(url))
        try:
            rest = requests.get(url)
        except ConnectionError:
            rospy.logerr('http request failed.')

        self.assertIsNotNone(rest)

    def test_roswww_by_launch(self):
        ''' Test if roswww server is running, ensured by downloading index.htm'''
        self._is_wwwserver_running()

    def test_roswww_by_pythonmod(self):
        ''' Test if roswww server is running, which is run via Python module'''
        ROSWWWServer('test_roswwww_py', 'www', '8086', True)
        self._is_wwwserver_running()

if __name__ == '__main__':
    rostest.rosrun(_PKG, 'test_roswww', TestRoswww)
