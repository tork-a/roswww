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
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
import rostest
import unittest

from selenium import webdriver
from selenium.common.exceptions import WebDriverException
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.support import expected_conditions as EC


class TestClient(unittest.TestCase):

    def setUp(self):
        self.url_base = rospy.get_param("url_roswww_testserver")

        try:
            opts = webdriver.firefox.options.Options()
            opts.set_headless(True)
            self.browser = webdriver.Firefox(firefox_options=opts)
        except WebDriverException:
            rospy.logwarn("Failling back to PhantomJS driver")
            self.browser = webdriver.PhantomJS()

        self.wait = webdriver.support.ui.WebDriverWait(self.browser, 10)
        # maximize screen
        self.browser.find_element_by_tag_name("html").send_keys(Keys.F11)

    def tearDown(self):
        try:
            self.browser.close()
            self.browser.quit()
        except:
            pass

    def _check_index(self, url):
        rospy.logwarn("Accessing to %s" % url)

        self.browser.get(url)
        self.wait.until(EC.presence_of_element_located((By.ID, "title")))

        title = self.browser.find_element_by_id("title")
        self.assertIsNotNone(title, "Object id=title not found")

        # check load other resouces
        self.wait.until(EC.presence_of_element_located((By.ID, "relative-link-check")))
        check = self.browser.find_element_by_id("relative-link-check")
        self.assertIsNotNone(check, "Object id=relative-link-check not found")
        self.assertEqual(check.text, "Relative link is loaded",
                         "Loading 'css/index.css' from 'index.html' failed")

    def test_index_served(self):
        url = '%s/roswww/' % (self.url_base)
        self._check_index(url)

    def test_index_redirected(self):
        url = '%s/roswww' % (self.url_base)
        self._check_index(url)


if __name__ == '__main__':
    rospy.init_node("test_client")
    exit(rostest.rosrun("roswww", "test_client", TestClient))
