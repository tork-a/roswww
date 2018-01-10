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

import logging

import socket
import tornado.ioloop  # rosbridge installs tornado
import tornado.web
from .webrequest_handler import WebRequestHandler
from .utils import run_shellcommand, split_words, get_packages

class ROSWWWServer():

    def __init__(self, name, webpath, ports, cached):
        '''
          :param str name: webserver name
          :param str webpath: package relative path to web page source.
          :param tuple ports: ports to use in webserver. Provides default and scan range (default, start, end)
        '''
        self._name = name
        self._webpath = webpath
        self._ports = ports
        self._cached = cached
        self._logger = self._set_logger()
        self._packages = get_packages()
        self._application = self._create_webserver(self._packages)

    def _create_webserver(self, packages):
        '''
        @type packages: {str, str}
        @param packages: name and path of ROS packages.
        '''
        class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
            def set_extra_headers(self, path):
                self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

        file_handler = tornado.web.StaticFileHandler if self._cached else NoCacheStaticFileHandler

        handlers = [(r"/", WebRequestHandler, {"packages": packages})]

        for package in packages:
            handler_root = ("/" + package['name'] + "/()",
                            file_handler,
                            {"path": package['path'] + "/" + self._webpath + "/index.html"})
            handlers.append(handler_root)

            # redirect '/<package>' -> /<package>/'
            handler = ('/' + package['name'],
                       tornado.web.RedirectHandler,
                       {'url': '/' + package['name'] + '/'})
            handlers.append(handler)

            handler = ("/" + package['name'] + "/(.*)",
                       file_handler,
                       {"path": package['path'] + "/" + self._webpath,
                        "default_filename": "index.html"})
            handlers.append(handler)

        self.loginfo("# of packages : %s"%(len(packages)))
        self.loginfo("Weg Page root : %s"%(self._webpath))
        application = tornado.web.Application(handlers)
        return application

    def _bind_webserver(self):
        default, start, end = self._ports

        """ First, we try the default http port """
        bound = self._bind_to_port(self._application, default)
        if not bound:
            """ Otherwise bind any available port within the specified range """
            bound = self._bind_in_range(self._application, start, end)
        return True

    def _bind_in_range(self, application, start_port, end_port):
        if (end_port > start_port):
            for i in range(start_port, end_port):
                if self._bind_to_port(application, i):
                    return True
        return False

    def _bind_to_port(self, application, portno):
        self.loginfo("Attempting to start webserver on port %s"%portno)
        try:
            application.listen(portno)
            self.loginfo("Webserver successfully started on port %s"%portno)
        except socket.error as err:
            # Socket exceptions get handled, all other exceptions propagated
            if err.errno == 13:
                self.logwarn("Insufficient priveliges to run webserver " +
                              "on port %s. Error: %s"%(portno, err.strerror))
                self.loginfo("-- Try re-running as super-user: sudo su; " +
                              "source ~/.bashrc)")
            elif err.errno == 98:
                self.logwarn("There is already a webserver running on port %s. " +
                              "Error: %s"%(portno, err.strerror))
                self.loginfo("-- Try stopping your web server. For example, " +
                              "to stop apache: sudo /etc/init.d/apache2 stop")
            else:
                self.logerr("An error occurred attempting to listen on " +
                             "port %s: %s"%(portno, err.strerror))
            return False
        return True

    def _start_webserver(self):
        try:
            tornado.ioloop.IOLoop.instance().start()
        except KeyboardInterrupt:
            self.loginfo("Webserver shutting down")

    def spin(self):
        try:
            bound = self._bind_webserver()
            if bound:
                self._start_webserver()
            else:
                raise Exception()
        except Exception as exc:
            self.logerr("Unable to bind webserver.  Exiting.  %s" % exc)

    def _set_logger(self):
        logger = logging.getLogger('roswww')
        logger.setLevel(logging.DEBUG)

        # create console handler and set level to debug
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        # create formatter
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        # add formatter to ch
        ch.setFormatter(formatter)

        # add ch to logger
        logger.addHandler(ch)

        return logger


    def loginfo(self, msg):
        self._logger.info('%s : %s'%(self._name, msg))

    def logwarn(self, msg):
        self._logger.warning('%s : %s'%(self._name, msg))

    def logerr(self, msg):
        self._logger.error('%s : %s'%(self._name, msg))
