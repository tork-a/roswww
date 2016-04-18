Welcome to roswww's documentation!
==================================

Contents:

.. contents:: Table of Contents
   :depth: 3

Convenient tool to develop the web apps under ROS infrastructure

Installation
==================

On Ubuntu,:

  $ apt-get install ros-indigo-roswww

On other platform, download sourcecode into your Catkin workspace.

Usage
======

Simply run web server
------------------------

To test if `roswww` can run on your machine and provide http server feature, run a web server from this package by simply running a launch file:

  $ roslaunch roswww roswww.launch
  
  Checking log directory for disk usage. This may take awhile.
  Press Ctrl-C to interrupt
  Done checking log file disk usage. Usage is <1GB.
  
  started roslaunch server http://tork-kudu1:48011/
  
  SUMMARY
    
  PARAMETERS
   * /rosdistro: indigo
   * /rosversion: 1.11.16
  
  NODES
    /
      roswww (roswww/webserver.py)
  
  auto-starting new master
  process[master]: started with pid [26851]
  ROS_MASTER_URI=http://localhost:11311
  
  setting /run_id to 8482c492-96b5-11e5-bc2d-f816542d218e
  process[rosout-1]: started with pid [26864]
  started core service [/rosout]
  process[roswww-2]: started with pid [26867]
  2015-11-29 08:23:27,191 - roswww - INFO - roswww : # of packages : 689
  2015-11-29 08:23:27,191 - roswww - INFO - roswww : Weg Page root : www
  2015-11-29 08:23:27,285 - roswww - INFO - roswww : Initialised
  2015-11-29 08:23:27,285 - roswww - INFO - roswww : Attempting to start webserver on port 8085
  2015-11-29 08:23:27,289 - roswww - INFO - roswww : Webserver successfully started on port 8085
  WARNING:tornado.access:404 GET /favicon.ico (127.0.0.1) 1.60ms
  WARNING:tornado.access:404 GET /favicon.ico (127.0.0.1) 1.33ms

You can see a web page being published at http://localhost:%PORT_OF_YOURCHOICE%/ like below, which should show the list of ROS packages in your `ROS_PACKAGE_PATH`:

 ::

  ROS web server successfully started.
  
  Package List
  
  abb_driver
  abb_irb2400_moveit_plugins
  abb_irb2400_support
  

Integrate into your ROS package
---------------------------------------------

You can integrate web server capability from `roswww` by either launch file or python module. 

Integrate by launch
++++++++++++++++++++++++++++++++++++

Examples are also available in packages in `visualization_rwt <https://github.com/tork-a/visualization_rwt>`_.

1. In your own launch, include `roswww.launch` file. Customize arguments if necessary.

 ::

  <arg name="name" default="www server for ros"/>
  <arg name="port" default="8085"/> <!-- avoid to use apache default port -->
  <arg name="webpath" default="www"/> <!-- relative path to the webroot. E.g. place this foloder in the ROS package root dir -->
  <arg name="use_roswww" default="true" />
  <include if="$(arg use_roswww)" file="$(find roswww)/launch/roswww.launch">
    <arg name="name" value="$(arg name)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="webpath" value="$(arg webpath)"/>
  </include>

2. Add `<run_depend>roswww</run_depend>` in your `package.xml`, to avoid "404 package not found" kind of error when you run.

Integrate by Python module
++++++++++++++++++++++++++++++++++++

Use `roswww.roswww_server.ROSWWWServer <http://docs.ros.org/indigo/api/roswww/html/roswww__server_8py.html>`_ class to include the ROS web server capability into your Python package.

Sample
========

Static page
--------------

When you launch the roswww, you can access static pages(html) which are installed in share/%PACKAGE_NAME%/www folder through http://localhost:%PORT_OF_YOURCHOICE%/%PACKAGE_NAME%/%STATIC_PAGE%.html. 

Simple talker and listener
--------------

To play with the rostopic, you can launch a simple talker and listener:

  $ roslaunch roswww start_bridge.launch

You can send a message through ROS topic from http://localhost:%PORT_OF_YOURCHOICE%/roswww/talker.html. And also, you can subscribe the message on http://localhost:%PORT_OF_YOURCHOICE%/roswww/listener.html.

Chat
--------------

After launching start_bridge.launch, let's open http://localhost:%PORT_OF_YOURCHOICE%/roswww/chat.html with a browser in two windows. Once you send a message from one of the windows, the message will be shown in both windows.

Support, communication
==========================

 * `ROS Answers <http://answers.ros.org/>`_ for questions.
 * `Issue tracker <https://github.com/tork-a/roswww/issues>`_ for issues.
 * You could also ask ROS-web related discussions on `robot-web-tools <https://groups.google.com/forum/#!forum/robot-web-tools>`_ ML. Note that roswww is an individual tool from `robot-web-tools`.


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
