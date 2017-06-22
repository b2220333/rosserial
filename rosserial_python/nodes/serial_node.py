#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rclpy
from rosserial_python import SerialClient, RosSerialServer
from rosserial_python.rosserial_python.SerialClient import loginfo, dbginfo
from serial import SerialException
from time import sleep
import multiprocessing
import argparse
import sys


def main():
    """
    parser = argparse.ArgumentParser(description='ROS 2 Serial Python Node')
    parser.add_argument('~port', default='/dev/ttyUSB0', help='where the device is')
    parser.add_argument('~baud', default=57600, help='baud rate to communicate with the device')
    parser.add_argument('/rosserial_embeddedlinux/tcp_port', default=11411, 'port on which to talk to the server')
    parser.add_argument('/rosserial_embeddedlinux/fork_server', default=False, 'fork the server')
    parser.parse_args()
    """
    rclpy.init()
    rclpy.create_node("serial_node")
    loginfo("ROS 2 Serial Python Node")

    # port_name = rclpy.get_param('~port','/dev/ttyUSB0')
    port_name = '/dev/ttyUSB0'
    # baud = int(rclpy.get_param('~baud','57600'))
    baud = 57600

    # TODO: should these really be global?
    # tcp_portnum = int(rospy.get_param('/rosserial_embeddedlinux/tcp_port', '11411'))
    tcp_portnum = 11411
    # fork_server = rospy.get_param('/rosserial_embeddedlinux/fork_server', False)
    fork_server = False

    # TODO: do we really want command line params in addition to parameter server params?
    # sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2 :
        port_name  = sys.argv[1]
    if len(sys.argv) == 3 :
        baud = int(sys.argv[2])

    if port_name == "tcp" :
        server = RosSerialServer(tcp_portnum, fork_server)
        rospy.loginfo("Waiting for socket connections on port %d" % tcp_portnum)
        try:
            server.listen()
        except KeyboardInterrupt:
            loginfo("got keyboard interrupt")
        finally:
            loginfo("Shutting down")
            for process in multiprocessing.active_children():
                loginfo("Shutting down process %r", process)
                process.terminate()
                process.join()
            loginfo("All done")

    else :          # Use serial port
        while rclpy.ok():
            loginfo("Connecting to %s at %d baud" % (port_name,baud) )
            try:
                client = SerialClient(port_name, baud)
                client.run()
            except KeyboardInterrupt:
                break
            except SerialException:
                sleep(1.0)
                continue
            except OSError:
                sleep(1.0)
                continue
