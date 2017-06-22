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
import imp

from builtin_interfaces.msg import Time
from rosserial_msgs.msg import Log, TopicInfo
from rosserial_msgs.srv import RequestMessageInfo, RequestParam, RequestServiceInfo
from rosserial_python.rosserial_python.SerialClient import load_pkg_module, load_message, load_service, loginfo


class RosSerialServer:
    """
        RosSerialServer waits for a socket connection then passes itself, forked as a
        new process, to SerialClient which uses it as a serial port. It continues to listen
        for additional connections. Each forked process is a new ros node, and proxies ros
        operations (e.g. publish/subscribe) from its connection to the rest of ros.
    """
    def __init__(self, tcp_portnum, fork_server=False):
        print("Fork_server is: %s" % fork_server)
        self.tcp_portnum = tcp_portnum
        self.fork_server = fork_server


    def listen(self):
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #bind the socket to a public host, and a well-known port
        self.serversocket.bind(("", self.tcp_portnum)) #become a server socket
        self.serversocket.listen(1)

        while True:
            # accept connections
            print("waiting for socket connection")
            (clientsocket, address) = self.serversocket.accept()

            # now do something with the clientsocket
            rospy.loginfo("Established a socket connection from %s on port %s" % (address))
            self.socket = clientsocket
            self.isConnected = True

            if (self.fork_server == True):  # if configured to launch server in a separate process
                loginfo("Forking a socket server process")
                process = multiprocessing.Process(target=self.startSocketServer, args=(address))
                process.daemon = True
                process.start()
                loginfo("launched startSocketServer")
            else:
                loginfo("calling startSerialClient")
                self.startSerialClient()
                loginfo("startSerialClient() exited")


    def startSerialClient(self):
        client = SerialClient(self)
        try:
            client.run()
        except KeyboardInterrupt:
            pass
        except RuntimeError:
            loginfo("RuntimeError exception caught")
            self.isConnected = False
        except socket.error:
            loginfo("socket.error exception caught")
            self.isConnected = False
        finally:
            self.socket.close()
            for sub in client.subscribers.values():
                sub.unregister()
            for srv in client.services.values():
                srv.unregister()


    def startSocketServer(self, port, address):
        loginfo("starting ROS Serial Python Node serial_node-%r" % (address,))
        rclpy.create_node("serial_node_%r" % (address,))
        self.startSerialClient()


    def flushInput(self):
         pass


    def write(self, data):
        if (self.isConnected == False):
            return
        length = len(data)
        totalsent = 0

        while totalsent < length:
            sent = self.socket.send(data[totalsent:])
            if sent == 0:
                raise RuntimeError("RosSerialServer.write() socket connection broken")
            totalsent = totalsent + sent


    def read(self, rqsted_length):
        self.msg = ''
        if (self.isConnected == False):
            return self.msg

        while len(self.msg) < rqsted_length:
            chunk = self.socket.recv(rqsted_length - len(self.msg))
            if chunk == '':
                raise RuntimeError("RosSerialServer.read() socket connection broken")
            self.msg = self.msg + chunk
        return self.msg


    def inWaiting(self):
        try: # the caller checks just for <1, so we'll peek at just one byte
            chunk = self.socket.recv(1, socket.MSG_DONTWAIT|socket.MSG_PEEK)
            if chunk == '':
                raise RuntimeError("RosSerialServer.inWaiting() socket connection broken")
            return len(chunk)
        except socket.error as e:
            if e.args[0] == errno.EWOULDBLOCK:
                return 0
            raise
