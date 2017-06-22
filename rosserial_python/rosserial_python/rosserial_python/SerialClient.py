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

# import thread
import multiprocessing
from serial import *
import io
import _thread as thread

from builtin_interfaces.msg import Time
from rosserial_msgs.msg import *
from rosserial_msgs.srv import *

import diagnostic_msgs.msg

import errno
import signal
import socket
import struct
import time

try:
    from termcolor import colored
except ImportError as e:
    print("Install termcolor for colored output")
    print("  sudo apt-get install python3-termcolor")
    def colored(string, color):
        print(string)


def loginfo(text):
    """
    TODO: rclpy.loginfo
    """
    print(colored(text, 'cyan'))


def logerr(text):
    """
    TODO: rclpy.logerr
    """
    print(colored(text, 'red'))


def dbginfo(text):
    """
    TODO: rclpy.dbginfo
    """
    print(colored(text, 'yellow'))


def load_pkg_module(package, directory):
    #check if its in the python path
    path = sys.path
    try:
        imp.find_module(package)
        m = __import__( package + '.' + directory )
    except:
        rospy.logerr( "Cannot import package : %s"% package )
        rospy.logerr( "sys.path was " + str(path) )
        return None
    return m


def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)


def load_service(package,service):
    s = load_pkg_module(package, 'srv')
    s = getattr(s, 'srv')
    srv = getattr(s, service)
    mreq = getattr(s, service+"Request")
    mres = getattr(s, service+"Response")
    return srv,mreq,mres


class SerialClient:
    """
        ServiceServer responds to requests from the serial device.
    """

    def __init__(self, port=None, baud=57600, timeout=5.0):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.mutex = thread.allocate_lock()

        self.lastsync = time.time()
        self.lastsync_lost = time.time()
        self.timeout = timeout
        self.synced = False

        # self.pub_diagnostics = rclpy.create_Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=10)

        if port == None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            # assume its a filelike object
            self.port=port
        else:
            # open a specific port
            try:
                self.port = Serial(port, baud, timeout=self.timeout*0.5)
            except SerialException as e:
                logerr("Error opening serial: %s" % e)
                rclpy.shutdown()
                raise SystemExit

        self.port.timeout = 0.01  # Edit the port timeout

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        # hydro introduces protocol ver2 which must match node_handle.h
        # The protocol version is sent as the 2nd sync byte emitted by each end
        self.protocol_ver1 = '\xff'
        self.protocol_ver2 = '\xfe'
        self.protocol_ver = self.protocol_ver2

        self.publishers = dict()  # id:Publishers
        self.subscribers = dict() # topic:Subscriber
        self.services = dict()    # topic:Service

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        time.sleep(2.0) # TODO
        self.requestTopics()
        self.lastsync = time.time()

        signal.signal(signal.SIGINT, self.txStopRequest)


    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.port.flushInput()
        # request topic sync
        ser_msg = ("\xff" + self.protocol_ver + "\x00\x00\xff\x00\x00\xff").encode()
        self.port.write(ser_msg)


    def txStopRequest(self, signal, frame):
        """ send stop tx request to arduino when receive SIGINT(Ctrl-c)"""
        self.port.flushInput()
        self.port.write(("\xff" + self.protocol_ver + "\x00\x00\xff\x0b\x00\xf4").encode())
        # tx_stop_request is x0b
        loginfo("Send tx stop request")
        sys.exit(0)


    def tryRead(self, length):
        try:
            read_start = time.time()
            read_current = read_start
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and read_current - read_start < self.timeout:
                received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    result.extend(received)
                    bytes_remaining -= len(received)
                read_current = time.time()

            if bytes_remaining != 0:
                dbginfo("Serial Port read returned short (expected %d bytes, received %d instead)."
                       % (length, length - bytes_remaining))
                raise IOError()
            return bytes(result)
        except Exception as e:
            dbginfo("Serial Port read failure: %s", e)
            raise IOError()


    def run(self):
        """ Forward recieved messages to appropriate publisher. """
        data = ''
        while rclpy.ok():
            if (int(time.time() - self.lastsync)) > (self.timeout):
                if (self.synced == True):
                    logerr("Lost sync with device, restarting...")
                else:
                    logerr("Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = time.time() # rospy.Time.now()
                # self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "no sync with device")
                self.requestTopics()
                self.lastsync = time.time()

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
            try:
                if self.port.inWaiting() < 1:
                    time.sleep(0.5)
                    self.port.read()
                    continue

                flag = [0,0]
                flag[0] = self.tryRead(1)
                if (flag[0] != '\xff'):
                    continue

                flag[1] = self.tryRead(1)
                if ( flag[1] != self.protocol_ver):
                    # self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client")
                    logerr("Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client")
                    protocol_ver_msgs = {'\xff': 'Rev 0 (rosserial 0.4 and earlier)', '\xfe': 'Rev 1 (rosserial 0.5+)', '\xfd': 'Some future rosserial version'}
                    if (flag[1] in protocol_ver_msgs):
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    loginfo("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                msg_len_bytes = self.tryRead(2)
                msg_length, = struct.unpack("<h", msg_len_bytes)

                msg_len_chk = self.tryRead(1)
                msg_len_checksum = sum(map(ord, msg_len_bytes)) + ord(msg_len_chk)

                if msg_len_checksum % 256 != 255:
                    loginfo("wrong checksum for msg length, length %d" %(msg_length))
                    loginfo("chk is %d" % ord(msg_len_chk))
                    continue

                # topic id (2 bytes)
                topic_id_header = self.tryRead(2)
                topic_id, = struct.unpack("<h", topic_id_header)

                try:
                    msg = self.tryRead(msg_length)
                except IOError:
                    # self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Packet Failed : Failed to read msg data")
                    loginfo("Packet Failed :  Failed to read msg data")
                    loginfo("msg len is %d",len(msg))
                    raise

                # checksum for topic id and msg
                chk = self.tryRead(1)
                checksum = sum(map(ord, topic_id_header) ) + sum(map(ord, msg)) + ord(chk)

                if checksum % 256 == 255:
                    self.synced = True
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        logerr("Tried to publish before configured, topic id %d" % topic_id)
                    time.sleep(0.001)
                else:
                    dbginfo("wrong checksum for topic id and msg")

            except IOError:
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                self.requestTopics()


    def setPublishSize(self, bytes):
        if self.buffer_out < 0:
            self.buffer_out = bytes
            loginfo("Note: publish buffer size is %d bytes" % self.buffer_out)


    def setSubscribeSize(self, bytes):
        if self.buffer_in < 0:
            self.buffer_in = bytes
            loginfo("Note: subscribe buffer size is %d bytes" % self.buffer_in)


    def setupPublisher(self, data):
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            loginfo("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type) )
        except Exception as e:
            logerr("Creation of publisher failed: %s", e)


    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in self.subscribers.keys():
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                loginfo("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )
            elif msg.message_type != self.subscribers[msg.topic_name].message._type:
                old_message_type = self.subscribers[msg.topic_name].message._type
                self.subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                loginfo("Change the message type of subscriber on %s from [%s] to [%s]" % (msg.topic_name, old_message_type, msg.message_type) )
        except Exception as e:
            logerr("Creation of subscriber failed: %s", e)


    def setupServiceServerPublisher(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceServer(msg, self)
                loginfo("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
        except Exception as e:
            logerr("Creation of service server failed: %s", e)


    def setupServiceServerSubscriber(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceServer(msg, self)
                loginfo("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
        except Exception as e:
            logerr("Creation of service server failed: %s", e)


    def setupServiceClientPublisher(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceClient(msg, self)
                loginfo("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
        except Exception as e:
            logerr("Creation of service client failed: %s", e)


    def setupServiceClientSubscriber(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except:
                srv = ServiceClient(msg, self)
                loginfo("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
        except Exception as e:
            logerr("Creation of service client failed: %s", e)


    def handleTimeRequest(self, data):
        """ Respond to device with system time. """
        t = Time()
        t.data = time.time()
        data_buffer = StringIO.StringIO()
        t.serialize(data_buffer)
        self.send(TopicInfo.ID_TIME, data_buffer.getvalue())
        self.lastsync = time.now()


    def handleParameterRequest(self, data):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        req = RequestParamRequest()
        req.deserialize(data)
        resp = RequestParamResponse()
        logerr("handleParameterRequest not yet implemented for ROS 2!")
        """
        try:
            param = rospy.get_param(req.name)
        except KeyError:
        """
        logerr("Parameter %s does not exist" % req.name)
        #    return

        if param == None:
            logerr("Parameter %s does not exist"%req.name)
            return

        if (type(param) == dict):
            logerr("Cannot send param %s because it is a dictionary"%req.name)
            return
        if (type(param) != list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t!= type(p):
                logerr('All Paramers in the list %s must be of the same type'%req.name)
                return
        if (t == int):
            resp.ints= param
        if (t == float):
            resp.floats=param
        if (t == str):
            resp.strings = param
        data_buffer = StringIO.StringIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())


    def handleLoggingRequest(self, data):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if (msg.level == Log.ROSDEBUG):
            dbginfo(msg.msg)
        elif(msg.level== Log.INFO):
            loginfo(msg.msg)
        elif(msg.level== Log.WARN):
            dbginfo(msg.msg)
        elif(msg.level== Log.ERROR):
            logerr(msg.msg)
        elif(msg.level==Log.FATAL):
            logerr(msg.msg)


    def send(self, topic, msg):
        """ Send a message on a particular topic to the device. """
        with self.mutex:
            length = len(msg)
            if self.buffer_in > 0 and length > self.buffer_in:
                logerr("Message from ROS network dropped: message larger than buffer.")
                print(msg)
                return -1
            else:
                    #modified frame : header(2 bytes) + msg_len(2 bytes) + msg_len_chk(1 byte) + topic_id(2 bytes) + msg(x bytes) + msg_topic_id_chk(1 byte)
                    # second byte of header is protocol version
                    data = "\xff" + self.protocol_ver  + chr(length&255) + chr(length>>8) + chr(msg_len_checksum) + chr(topic&255) + chr(topic>>8)
                    data = data + msg
                    self.port.write(data)
                    return length

    """
    TODO:

    def sendDiagnostics(self, level, msg_text):
        msg = diagnostic_msgs.msg.DiagnosticArray()
        status = diagnostic_msgs.msg.DiagnosticStatus()
        status.name = "rosserial_python"
        msg.header.stamp = rospy.Time.now()
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[0].key="last sync"
        if self.lastsync.to_sec()>0:
            status.values[0].value=time.ctime(self.lastsync.to_sec())
        else:
            status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
        status.values[1].value=time.ctime(self.lastsync_lost.to_sec())

        self.pub_diagnostics.publish(msg)
    """
