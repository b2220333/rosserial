/**
 *
 *  \file
 *  \brief      Classes which manage the Publish and Subscribe relationships
 *              that a Session has with its client.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_TOPIC_HANDLERS_H
#define ROSSERIAL_SERVER_TOPIC_HANDLERS_H

#include <rclcpp/rclcpp.hpp>
#include <rosserial_msgs/msg/topic_info.hpp>
#include <rosserial_msgs/srv/request_message_info.hpp>
#include <rosserial_msgs/srv/request_service_info.hpp>
/* #include <topic_tools/shape_shifter.h> */

#define ROS_WARN(stuff) std::cerr<<stuff<<std::endl;

using RequestMessageInfo = rosserial_msgs::srv::RequestMessageInfo;

namespace rosserial_server
{

class Publisher {
public:
  Publisher(auto & nh, const rosserial_msgs::TopicInfo& topic_info) {
    if (!message_service_.isValid()) {
      message_service_ = nh->create_client<>("message_info");
    }

    auto info = std::make_shared<RequestMessageInfo::Request>();
    info.request.type = topic_info.message_type;
    auto result_future = nh->async_send_request(info);
    if (rclcpp::spin_until_future_complete(nh, result_future) != rclcpp::executor::FutureREturnCode::SUCCESS) {
      std::cerr<<"Message"<<topic_info.message_type
               <<"MD5 sum from client does not match that in system."
               <<"Will avoid using system's message definition."
               <<std::endl;
    }
  }

  void handle(std::istream & stream) {
    publisher_.publish(stream.str());
  }

  std::string get_topic() {
    return publisher_.getTopic();
  }

private:
  rclcpp::Publisher publisher_;
  static rclcpp::ServiceClient message_service_;
};

ros::ServiceClient Publisher::message_service_;
typedef boost::shared_ptr<Publisher> PublisherPtr;


class Subscriber {
public:
  Subscriber(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      std::function<void(std::vector<uint8_t>& buffer)> write_fn)
    : write_fn_(write_fn) {
    ros::SubscribeOptions opts;
    opts.init<topic_tools::ShapeShifter>(
        topic_info.topic_name, 1, std::bind(&Subscriber::handle, this, _1));
    opts.md5sum = topic_info.md5sum;
    opts.datatype = topic_info.message_type;
    subscriber_ = nh.subscribe(opts);
  }

  std::string get_topic() {
    return subscriber_.getTopic();
  }

private:
  void handle(const std::shared_ptr<topic_tools::ShapeShifter const>& msg) {
    size_t length = ros::serialization::serializationLength(*msg);
    std::vector<uint8_t> buffer(length);

    std::ostream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

    write_fn_(buffer);
  }

  ros::Subscriber subscriber_;
  boost::function<void(std::vector<uint8_t>& buffer)> write_fn_;
};

typedef boost::shared_ptr<Subscriber> SubscriberPtr;

class ServiceClient {
public:
  ServiceClient(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      std::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn)
    : write_fn_(write_fn) {
    topic_id_ = -1;
    if (!service_info_service_.isValid()) {
      // lazy-initialize the service caller.
      service_info_service_ = nh.serviceClient<rosserial_msgs::RequestServiceInfo>("service_info");
      if (!service_info_service_.waitForExistence(rclcpp::Duration(5.0))) {
        ROS_WARN("Timed out waiting for service_info service to become available.");
      }
    }

    rosserial_msgs::RequestServiceInfo info;
    info.request.service = topic_info.message_type;
    ROS_DEBUG("Calling service_info service for topic name %s",topic_info.topic_name.c_str());
    if (service_info_service_.call(info)) {
      request_message_md5_ = info.response.request_md5;
      response_message_md5_ = info.response.response_md5;
    } else {
      ROS_WARN("Failed to call service_info service. The service client will be created with blank md5sum.");
    }
    ros::ServiceClientOptions opts;
    opts.service = topic_info.topic_name;
    opts.md5sum = service_md5_ = info.response.service_md5;
    opts.persistent = false; // always false for now
    service_client_ = nh.serviceClient(opts);
  }
  void setTopicId(uint16_t topic_id) {
    topic_id_ = topic_id;
  }
  std::string getRequestMessageMD5() {
    return request_message_md5_;
  }
  std::string getResponseMessageMD5() {
    return response_message_md5_;
  }

  void handle(ros::serialization::IStream stream) {
    // deserialize request message
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, request_message_);

    // perform service call
    // note that at present, at least for rosserial-windows a service call returns nothing,
    // so we discard the return value of this call() invocation.
    service_client_.call(request_message_, response_message_, service_md5_);

    // write service response over the wire
    size_t length = ros::serialization::serializationLength(response_message_);
    std::vector<uint8_t> buffer(length);
    std::ostream ostream(&buffer[0], length);
    response_message_ = ostream.str();
    write_fn_(buffer,topic_id_);
  }

private:
  topic_tools::ShapeShifter request_message_;
  topic_tools::ShapeShifter response_message_;
  ros::ServiceClient service_client_;
  static ros::ServiceClient service_info_service_;
  std::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn_;
  std::string service_md5_;
  std::string request_message_md5_;
  std::string response_message_md5_;
  uint16_t topic_id_;
};

ros::ServiceClient ServiceClient::service_info_service_;
typedef std::shared_ptr<ServiceClient> ServiceClientPtr;

}  // namespace

#endif  // ROSSERIAL_SERVER_TOPIC_HANDLERS_H
