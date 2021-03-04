// Copyright 2020-2021 Wayties, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "wx_msgs/msg/fac_poti_indication.hpp"
#include "wx_msgs/msg/fac_poti_status_indication.hpp"

#include "wx_msgs/msg/fac_wsmp_request.hpp"
#include "wx_msgs/msg/fac_wsmp_confirm.hpp"
#include "wx_msgs/msg/fac_wsmp_indication.hpp"
#include "wx_msgs/msg/fac_wsmp_status_indication.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace wx_msgs::msg;

const std::string kTOPIC_FAC_POTI_IND        = "wx/fac/poti/ind";
const std::string kTOPIC_FAC_POTI_STATUS_IND = "wx/fac/poti/status/ind";

const std::string kTOPIC_FAC_WSMP_REQ        = "wx/fac/wsmp/req";
const std::string kTOPIC_FAC_WSMP_CFM        = "wx/fac/wsmp/cfm";
const std::string kTOPIC_FAC_WSMP_IND        = "wx/fac/wsmp/ind";
const std::string kTOPIC_FAC_WSMP_STATUS_IND = "wx/fac/wsmp/status/ind";

class WsmpPoti: public rclcpp::Node
{
public:
  WsmpPoti()
  : Node("WsmpPoti"), seq_(0)
  {
    timer_ = this->create_wall_timer(100ms, std::bind(&WsmpPoti::on_timer_100ms, this));

    sub_fac_poti_ind_ = this->create_subscription<FacPotiIndication>(
      kTOPIC_FAC_POTI_IND, 10, std::bind(&WsmpPoti::fac_poti_ind, this, _1));

    pub_fac_wsmp_req_ = this->create_publisher<FacWsmpRequest>(kTOPIC_FAC_WSMP_REQ, 10);

    sub_fac_wsmp_cfm_ = this->create_subscription<FacWsmpConfirm>(
      kTOPIC_FAC_WSMP_CFM, 10, std::bind(&WsmpPoti::fac_wsmp_cfm, this, _1));

    sub_fac_wsmp_ind_ = this->create_subscription<FacWsmpIndication>(
      kTOPIC_FAC_WSMP_IND, 10, std::bind(&WsmpPoti::fac_wsmp_ind, this, _1));

    sub_fac_wsmp_status_ind_ = this->create_subscription<FacWsmpStatusIndication>(
      kTOPIC_FAC_WSMP_STATUS_IND, 10, std::bind(&WsmpPoti::fac_wsmp_status_ind, this, _1));
  }

private:
  void on_timer_100ms()
  {
    auto req = FacWsmpRequest();

    std::vector<uint8_t> data { 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x20,
                                0x56, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x00 }; // "Hello World!"
    req.data = data;

    pub_fac_wsmp_req_->publish(req);

    RCLCPP_INFO(this->get_logger(), "[WSMP] requsted, seq: %d", int(seq_++));
  }

  void fac_poti_ind(const FacPotiIndication::SharedPtr ind) const
  {
    RCLCPP_INFO(this->get_logger(), "[POTI] indication, seq: %d", ind->wx_header.seq);
  }

  void fac_wsmp_cfm(const FacWsmpConfirm::SharedPtr cfm) const
  {
    RCLCPP_INFO(this->get_logger(), "[WSMP] confirm, seq: %d", cfm->wx_header.seq);
  }

  void fac_wsmp_ind(const FacWsmpIndication::SharedPtr ind) const
  {
    RCLCPP_INFO(this->get_logger(), "[WSMP] indication, seq: %d", ind->wx_header.seq);
  }

  void fac_wsmp_status_ind(const FacWsmpStatusIndication::SharedPtr status) const
  {
    RCLCPP_INFO(this->get_logger(), "[WSMP] status indication, seq: %d", status->wx_header.seq);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<FacPotiIndication>::SharedPtr       sub_fac_poti_ind_;
  rclcpp::Subscription<FacPotiStatusIndication>::SharedPtr sub_fac_poti_status_ind_;

  rclcpp::Publisher   <FacWsmpRequest>::SharedPtr          pub_fac_wsmp_req_;
  rclcpp::Subscription<FacWsmpConfirm>::SharedPtr          sub_fac_wsmp_cfm_;
  rclcpp::Subscription<FacWsmpIndication>::SharedPtr       sub_fac_wsmp_ind_;
  rclcpp::Subscription<FacWsmpStatusIndication>::SharedPtr sub_fac_wsmp_status_ind_;

  uint64_t seq_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WsmpPoti>());
  rclcpp::shutdown(); 

  return EXIT_SUCCESS;
}
