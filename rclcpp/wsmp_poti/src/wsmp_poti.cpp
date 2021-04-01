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

#include <string>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "wx_msgs/msg/facx_poti_indication.hpp"

#include "wx_msgs/msg/facx_wsmp_request.hpp"
#include "wx_msgs/msg/facx_wsmp_confirm.hpp"
#include "wx_msgs/msg/facx_wsmp_indication.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace wx_msgs::msg;

// The constant topic name is WX protocol. Do not change.
const std::string kTOPIC_FACX_POTI_IND = "wx/facx/poti/ind";
const std::string kTOPIC_FACX_WSMP_REQ = "wx/facx/wsmp/req";
const std::string kTOPIC_FACX_WSMP_CFM = "wx/facx/wsmp/cfm";
const std::string kTOPIC_FACX_WSMP_IND = "wx/facx/wsmp/ind";

enum kWsmpValidity {
  SEC_TYPE                    = (0x01 << 0),
  SIGNER_IDENT_TYPE           = (0x01 << 1),
  SOURCE_MAC_ADDRESS          = (0x01 << 2),
  CHANNEL_IDENTIFIER          = (0x01 << 3),
  TIME_SLOT                   = (0x01 << 4),
  FLOW_SLOT                   = (0x01 << 5),
  DATA_RATE                   = (0x01 << 6),
  TRANSMIT_POWER_LEVEL        = (0x01 << 7),
  USER_PRIORITY               = (0x01 << 8),
  EXPIRY_TIME                 = (0x01 << 9),
  PEER_MAC_ADDRESS            = (0x01 << 10),
  PROVIDER_SERVICE_IDENTIFIER = (0x01 << 11),
  WSMP_MAX                    = (0x01 << 12)
  // LENGTH, NOT USED
  // DATA,   NOT USED
};

enum eFixStatus {
  NO,
  TIME,
  FIX2D,  
  FIX3D,  
  FIX_MAX
};

enum kPotiIndValiditiy {
  FIX_STATUS                         = (0x01 << 0),
  FIX_TYPE                           = (0x01 << 1),
  NUM_USED_SV                        = (0x01 << 2),
  NUM_VISIBLE_SV                     = (0x01 << 3),
  PDOP                               = (0x01 << 4), 
  TIMESTAMP                          = (0x01 << 5),
  LATITUDE                           = (0x01 << 6),
  LONGITUDE                          = (0x01 << 7),
  ELEVATION_MSL                      = (0x01 << 8),
  ELEVATION_ELLIPSOID                = (0x01 << 9),
  SPEED_GROUND                       = (0x01 << 10),
  SPEED_VEHICLE                      = (0x01 << 11),
  HEADING_MOTION                     = (0x01 << 12),
  HEADING_VEHICLE                    = (0x01 << 13),
  LONGITUDINAL_ACCELERATION          = (0x01 << 14),
  LATERAL_ACCELERATION               = (0x01 << 15),
  VERTICAL_ACCELERATION              = (0x01 << 16),
  YAW_RATE                           = (0x01 << 17),
  SEMI_MAJOR_AXIS_ACCURACY           = (0x01 << 18),
  SEMI_MINOR_AXIS_ACCURACY           = (0x01 << 19),
  SEMI_MAJOR_ORIENTAtion             = (0x01 << 20),
  TIME_ACCURACY                      = (0x01 << 21),
  LATITUDE_ACCURACY                  = (0x01 << 22),
  LONGITUDE_ACCURACY                 = (0x01 << 23),
  ELEVATION_ACCURACY                 = (0x01 << 24),
  SPEED_ACCURACY                     = (0x01 << 25),
  HEADING_ACCURACY                   = (0x01 << 26),
  LONGITUDINAL_ACCELERATION_ACCURACY = (0x01 << 27),
  LATTERAL_ACCELERATION_ACCURACY     = (0x01 << 28),
  VERTICAL_ACCELERATION_ACCURACY     = (0x01 << 29),
  YAW_RATE_ACCURACY                  = (0x01 << 30),
  POTI_MAX                           = (0x01 << 31)
};

enum eResultCode {
  ACCEPTED,                      
  REJECTED_MAX_LENGTH_EXCCEEDED, 
  REJECTED_UNSPECIFIED,          
  RC_MAX          
};

class WsmpPoti: public rclcpp::Node
{
public:
  WsmpPoti()
  : Node("WsmpPoti"), seq_(0)
  {
    // initialize timers, If you want to change current(10Hz) period, change the 100ms 
    timer_ = this->create_wall_timer(100ms, std::bind(&WsmpPoti::on_timer_100ms, this));

    // initialize publisher, subscriber
    sub_facx_poti_ind_ = this->create_subscription<FacxPotiIndication>(
      kTOPIC_FACX_POTI_IND, 10, std::bind(&WsmpPoti::facx_poti_ind, this, _1));

    pub_facx_wsmp_req_ = this->create_publisher<FacxWsmpRequest>(kTOPIC_FACX_WSMP_REQ, 10);

    sub_facx_wsmp_cfm_ = this->create_subscription<FacxWsmpConfirm>(
      kTOPIC_FACX_WSMP_CFM, 10, std::bind(&WsmpPoti::facx_wsmp_cfm, this, _1));

    sub_facx_wsmp_ind_ = this->create_subscription<FacxWsmpIndication>(
      kTOPIC_FACX_WSMP_IND, 10, std::bind(&WsmpPoti::facx_wsmp_ind, this, _1));
  }

private:
  /**
   * Timer Handler for Fac-WSMP.request
   * 
   * This handler sends a request to transmit WSM at 10 Hz to device.
   */
  void on_timer_100ms()
  {
    auto req = FacxWsmpRequest();

    auto now = std::chrono::high_resolution_clock::now();
    uint64_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (now.time_since_epoch()).count();
    req.wx_header.nts = nsec; 
    req.wx_header.seq = seq_++; 

    req.user_priority               = 5;    // no ciritical event flags, SAE J2945/1
    req.provider_service_identifier = 0x7F; // 127

    std::string str("This is a sample payload of FacWsmpRequest.");
    std::vector<uint8_t> data(str.begin(), str.end()); 
    req.data = data;

    req.validity = USER_PRIORITY |
                   PROVIDER_SERVICE_IDENTIFIER;

    pub_facx_wsmp_req_->publish(req);

    RCLCPP_INFO(this->get_logger(), 
      "[PUB][FACX-WSMP-REQ] nts: %.9lf, seq: %llu", double(req.wx_header.nts) * 1e-9, 
                                                   req.wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     WSM Payload Size: %d",   req.data.size());
    RCLCPP_INFO(this->get_logger(), 
      "                     User Priority   : %d",   req.user_priority);
    RCLCPP_INFO(this->get_logger(),  
      "                     PSID            : 0x%X", req.provider_service_identifier);
  }

  /**
   * Callback Function for Fac-POTI.indication
   * 
   * This function receives POTI(Positioning and Timing) information from device. 
   */
  void facx_poti_ind(const FacxPotiIndication::SharedPtr ind) const
  {
    RCLCPP_INFO(this->get_logger(), 
      "[SUB][FACX-POTI-IND] nts: %.9lf, seq: %llu", double(ind->wx_header.nts) * 1e-9, 
                                                   ind->wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     Fix Status %s: %d (0: NO, 1: TIME, 2: 2D FIX, 3: 3D FIX)",
      ((ind->validity & FIX_STATUS) ? "(valid)  " : "(invalid)"),
      ((ind->validity & FIX_STATUS) ? ind->fix_status : 0));
    RCLCPP_INFO(this->get_logger(), 
      "                     Used SV    %s: %d ",
      ((ind->validity & NUM_USED_SV) ? "(valid)  " : "(invalid)"),
      ((ind->validity & NUM_USED_SV) ? ind->num_used_sv: 0));
    RCLCPP_INFO(this->get_logger(), 
      "                     Latitude   %s: %11.7lf [deg]",   // degree
      ((ind->validity & LATITUDE) ? "(valid)  " : "(invalid)"),
      ((ind->validity & LATITUDE) ? (double)ind->latitude  * 1e-7 : 0.00));
    RCLCPP_INFO(this->get_logger(), 
      "                     Longitude  %s: %11.7lf [deg]",   // degree
      ((ind->validity & LONGITUDE) ? "(valid)  " : "(invalid)"),
      ((ind->validity & LONGITUDE) ? (double)ind->longitude * 1e-7 : 0.00));
    RCLCPP_INFO(this->get_logger(), 
      "                     Elevation  %s: %11.1lf [meter]", // meter 
      ((ind->validity & ELEVATION_ELLIPSOID) ? "(valid)  " : "(invalid)"),
      ((ind->validity & ELEVATION_ELLIPSOID) ? (double)ind->elevation_ellipsoid * 0.1 : 0.00));
    RCLCPP_INFO(this->get_logger(), 
      "                     Speed      %s: %11.2lf [m/s]",   // m/s 
      ((ind->validity & SPEED_GROUND) ? "(valid)  " : "(invalid)"),
      ((ind->validity & SPEED_GROUND) ? (double)ind->speed_ground * 0.02 : 0.00));
  }

  /**
   * Callback Function for Fac-WSMP.confirm
   * 
   * This function receives response for Fac-WSMP.request from device. 
   */
  void facx_wsmp_cfm(const FacxWsmpConfirm::SharedPtr cfm) const
  {
    RCLCPP_INFO(this->get_logger(), 
      "[SUB][FACX-WSMP-CFM] nts: %.9lf, seq: %llu", double(cfm->wx_header.nts) * 1e-9, 
                                                   cfm->wx_header.seq);
    if (cfm->result_code == ACCEPTED) {
      RCLCPP_INFO(this->get_logger(), 
      "                     Result Code: ACCEPTED");
    }
    else if (cfm->result_code == REJECTED_MAX_LENGTH_EXCCEEDED) {
      RCLCPP_INFO(this->get_logger(), 
      "                     Result Code: REJECTED_MAX_LENGTH_EXCCEEDED");
    }
    else if (cfm->result_code == REJECTED_UNSPECIFIED) {
      RCLCPP_INFO(this->get_logger(), 
      "                     Result Code: REJECTED_UNSPECIFIED");
    }
    else {
      RCLCPP_INFO(this->get_logger(), 
      "                     Result Code: UNKNOWN");
    }
  }

  /**
   * Callback Function for Fac-WSMP.indication
   * 
   * This function receives WSM packet as Fac-WSMP.indication from device.
   */
  void facx_wsmp_ind(const FacxWsmpIndication::SharedPtr ind) const
  {
    RCLCPP_INFO(this->get_logger(), 
      "[SUB][FACX-WSMP-IND] nts: %.9lf, seq: %llu", double(ind->wx_header.nts) * 1e-9, 
                                                   ind->wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     WSM payload size: %d", ind->data.size());
  }

  // member variable for WSMP request timer
  rclcpp::TimerBase::SharedPtr timer_;

  // member variable for POTI publisher
  rclcpp::Subscription<FacxPotiIndication>::SharedPtr sub_facx_poti_ind_;

  // member variable for WSMP publisher, subscriber
  rclcpp::Publisher   <FacxWsmpRequest>::SharedPtr    pub_facx_wsmp_req_;
  rclcpp::Subscription<FacxWsmpIndication>::SharedPtr sub_facx_wsmp_ind_;
  rclcpp::Subscription<FacxWsmpConfirm>::SharedPtr    sub_facx_wsmp_cfm_;

  uint64_t seq_;
};

int main(int argc, char *argv[])
{
  // ROS2 environment initialize
  rclcpp::init(argc, argv);
  // ROS2 execute main thread
  rclcpp::spin(std::make_shared<WsmpPoti>());
  // ROs2 shutdown main thread 
  rclcpp::shutdown(); 

  return EXIT_SUCCESS;
}
