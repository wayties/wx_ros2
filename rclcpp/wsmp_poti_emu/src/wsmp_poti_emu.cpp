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
  RESET_MAC_ADDRESS           = (0x01 << 3),
  CHANNEL_IDENTIFIER          = (0x01 << 4),
  TIME_SLOT                   = (0x01 << 5),
  DATA_RATE                   = (0x01 << 6),
  TRANSMIT_POWER_LEVEL        = (0x01 << 7),
  USER_PRIORITY               = (0x01 << 8),
  EXPIRY_TIME                 = (0x01 << 9),
  // NOT USED 
  // NOT USED 
  PEER_MAC_ADDRESS            = (0x01 << 12),
  PROVIDER_SERVICE_IDENTIFIER = (0x01 << 13),
};

enum eFixStatus {
  NO,
  TIME,
  FIX2D,  
  FIX3D,  
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
  YAW_RATE_ACCURACY                  = (0x01 << 30)
};

enum eResultCode {
  ACCEPTED,                      
  REJECTED_MAX_LENGTH_EXCCEEDED, 
  REJECTED_UNSPECIFIED,          
};

class WsmpPotiEmu: public rclcpp::Node
{
public:
  WsmpPotiEmu()
  : Node("WsmpPotiEmu"), poti_ind_seq_(0), wsmp_ind_seq_(0)
  {
    // initialize timers, If you want to change current(10Hz) period, change the 100ms 
    poti_ind_timer_ = this->create_wall_timer(100ms, std::bind(&WsmpPotiEmu::on_poti_ind_timer_100ms, this));
    wsmp_ind_timer_ = this->create_wall_timer(100ms, std::bind(&WsmpPotiEmu::on_wsmp_ind_timer_100ms, this));

    // initialize publisher, subscriber
    pub_facx_poti_ind_ = this->create_publisher<FacxPotiIndication>(kTOPIC_FACX_POTI_IND, 10);

    sub_facx_wsmp_req_ = this->create_subscription<FacxWsmpRequest>(
      kTOPIC_FACX_WSMP_REQ, 10, std::bind(&WsmpPotiEmu::facx_wsmp_req, this, _1));

    pub_facx_wsmp_cfm_ = this->create_publisher<FacxWsmpConfirm>(kTOPIC_FACX_WSMP_CFM, 10);

    pub_facx_wsmp_ind_ = this->create_publisher<FacxWsmpIndication>(kTOPIC_FACX_WSMP_IND, 10);
  }

private:
  /**
   * Timer Handler for Fac-POTI.indication
   * 
   * This handler sends a indication with static POTI(Positioning and Timing) information at 10 Hz.
   */
  void on_poti_ind_timer_100ms()
  {
    auto poti = FacxPotiIndication();

    auto now = std::chrono::high_resolution_clock::now();
    uint64_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (now.time_since_epoch()).count();
    poti.wx_header.nts = nsec;            
    poti.wx_header.seq = poti_ind_seq_++;

    // set a static FacPotiIndication data
    poti.fix_status          = FIX3D;
    poti.latitude            =  374110107; // 1/10 micro degree, SAE J2735
    poti.longitude           = 1270954984; // 1/10 micro degree, SAE J2735
    poti.elevation_ellipsoid = 1123;       // Units of 10 cm(SAE J2735), Ellipsoid(SAE J2945/1)
    poti.speed_ground        = 5482;       // Units of 0.02 m/s(SAE J2735)

    poti.validity = FIX_STATUS |
                    // NUM_USED_SV | // invalid case
                    LATITUDE |
                    LONGITUDE |
                    ELEVATION_ELLIPSOID |
                    SPEED_GROUND;

    pub_facx_poti_ind_->publish(poti);

    RCLCPP_INFO(this->get_logger(), 
      "[PUB][FACX-POTI-IND] nts: %.9lf, seq: %llu", double(poti.wx_header.nts) * 1e-9, 
                                                   poti.wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     Fix Status: %s (0: NO, 1: TIME, 2: 2D FIX, 3: 3D FIX)", 
      std::to_string(poti.fix_status).c_str());
    RCLCPP_INFO(this->get_logger(), 
      "                     Latitude : %11.7lf [deg]",   (double)poti.latitude  * 1e-7);
    RCLCPP_INFO(this->get_logger(), 
      "                     Longitude: %11.7lf [deg]",   (double)poti.longitude * 1e-7);
    RCLCPP_INFO(this->get_logger(), 
      "                     Elevation: %11.1lf [meter]", (double)poti.elevation_ellipsoid * 0.1);
    RCLCPP_INFO(this->get_logger(), 
      "                     Speed    : %11.2lf [m/s]",   (double)poti.speed_ground * 0.02);
  }
 
  /**
   * Timer Handler for Fac-WSMP.indication
   * 
   * This handler sends a indication with the static WSMP packet at 10 Hz.
   */
  void on_wsmp_ind_timer_100ms()
  {
    auto wsmp = FacxWsmpIndication();

    auto now = std::chrono::high_resolution_clock::now();
    uint64_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (now.time_since_epoch()).count();
    wsmp.wx_header.nts = nsec;
    wsmp.wx_header.seq = wsmp_ind_seq_++;

    std::string str("This is a sample payload of FacWsmpIndication.");
    std::vector<uint8_t> data(str.begin(), str.end()); 
    wsmp.data = data;

    pub_facx_wsmp_ind_->publish(wsmp);

    RCLCPP_INFO(this->get_logger(), 
      "[PUB][FACX-WSMP-IND] nts: %.9lf, seq: %llu", (double)wsmp.wx_header.nts * 1e-9, 
                                                   wsmp.wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     WSM payload size: %d", wsmp.data.size());
  }

  /**
   * Callback Function for Fac-WSMP.request
   * 
   * This function receives a request for Fac-WSMP.request and sends a response to it as Fac-WSMP.confirm.
   */
  void facx_wsmp_req(const FacxWsmpRequest::SharedPtr req) 
  {
    RCLCPP_INFO(this->get_logger(), 
      "[SUB][FACX-WSMP-REQ] nts: %.9lf, seq: %llu", (double)req->wx_header.nts * 1e-9, 
                                                   req->wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     WSM payload size: %d", req->data.size());
    RCLCPP_INFO(this->get_logger(), 
      "                     User Priority %s: %d",   
      ((req->validity & USER_PRIORITY) ? "(valid)  " : "(invalid)"),
      ((req->validity & USER_PRIORITY) ? req->user_priority : 0));
    RCLCPP_INFO(this->get_logger(),  
      "                     PSID          %s: 0x%X", 
      ((req->validity & PROVIDER_SERVICE_IDENTIFIER) ? "(valid)  " : "(invalid)"),
      ((req->validity & PROVIDER_SERVICE_IDENTIFIER) ? req->provider_service_identifier : 0));

    auto cfm = FacxWsmpConfirm();

    cfm.wx_header.nts = req->wx_header.nts;
    cfm.wx_header.seq = req->wx_header.seq;

    cfm.result_code = ACCEPTED;

    pub_facx_wsmp_cfm_->publish(cfm);

    RCLCPP_INFO(this->get_logger(), 
      "[PUB][FACX-WSMP-CFM] nts: %.9lf, seq: %llu", double(cfm.wx_header.nts) * 1e-9, 
                                                   cfm.wx_header.seq);
    RCLCPP_INFO(this->get_logger(), 
      "                     Result Code: ACCEPTED");
  }

  // member variable for POTI indication timer
  rclcpp::TimerBase::SharedPtr poti_ind_timer_;

  // member variable for WSMP indication timer
  rclcpp::TimerBase::SharedPtr wsmp_ind_timer_;

  // member variable for POTI publisher
  rclcpp::Publisher   <FacxPotiIndication>::SharedPtr pub_facx_poti_ind_;

  // member variable for WSMP publisher, subscriber
  rclcpp::Subscription<FacxWsmpRequest>::SharedPtr    sub_facx_wsmp_req_;
  rclcpp::Publisher   <FacxWsmpConfirm>::SharedPtr    pub_facx_wsmp_cfm_;
  rclcpp::Publisher   <FacxWsmpIndication>::SharedPtr pub_facx_wsmp_ind_;

  uint64_t poti_ind_seq_;
  uint64_t wsmp_ind_seq_;
};

int main(int argc, char *argv[])
{
  // ROS2 environment initialize
  rclcpp::init(argc, argv);
  // ROS2 execute main thread
  rclcpp::spin(std::make_shared<WsmpPotiEmu>());
  // ROs2 shutdown main thread 
  rclcpp::shutdown(); 

  return EXIT_SUCCESS;
}
