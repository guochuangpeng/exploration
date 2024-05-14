//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "attitude_controller/attitude_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  attitudeCtrl* attitudeController = new attitudeCtrl(nh, nh_private);

  dynamic_reconfigure::Server<attitude_controller::AttitudeControllerConfig> srv;
  dynamic_reconfigure::Server<attitude_controller::AttitudeControllerConfig>::CallbackType f;
  f = boost::bind(&attitudeCtrl::dynamicReconfigureCallback, attitudeController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}


