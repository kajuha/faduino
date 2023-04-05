#include "faduino/PanelOut.h"

#include <ros/ros.h>

std::string id;

bool panelOutCallback(faduino::PanelOut::Request &req, faduino::PanelOut::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    req.buzzer;
    req.md_power;
    req.md_estop;
    req.led_start;
    req.led_stop;
    req.bat_relay;
    req.out_spare1;
    req.out_spare2;
    
#define SRV_SUCCESS	true
    res.success = SRV_SUCCESS;

    return true;
}

int main(int argc, char* argv[]) {
    id = "faduino";
    ros::init(argc, argv, id.c_str());
    ros::NodeHandle nh("~");

    int main_hz;
    double PUB_SEC_PERIOD;

    ros::param::get("~main_hz", main_hz);
    ros::param::get("~PUB_SEC_PERIOD", PUB_SEC_PERIOD);

    // service
    ros::ServiceServer srv_panelOut = nh.advertiseService("/faduino/panel_out", panelOutCallback);

    double time_cur = ros::Time::now().toSec();
    double time_pre = time_cur;
    double time_diff;

    ros::Rate r(main_hz);

    while(ros::ok())
    {
        time_cur = ros::Time::now().toSec();

        time_diff = time_cur - time_pre;
        if (time_diff > PUB_SEC_PERIOD) {
            time_pre = time_cur;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
