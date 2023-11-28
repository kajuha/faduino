#include "faduino/PanelOut.h"

#include <ros/ros.h>

std::string id;

bool panelOutCallback(faduino::PanelOut::Request &req, faduino::PanelOut::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    if (req.buzzer.update) {
        printf("buzzer updated.\n");
    }
    if (req.md_power.update) {
        printf("md_power updated.\n");
    }
    if (req.md_estop.update) {
        printf("md_estop updated.\n");
    }
    if (req.led_start.update) {
        printf("led_start updated.\n");
    }
    if (req.led_stop.update) {
        printf("led_stop updated.\n");
    }
    if (req.bat_relay.update) {
        printf("bat_relay updated.\n");
    }
    if (req.out_spare1.update) {
        printf("out_spare1 updated.\n");
    }
    if (req.out_spare2.update) {
        printf("out_spare2 updated.\n");
    }
    
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
