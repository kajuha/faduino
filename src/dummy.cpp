#include "faduino/PanelOut.h"

#include <ros/ros.h>

std::string id;

bool panelOutCallback(faduino::PanelOut::Request &req, faduino::PanelOut::Response &res) {
    ROS_INFO("[%s] %s", id.c_str(), __FUNCTION__);
    if (req.buzzer.update) {
        printf("buzzer updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.buzzer.targetCount, req.buzzer.onTime, req.buzzer.offTime, req.buzzer.lastState, req.buzzer.order);
    }
    if (req.md_power.update) {
        printf("md_power updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.md_power.targetCount, req.md_power.onTime, req.md_power.offTime, req.md_power.lastState, req.md_power.order);
    }
    if (req.md_estop.update) {
        printf("md_estop updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.md_estop.targetCount, req.md_estop.onTime, req.md_estop.offTime, req.md_estop.lastState, req.md_estop.order);
    }
    if (req.led_start.update) {
        printf("led_start updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.led_start.targetCount, req.led_start.onTime, req.led_start.offTime, req.led_start.lastState, req.led_start.order);
    }
    if (req.led_stop.update) {
        printf("led_stop updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.led_stop.targetCount, req.led_stop.onTime, req.led_stop.offTime, req.led_stop.lastState, req.led_stop.order);
    }
    if (req.bat_relay.update) {
        printf("bat_relay updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.bat_relay.targetCount, req.bat_relay.onTime, req.bat_relay.offTime, req.bat_relay.lastState, req.bat_relay.order);
    }
    if (req.out_spare1.update) {
        printf("out_spare1 updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.out_spare1.targetCount, req.out_spare1.onTime, req.out_spare1.offTime, req.out_spare1.lastState, req.out_spare1.order);
    }
    if (req.out_spare2.update) {
        printf("out_spare2 updated[cnt:%d, on:%d, off:%d, last:%d, order:%d].\n",
            req.out_spare2.targetCount, req.out_spare2.onTime, req.out_spare2.offTime, req.out_spare2.lastState, req.out_spare2.order);
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
