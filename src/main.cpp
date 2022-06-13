#include <ros/ros.h>

#include "faduino/PanelOut.h"

#include "faduino/main.h"

#include "../arduino/daemon/protocol_serial.h"

ValueOutput srvValueOutput;

bool servicePanelOutCallback(faduino::PanelOut::Request &req, faduino::PanelOut::Response &res) {
    #if 0
    printf("[req:led_green] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        req.led_green.onTime, req.led_green.offTime, req.led_green.targetCount, req.led_green.lastState, req.led_green.update);
    printf("[req:led_red  ] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        req.led_red.onTime, req.led_red.offTime, req.led_red.targetCount, req.led_red.lastState, req.led_red.update);
    printf("[req:buzzer   ] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        req.buzzer.onTime, req.buzzer.offTime, req.buzzer.targetCount, req.buzzer.lastState, req.buzzer.update);
    printf("[req:led_start] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        req.led_start.onTime, req.led_start.offTime, req.led_start.targetCount, req.led_start.lastState, req.led_start.update);
    printf("[req:led_stop ] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        req.led_stop.onTime, req.led_stop.offTime, req.led_stop.targetCount, req.led_stop.lastState, req.led_stop.update);
    printf("[req:rel_break] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        req.rel_break.onTime, req.rel_break.offTime, req.rel_break.targetCount, req.rel_break.lastState, req.rel_break.update);
    #endif

    #if 0
    printf("sizeof(srvValueOutput): %ld\n", sizeof(srvValueOutput));
    printf("sizeof(req): %ld\n", sizeof(req));
    #endif

    memcpy(&srvValueOutput, &req, sizeof(srvValueOutput));

    #if 0
    printf("[srv:led_green] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        srvValueOutput.led_green.onTime, srvValueOutput.led_green.offTime, srvValueOutput.led_green.targetCount, srvValueOutput.led_green.lastState, srvValueOutput.led_green.update);
    printf("[srv:led_red  ] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        srvValueOutput.led_red.onTime, srvValueOutput.led_red.offTime, srvValueOutput.led_red.targetCount, srvValueOutput.led_red.lastState, srvValueOutput.led_red.update);
    printf("[srv:buzzer   ] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        srvValueOutput.buzzer.onTime, srvValueOutput.buzzer.offTime, srvValueOutput.buzzer.targetCount, srvValueOutput.buzzer.lastState, srvValueOutput.buzzer.update);
    printf("[srv:led_start] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        srvValueOutput.led_start.onTime, srvValueOutput.led_start.offTime, srvValueOutput.led_start.targetCount, srvValueOutput.led_start.lastState, srvValueOutput.led_start.update);
    printf("[srv:led_stop ] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        srvValueOutput.led_stop.onTime, srvValueOutput.led_stop.offTime, srvValueOutput.led_stop.targetCount, srvValueOutput.led_stop.lastState, srvValueOutput.led_stop.update);
    printf("[srv:rel_break] on: %5d, off: %5d, cnt: %2d, last: %1d, update: %1d\n",
        srvValueOutput.rel_break.onTime, srvValueOutput.rel_break.offTime, srvValueOutput.rel_break.targetCount, srvValueOutput.rel_break.lastState, srvValueOutput.rel_break.update);
    #endif
    
#define SRV_SUCCESS	true
    res.success = SRV_SUCCESS;

    return true;
}

int main(int argc, char* argv[]) {
    std::string nodeName = "faduino";
    ros::init(argc, argv, nodeName.c_str());
    ros::NodeHandle nh("~");
    
    int main_hz;
    int tcp_port;

    // 파라미터 초기화
    #if 1
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~tcp_port", tcp_port);
    #else
    nh.getParam("main_hz", main_hz);
    nh.getParam("tcp_port", tcp_port);
    #endif

    // 패널 출력 서비스
    ros::ServiceServer service_panel_out = nh.advertiseService("/faduino/panel_out", servicePanelOutCallback);
    
    ros::Rate r(main_hz);

    double ts_now, ts_pre;

    ts_now = ts_pre = ros::Time::now().toSec();

    while (ros::ok()) {
        ts_now = ros::Time::now().toSec();

#define TEST_LOOP_SEC 1.0 
        if ((ts_now-ts_pre) > TEST_LOOP_SEC) {
            ts_pre = ts_now;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}