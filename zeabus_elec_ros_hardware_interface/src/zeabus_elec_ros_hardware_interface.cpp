/*
 * Author: Natchanan Thongtem
 * Created on: 10/06/2017
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <boost/array.hpp>

#include <string>
#include <algorithm>

#include <zeabus_utility/ServiceDepth.h>
#include <zeabus_utility/SendThrottle.h>
#include <zeabus_utility/Telemetry.h>
#include <zeabus_elec_ros_hardware_interface/PowerSwitchCommand.h>
#include <zeabus_elec_ros_hardware_interface/IOCommand.h>
#include <zeabus_elec_ros_power_dist/power_dist.h>
#include <zeabus_elec_ros_peripheral_bridge/barometer.h>
#include <zeabus_elec_ros_peripheral_bridge/ios_state.h>
#include <zeabus_elec_ros_peripheral_bridge/solenoid_sw.h>
#include <zeabus_elec_ros_etcam/CommandSetThrusterThrottle.h>
#include <zeabus_elec_ros_etcam/Telemetry.h>

#include <libetcam.hpp>

#define ONE_ATM_AS_PSI 14.6959
#define PSI_PER_DEPTH 0.6859

static ros::Publisher planner_switch_publisher;

static ros::Subscriber barometer_subsciber;
static ros::Subscriber ios_state_subsciber;
static ros::Subscriber telemetry_subsciber;

static ros::ServiceServer set_power_switch_on_service_server;
static ros::ServiceServer set_power_switch_off_service_server;
static ros::ServiceServer set_solenoid_on_service_server;
static ros::ServiceServer set_solenoid_off_service_server;
static ros::ServiceServer set_thruster_throttle_service_server;
static ros::ServiceServer get_depth_service_server;
static ros::ServiceServer get_telemetry_service_server;

static ros::ServiceClient power_dist_service_client;
static ros::ServiceClient solenoid_service_client;
static ros::ServiceClient thruster_throttle_service_client;

static double atm_pressure, depth_offset;

static zeabus_utility::ServiceDepth::Response depth_state;
static zeabus_utility::Telemetry::Response telemetry_state;

void barometer_value_to_depth(const zeabus_elec_ros_peripheral_bridge::barometer::ConstPtr& msg)
{
    double barometer_voltage, psi, depth;

    barometer_voltage = (msg->pressureValue) * (5.0 / 1023.0);
    psi = (barometer_voltage - 0.5) * (30.0 / 4.0);

    depth = ((psi - atm_pressure) * PSI_PER_DEPTH) + depth_offset;

    depth_state.depth = depth;
    depth_state.header = msg->header;

    ROS_INFO("pressure sensor analog value : %.4d", msg->pressureValue);
    ROS_INFO("pressure sensor voltage : %.4lf V", barometer_voltage);
    ROS_INFO("pressure : %.4lf psi", psi);
    ROS_INFO("depth : %.4lf meter\n", depth);
}

void send_planner_switch(const zeabus_elec_ros_peripheral_bridge::ios_state::ConstPtr& msg)
{
    std_msgs::Bool planner_switch_msg;
    bool planner_switch_state;

    planner_switch_state = (msg->iosState) & 0x04;

    planner_switch_msg.data = planner_switch_state;

    planner_switch_publisher.publish(planner_switch_msg);
}

void telemetry_parser(const zeabus_elec_ros_etcam::Telemetry::ConstPtr& msg)
{
    boost::array<uint8_t, libetcam::ku_TELEMETRY_SIZE> boost_telemetry;
    std::array<uint8_t, libetcam::ku_TELEMETRY_SIZE> telemetry;
    std::array<libetcam::TelemetryStruct, libetcam::ku_THRUSTER_NUMBER> parsed_telemetry;
    boost::array<zeabus_utility::StructTelemetry, libetcam::ku_THRUSTER_NUMBER> boost_parsed_telemetry;
    
    boost_telemetry = msg->au_telemetry;
    std::copy(boost_telemetry.begin(), boost_telemetry.end(), telemetry.begin());

    parsed_telemetry = libetcam::ax_telemetry_parser(telemetry);

    for(uint8_t i = 0U; i < libetcam::ku_THRUSTER_NUMBER; i++)
    {
        boost_parsed_telemetry[i].temperature = parsed_telemetry[i].u_temperature;
        boost_parsed_telemetry[i].voltage = parsed_telemetry[i].f_voltage;
        boost_parsed_telemetry[i].current = parsed_telemetry[i].f_current;
        boost_parsed_telemetry[i].power_consumption = parsed_telemetry[i].us_power_consumption;
        boost_parsed_telemetry[i].erpm = parsed_telemetry[i].us_erpm;
    }
    
    telemetry_state.telemetry = boost_parsed_telemetry;
    telemetry_state.header = msg->header;
}

bool get_depth(zeabus_utility::ServiceDepth::Request &req,
                    zeabus_utility::ServiceDepth::Response &res)
{
    res = depth_state;

    return true;
}

bool get_telemetry(zeabus_utility::Telemetry::Request &req,
                    zeabus_utility::Telemetry::Response &res)
{
    res = telemetry_state;

    return true;
}

bool set_power_switch_on(zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Request &req,
                    zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Response &res)
{
    zeabus_elec_ros_power_dist::power_dist power_dist_service;

    power_dist_service.request.switchIndex = req.channel;

    if(req.channel >= 6)
    {
        power_dist_service.request.isSwitchHigh = false;
    }
    else
    {
        power_dist_service.request.isSwitchHigh = true;
    }

    res.result = power_dist_service_client.call(power_dist_service);

    return res.result;
}

bool set_power_switch_off(zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Request &req,
                        zeabus_elec_ros_hardware_interface::PowerSwitchCommand::Response &res)
{
    zeabus_elec_ros_power_dist::power_dist power_dist_service;

    power_dist_service.request.switchIndex = req.channel;

    if(req.channel >= 6)
    {
        power_dist_service.request.isSwitchHigh = true;
    }
    else
    {
        power_dist_service.request.isSwitchHigh = false;
    }

    res.result = power_dist_service_client.call(power_dist_service);

    return res.result;
}

bool set_solenoid_on(zeabus_elec_ros_hardware_interface::IOCommand::Request &req,
                    zeabus_elec_ros_hardware_interface::IOCommand::Response &res)
{
    zeabus_elec_ros_peripheral_bridge::solenoid_sw solenoid_service;

    solenoid_service.request.switchIndex = req.channel;
    solenoid_service.request.isSwitchHigh = true;

    res.result = solenoid_service_client.call(solenoid_service);

    return res.result;
}

bool set_solenoid_off(zeabus_elec_ros_hardware_interface::IOCommand::Request &req,
                    zeabus_elec_ros_hardware_interface::IOCommand::Response &res)
{
    zeabus_elec_ros_peripheral_bridge::solenoid_sw solenoid_service;

    solenoid_service.request.switchIndex = req.channel;
    solenoid_service.request.isSwitchHigh = false;

    res.result = solenoid_service_client.call(solenoid_service);

    return res.result;
}

bool set_thruster_throttle(zeabus_utility::SendThrottle::Request &req,
                        zeabus_utility::SendThrottle::Response &res)
{
    zeabus_elec_ros_etcam::CommandSetThrusterThrottle set_thruster_throttle_service;
    bool result = false;

    boost::array<int16_t, libetcam::ku_THRUSTER_NUMBER> thruster_throttle = req.data;

    for(uint8_t i = 0U; i < libetcam::ku_THRUSTER_NUMBER; i++)
    {
        if(thruster_throttle[i] < 0)
        {
            thruster_throttle[i] = (thruster_throttle[i] * (-1)) + 47U;
        }
        else if(thruster_throttle[i] > 0)
        {
            thruster_throttle[i] += 1047U;
        }
    }

    set_thruster_throttle_service.request.aus_thruster_throttle = thruster_throttle;

    result = thruster_throttle_service_client.call(set_thruster_throttle_service);

    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Zeabus_Elec_Hardware_interface");
    ros::NodeHandle nh("/zeabus/elec");

    nh.param<double>("/Zeabus_Elec_Hardware_interface/atm_pressure", atm_pressure, ONE_ATM_AS_PSI);
    nh.param<double>("/Zeabus_Elec_Hardware_interface/depth_offset", depth_offset, 0);

    planner_switch_publisher = nh.advertise<std_msgs::Bool>("/planner_switch", 100);

    barometer_subsciber = nh.subscribe("barometer", 100, barometer_value_to_depth);
    ios_state_subsciber = nh.subscribe("ios_state", 100, send_planner_switch);
    telemetry_subsciber = nh.subscribe("telemetry", 100, telemetry_parser);

    set_power_switch_on_service_server = nh.advertiseService("/power_distribution/switch_on", set_power_switch_on);
    set_power_switch_off_service_server = nh.advertiseService("/power_distribution/switch_off", set_power_switch_off);
    set_solenoid_on_service_server = nh.advertiseService("/io_and_pressure/IO_ON", set_solenoid_on);
    set_solenoid_off_service_server = nh.advertiseService("/io_and_pressure/IO_OFF", set_solenoid_off);
    set_thruster_throttle_service_server = nh.advertiseService("/hardware/thruster_throttle", set_thruster_throttle);
    get_depth_service_server = nh.advertiseService("/sensor/pressure", get_depth);
    get_telemetry_service_server = nh.advertiseService("/hardware/thruster_feedback", get_telemetry);

    power_dist_service_client = nh.serviceClient<zeabus_elec_ros_power_dist::power_dist>("power_switch");
    solenoid_service_client = nh.serviceClient<zeabus_elec_ros_peripheral_bridge::solenoid_sw>("solenoid_sw");
    thruster_throttle_service_client = nh.serviceClient<zeabus_elec_ros_etcam::CommandSetThrusterThrottle>("/zeabus/elec/set_thruster_throttle");

    ros::spin();

    return 0;
}
