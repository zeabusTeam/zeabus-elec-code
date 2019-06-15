/*
 * Author: Natchanan Thongtem
 * Created on: 09/06/2019
 */

#include "logger.hpp"

#include "zeabus_elec_ros/MessageNodeStatus.h"
#include "zeabus_elec_ros/MessageHardwareError.h"
#include "zeabus_elec_ros/MessageAction.h"

void v_log_node_status( const ros::Publisher &x_publisher, 
                        const std::string &x_description_assigned )
{
    zeabus_elec_ros::MessageNodeStatus x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.x_description = x_description_assigned;

    // publish the message
    x_publisher.publish( x_message );

    // print the message to ROS console
    ROS_INFO( x_description_assigned.c_str() );
}

void v_log_hardware_error(  const ros::Publisher &x_publisher,
                            const int64_t &l_error_code_assigned, 
                            const int64_t &l_hardware_status_code_assigned, 
                            const std::string &x_description_assigned )
{
    zeabus_elec_ros::MessageHardwareError x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.l_error_code =            l_error_code_assigned;
    x_message.l_hardware_status_code =  l_hardware_status_code_assigned;

    x_message.x_description = x_description_assigned;

    // publish the message
    x_publisher.publish( x_message );

    // print the message to ROS console
    ROS_ERROR( x_description_assigned.c_str() );
}

void v_log_hardware_error_fatal(    const ros::Publisher &x_publisher, 
                                    const int64_t &l_error_code_assigned, 
                                    const int64_t &l_hardware_status_code_assigned, 
                                    const std::string &x_description_assigned )
{
    zeabus_elec_ros::MessageHardwareError x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.l_error_code =            l_error_code_assigned;
    x_message.l_hardware_status_code =  l_hardware_status_code_assigned;

    x_message.x_description = x_description_assigned;

    // publish the message
    x_publisher.publish( x_message );

    // print the message to ROS console
    ROS_FATAL( x_description_assigned.c_str() );
}

void v_log_action(  const ros::Publisher &x_publisher, 
                    const int64_t &l_action_assigned, 
                    const int64_t &l_value1_assigned, 
                    const int64_t &l_value2_assigned, 
                    const std::string &x_description_assigned )
{
    zeabus_elec_ros::MessageAction x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.l_action = l_action_assigned;
    x_message.l_value1 = l_value1_assigned;
    x_message.l_value2 = l_value2_assigned;

    x_message.x_description = x_description_assigned;

    // publish the message
    x_publisher.publish( x_message );

    // print the message to ROS console
    ROS_INFO( x_description_assigned.c_str() );
}
