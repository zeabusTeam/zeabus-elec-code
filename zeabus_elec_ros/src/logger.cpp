/*
 * Author: Natchanan Thongtem
 * Created on: 09/06/2019
 */

#include "logger.hpp"

#include "zeabus_elec_ros/MessageNodeStatus.h"
#include "zeabus_elec_ros/MessageHardwareError.h"
#include "zeabus_elec_ros/MessageAction.h"

void v_log_node_status( const ros::Publisher &kx_publisher, 
                        const std::string &kx_description_assigned )
{
    zeabus_elec_ros::MessageNodeStatus x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.x_description = kx_description_assigned;

    // publish the message
    kx_publisher.publish( x_message );

    // print the message to ROS console
    ROS_INFO( kx_description_assigned.c_str() );

    return;
}

void v_log_hardware_error(  const ros::Publisher &kx_publisher,
                            const int &ki_error_code_assigned, 
                            const int &ki_hardware_status_code_assigned, 
                            const std::string &kx_description_assigned )
{
    zeabus_elec_ros::MessageHardwareError x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.l_error_code =            ki_error_code_assigned;
    x_message.l_hardware_status_code =  ki_hardware_status_code_assigned;

    x_message.x_description = kx_description_assigned;

    // publish the message
    kx_publisher.publish( x_message );

    // print the message to ROS console
    ROS_ERROR( kx_description_assigned.c_str() );

    return;
}

void v_log_hardware_error_fatal(    const ros::Publisher &kx_publisher, 
                                    const int &ki_error_code_assigned, 
                                    const int &ki_hardware_status_code_assigned, 
                                    const std::string &kx_description_assigned )
{
    zeabus_elec_ros::MessageHardwareError x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.l_error_code =            ki_error_code_assigned;
    x_message.l_hardware_status_code =  ki_hardware_status_code_assigned;

    x_message.x_description = kx_description_assigned;

    // publish the message
    kx_publisher.publish( x_message );

    // print the message to ROS console
    ROS_FATAL( kx_description_assigned.c_str() );

    return;
}

void v_log_action(  const ros::Publisher &kx_publisher, 
                    const int &ki_action_assigned, 
                    const std::vector<int> &x_vector_value_assigned,
                    const std::string &kx_description_assigned )
{
    zeabus_elec_ros::MessageAction x_message;
    
    // prepare the log message
    x_message.header.stamp = ros::Time::now();

    x_message.l_action = ki_action_assigned;
    for( int item : x_vector_value_assigned )
    {
        x_message.x_vector_value.push_back( item );
    }

    x_message.x_description = kx_description_assigned;

    // publish the message
    kx_publisher.publish( x_message );

    // print the message to ROS console
    ROS_INFO( kx_description_assigned.c_str() );

    return;
}
