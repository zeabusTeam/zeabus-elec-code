/*
 * Author: Natchanan Thongtem
 * Created on: 09/06/2019
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "main.hpp"

void v_log_node_status( const ros::Publisher &x_publisher, 
                        const std::string &x_description_assigned );

void v_log_hardware_error(  const ros::Publisher &x_publisher,
                            const int64_t &l_error_code_assigned, 
                            const int64_t &l_hardware_status_code_assigned, 
                            const std::string &x_description_assigned );

void v_log_hardware_error_fatal(    const ros::Publisher &x_publisher, 
                                    const int64_t &l_error_code_assigned, 
                                    const int64_t &l_hardware_status_code_assigned, 
                                    const std::string &x_description_assigned );

void v_log_action(  const ros::Publisher &x_publisher, 
                    const int64_t &l_action_assigned, 
                    const int64_t &l_value1_assigned, 
                    const int64_t &l_value2_assigned, 
                    const std::string &x_description_assigned );

#endif
