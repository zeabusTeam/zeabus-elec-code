/*
 * Author: Natchanan Thongtem
 * Created on: 09/06/2019
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "main.hpp"

#include <vector>

void v_log_node_status( const ros::Publisher &kx_publisher, 
                        const std::string &kx_description_assigned );

void v_log_hardware_error(  const ros::Publisher &kx_publisher,
                            const int &ki_error_code_assigned, 
                            const int &ki_hardware_status_code_assigned, 
                            const std::string &kx_description_assigned );

void v_log_hardware_error_fatal(    const ros::Publisher &kx_publisher, 
                                    const int &ki_error_code_assigned, 
                                    const int &ki_hardware_status_code_assigned, 
                                    const std::string &kx_description_assigned );

void v_log_action(  const ros::Publisher &kx_publisher, 
                    const int &ki_action_assigned, 
                    const std::vector<int> &x_vector_value_assigned,
                    const std::string &kx_description_assigned );

#endif
