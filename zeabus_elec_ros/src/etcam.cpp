/*
 * Author: Natchanan Thongtem
 * Created on: 24/05/2019
 */

#include "etcam.hpp"

#include <cstdint>
#include <array>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "boost/array.hpp"
#include "zeabus/serial/synchronous_port.hpp"
#include "zeabus_utility/SendThrottle.h"
#include "zeabus_utility/ServiceGetTelemetry.h"
#include "zeabus_elec_ros/MessageNodeStatus.h"
#include "zeabus_elec_ros/MessageHardwareError.h"
#include "zeabus_elec_ros/MessageAction.h"
#include "zeabus_elec_ros/MessageTelemetryValue.h"
#include "libetcam.hpp"
#include "logger.hpp"

static void v_get_telemetry_value( void );
static bool b_set_thruster_throttle(    zeabus_utility::SendThrottle::Request &x_request,
                                        zeabus_utility::SendThrottle::Response &x_response);
static bool b_get_telemetry(    zeabus_utility::ServiceGetTelemetry::Request &x_request,
                                zeabus_utility::ServiceGetTelemetry::Response &x_response );

static std::shared_ptr< zeabus::serial::SynchronousPort > px_dev_etcam;

static ros::Publisher x_publisher_node_status_log;
static ros::Publisher x_publisher_hardware_error_log;
static ros::Publisher x_publisher_action_log;

static ros::ServiceServer x_service_server_set_thruster_throttle;
static ros::ServiceServer x_service_server_get_telemetry;
static ros::Publisher x_publisher_telemetry_value;

static zeabus_utility::ServiceGetTelemetry::Response x_telemetry_state;

static void v_get_telemetry_value( void )
{
    libetcam::TelemetrySync x_telemetry_sync;
    std::vector<uint8_t> x_sync( 1U, 0U );
    std::vector<uint8_t> x_telemetry( libetcam::ku_TELEMETRY_SIZE, 0U );

    // stamp telemetry receive time on telemetry_state
    x_telemetry_state.header.stamp = ros::Time::now();

    // sync with ETCAM telemetry
    do
    {
        // read data from ETCAM
        if( px_dev_etcam->read_data( &x_sync, 1U ) != 1U )
        {
            // unable to receive telemetry sync
            throw( ki_ERROR_UNABLE_TO_RECEIVE_ETCAM_TELEMETRY_SYNC );
        }
    }
    while( x_telemetry_sync.b_telemetry_sync( x_sync.front() ) == false );

    // read the telemetry from ETCAM
    if( px_dev_etcam->read_data( &x_telemetry, libetcam::ku_TELEMETRY_SIZE ) != libetcam::ku_TELEMETRY_SIZE )
    {
        // unable to receive telemetry
        throw( ki_ERROR_UNABLE_TO_RECEIVE_ETCAM_TELEMETRY );
    }

    // publish telemetry value log
    {
        boost::array<uint8_t, libetcam::ku_TELEMETRY_SIZE> au_boost_telemetry;
        zeabus_elec_ros::MessageTelemetryValue x_message_telemetry_value;

        // parse std::vector to boost::array
        std::copy( x_telemetry.begin(), x_telemetry.end(), au_boost_telemetry.begin() );

        // prepare telemetry value log
        x_message_telemetry_value.header.stamp = x_telemetry_state.header.stamp;
        x_message_telemetry_value.au_telemetry_value = au_boost_telemetry;

        // publish telemetry value log
        x_publisher_telemetry_value.publish( x_message_telemetry_value );
    }

    // parse telemetry and save telemetry state
    {
        std::array<uint8_t, libetcam::ku_TELEMETRY_SIZE> au_telemetry;
        std::array<libetcam::TelemetryStruct, libetcam::ku_THRUSTER_NUMBER> ax_parsed_telemetry;

        // parse std::vector to std::array
        std::copy( x_telemetry.begin(), x_telemetry.end(), au_telemetry.begin() );

        // parse KISS telemetry to human telemetry
        ax_parsed_telemetry = libetcam::ax_telemetry_parser( au_telemetry );

        // parse std::array to boost::array
        boost::array<zeabus_utility::StructTelemetry, libetcam::ku_THRUSTER_NUMBER> boost_parsed_telemetry;
        for( int i = 0U ; i < libetcam::ku_THRUSTER_NUMBER ; i++ )
        {
            boost_parsed_telemetry[i].temperature =         ax_parsed_telemetry[i].u_temperature;
            boost_parsed_telemetry[i].voltage =             ax_parsed_telemetry[i].f_voltage;
            boost_parsed_telemetry[i].current =             ax_parsed_telemetry[i].f_current;
            boost_parsed_telemetry[i].power_consumption =   ax_parsed_telemetry[i].us_power_consumption;
            boost_parsed_telemetry[i].erpm =                ax_parsed_telemetry[i].us_erpm;
        }

        // write telemetry to telemetry_state
        x_telemetry_state.telemetry = boost_parsed_telemetry;
    }

    return;
}

static bool b_set_thruster_throttle(    zeabus_utility::SendThrottle::Request &x_request,
                                        zeabus_utility::SendThrottle::Response &x_response)
{
    bool b_return;
    std::string x_description;

    b_return = true;

    // prepare the log description
    x_description = std::string( "Set thruster throttle requests was received" );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ki_ACTION_SET_THRUSTER_THROTTLE_CALLED,
                    0U,
                    0U,
                    x_description );

    // parse human throttle to DSHOT throttle
    boost::array<int16_t, libetcam::ku_THRUSTER_NUMBER> as_boost_thruster_throttle = x_request.data;

    // for each human throttle
    for( int i = 0U ; i < libetcam::ku_THRUSTER_NUMBER ; i++ )
    {
        if( as_boost_thruster_throttle[i] < 0 )
        {
            // reverse throttle
            as_boost_thruster_throttle[i] = ( as_boost_thruster_throttle[i] * ( -1 ) ) + 47U;
        }
        else if( as_boost_thruster_throttle[i] > 0 )
        {
            // forward throttle
            as_boost_thruster_throttle[i] += 1047U;
        }
    }

    // boost int16 array to uint16 std::array
    std::array<uint16_t, libetcam::ku_THRUSTER_NUMBER> au_thruster_throttle;
    std::copy( as_boost_thruster_throttle.begin(), as_boost_thruster_throttle.end(), au_thruster_throttle.begin() );

    // pack the throttle
    std::array<uint8_t, libetcam::ku_THROTTLE_PACKET_SIZE> au_packet_thruster_throttle = libetcam::au_throttle_pack( au_thruster_throttle );

    // std::array to std::vector
    std::vector<uint8_t> x_payload_thruster_throttle ( au_packet_thruster_throttle.begin(), au_packet_thruster_throttle.end() );

    // write the payload to ETCAM
    if( px_dev_etcam->write_data( &x_payload_thruster_throttle, libetcam::ku_THROTTLE_PACKET_SIZE ) != libetcam::ku_THROTTLE_PACKET_SIZE )
    {
        // unable to completely write the payload to ETCAM
        b_return = false;

        // prepare the log description
        x_description = std::string( "Unable to set ETCAM thruster throttle" );

        // print and publish the error log
        v_log_hardware_error(   x_publisher_hardware_error_log,
                                ki_ERROR_UNALBE_TO_SET_ETCAM_THRUSTER_THROTTLE,
                                -1,
                                x_description );
    }

    // prepare the log description
    x_description = std::string( "Set thruster throttle requests was served" );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ki_ACTION_SET_THRUSTER_THROTTLE_COMPLETE,
                    0U,
                    0U,
                    x_description );

    return b_return;
}

static bool b_get_telemetry(    zeabus_utility::ServiceGetTelemetry::Request &x_request,
                                zeabus_utility::ServiceGetTelemetry::Response &x_response )
{
    std::string x_description;

    // prepare the log description
    x_description = std::string( "get telemetry requests was received" );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ki_ACTION_GET_TELEMETRY_CALLED,
                    0U,
                    0U,
                    x_description );

    // response with telemetry state
    x_response = x_telemetry_state;

    // prepare the log description
    x_description = std::string( "Set thruster throttle requests was served" );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ki_ACTION_GET_TELEMETRY_COMPLETE,
                    0U,
                    0U,
                    x_description );


    return true;
}

int main( int argc, char **argv )
{
    static int i_main_status = 0U;
    static std::string x_initial_device_path;

    i_main_status = ki_ERROR_NONE;

    // init ROS node
    ros::init( argc, argv, "zeabus_elec_etcam" );
    ros::NodeHandle x_node_handle( "/elec" );

    // retrive parameters from the launch file
    x_node_handle.param<std::string>( "/zeabus_elec_etcam/x_initial_device_path", x_initial_device_path, "/dev/ttyUSB0" );

    // register log publisher to ROS
    x_publisher_node_status_log =                   x_node_handle.advertise<zeabus_elec_ros::MessageNodeStatus>( "node_status", 100U );
    x_publisher_hardware_error_log =                x_node_handle.advertise<zeabus_elec_ros::MessageHardwareError>( "hardware_error", 100U );
    x_publisher_action_log =                        x_node_handle.advertise<zeabus_elec_ros::MessageAction>( "action", 100U );

    do
    {
        // wait for ROS initiation
        ros::Duration( 0.5 ).sleep();
    }while( !ros::ok() );

    v_log_node_status( x_publisher_node_status_log, std::string( "ETCAM node started" ) );

    // create the device manager object
    px_dev_etcam = std::make_shared<zeabus::serial::SynchronousPort>( x_initial_device_path );

    try
    {
        // open the device
        if( px_dev_etcam->open_port() != true )
        {
            // unable to open ETCAM
            throw( ki_ERROR_UNABLE_TO_OPEN_ETCAM );
        }

        // set settings of the object
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::baud_rate( 115200U ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::character_size( 8U ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::flow_control( boost::asio::serial_port_base::flow_control::none ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one ) );


        // register publisher to ROS
        x_publisher_telemetry_value = x_node_handle.advertise<zeabus_elec_ros::MessageTelemetryValue>( "telemetry_value", 100U );

        // register service server to ROS
        x_service_server_set_thruster_throttle =    x_node_handle.advertiseService( "/hardware/thruster_throttle", b_set_thruster_throttle );
        x_service_server_get_telemetry =            x_node_handle.advertiseService( "/hardware/thruster_feedback", b_get_telemetry );

        // define frequency of this node
        ros::Rate x_rate( 100U );

        while( ros::ok() )
        {
            try
            {
                v_get_telemetry_value();
            }
            catch( const int &ki_error )
            {
                std::string x_description;

                // prepare the log description
                switch( ki_error )
                {
                    case ki_ERROR_UNABLE_TO_RECEIVE_ETCAM_TELEMETRY_SYNC:
                        x_description = std::string( "Unable to receive ETCAM telemetry sync" );
                        break;
                    case ki_ERROR_UNABLE_TO_RECEIVE_ETCAM_TELEMETRY:
                        x_description = std::string( "Unable to receive ETCAM telemetry" );
                        break;
                    default:
                        x_description = std::string( "Unknown error" );
                }

                // print and publish the log
                v_log_hardware_error(  x_publisher_hardware_error_log,
                                        ki_error,
                                        -1,
                                        x_description );
            }

            // look for callback
            ros::spinOnce();

            // wait for next iteration
            x_rate.sleep();
        }
    }
    catch( const int &ki_error )
    {
        // catch the fatal error
        std::string x_description;

        i_main_status = ki_error;

        // prepare the log description
        switch( ki_error )
        {
            case ki_ERROR_UNABLE_TO_OPEN_ETCAM:
                x_description = std::string( "Unable to open ETCAM" );
                break;
            default:
                x_description = std::string( "Unknown error" );
        }

        // print and publish the log
        v_log_hardware_error_fatal( x_publisher_hardware_error_log,
                                    ki_error,
                                    -1,
                                    x_description );
    }

    v_log_node_status( x_publisher_node_status_log, std::string( "ETCAM node is shutting down" ) );

    // normally, the program should never reach this point
    return i_main_status;
}
