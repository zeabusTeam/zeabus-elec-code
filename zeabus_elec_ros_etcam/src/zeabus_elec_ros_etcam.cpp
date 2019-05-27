/*
 * Author: Natchanan Thongtem
 * Created on: 25/05/2019
 */

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "boost/array.hpp"
#include "ros/ros.h"
#include "zeabus/serial/synchronous_port.hpp"
#include "zeabus_elec_ros_etcam/CommandSetThrusterThrottle.h"
#include "zeabus_elec_ros_etcam/Telemetry.h"

#include "libetcam.hpp"

static const int32_t kl_UNABLE_TO_OPEN_DEVICE = -1;
static const int32_t kl_UNABLE_TO_RECEIVE_TELEMETRY_SYNC = -2;
static const int32_t kl_UNABLE_TO_RECEIVE_TELEMETRY = -3;

static std::shared_ptr< zeabus::serial::SynchronousPort > px_dev_etcam;
static ros::ServiceServer x_service_server_set_thruster_throttle;
static ros::Publisher x_publisher_telemetry;

static bool b_set_thruster_throttle( zeabus_elec_ros_etcam::CommandSetThrusterThrottle::Request &x_request,
                                    zeabus_elec_ros_etcam::CommandSetThrusterThrottle::Response &x_response)
{

    x_response.b_result = false;

    boost::array< uint16_t, libetcam::ku_THRUSTER_NUMBER > au_boost_thruster_throttle;
    std::array< uint16_t, libetcam::ku_THRUSTER_NUMBER > au_thruster_throttle;

    au_boost_thruster_throttle = x_request.aus_thruster_throttle;
    std::memcpy( au_thruster_throttle.begin(), au_boost_thruster_throttle.begin(), libetcam::ku_THRUSTER_NUMBER );

    std::array< uint8_t, libetcam::ku_THROTTLE_PACKET_SIZE > au_packet_thruster_throttle = libetcam::au_throttle_pack( au_thruster_throttle );

    std::vector< uint8_t > x_payload_thruster_throttle ( au_packet_thruster_throttle.begin(), au_packet_thruster_throttle.end() );

    if( px_dev_etcam->write_data( &x_payload_thruster_throttle, libetcam::ku_THROTTLE_PACKET_SIZE ) == libetcam::ku_THROTTLE_PACKET_SIZE )
    {
        x_response.b_result = true;
    }

    return x_response.b_result;
}

static void v_get_telemetry( void )
{
    libetcam::TelemetrySync x_telemetry_sync;
    std::vector< uint8_t > x_sync( 1U, 0U );
    std::vector< uint8_t > x_telemetry( libetcam::ku_TELEMETRY_SIZE, 0U );
    zeabus_elec_ros_etcam::Telemetry x_telemetry_message;
    boost::array< uint8_t, libetcam::ku_TELEMETRY_SIZE > au_boost_telemetry;

    do
    {
        if( px_dev_etcam->read_data( &x_sync, 1U ) != 1U )
        {
            throw( kl_UNABLE_TO_RECEIVE_TELEMETRY_SYNC );
        }
    }
    while( x_telemetry_sync.b_telemetry_sync( x_sync.front() ) == false );

    if( px_dev_etcam->read_data( &x_telemetry, libetcam::ku_TELEMETRY_SIZE ) != libetcam::ku_TELEMETRY_SIZE )
    {
        throw( kl_UNABLE_TO_RECEIVE_TELEMETRY );
    }

    std::copy( x_telemetry.begin(), x_telemetry.end(), au_boost_telemetry.begin() );

    x_telemetry_message.au_telemetry = au_boost_telemetry;

    x_publisher_telemetry.publish( x_telemetry_message );

    return;
}

int main( int argc, char **argv )
{
    static int32_t l_main_status = 0U;

    ros::init( argc, argv, "Zeabus_Elec_ETCAM" );
    ros::NodeHandle x_node_handle( "/zeabus/elec" );

    px_dev_etcam = std::make_shared< zeabus::serial::SynchronousPort >( "/dev/ttyUSB0" );

    ros::Rate r( 100 );
    try
    {
        if( px_dev_etcam->open_port() != true )
        {
            throw( kl_UNABLE_TO_OPEN_DEVICE );
        }
        
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::baud_rate( 115200U ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::character_size( 8U ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::flow_control( boost::asio::serial_port_base::flow_control::none ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one ) );

        x_service_server_set_thruster_throttle = x_node_handle.advertiseService( "set_thruster_throttle", b_set_thruster_throttle );
        x_publisher_telemetry = x_node_handle.advertise< zeabus_elec_ros_etcam::Telemetry >( "telemetry", 100U );

        while( ros::ok() )
        {
            try
            {
                v_get_telemetry();
            }
            catch( const int32_t &kl_ERROR )
            {
                switch( kl_ERROR )
                {
                    case kl_UNABLE_TO_RECEIVE_TELEMETRY_SYNC:
                        ROS_ERROR( "Unable to sync telemtry of ETCAM" );
                        break;
                    case kl_UNABLE_TO_RECEIVE_TELEMETRY:
                        ROS_ERROR( "Unable to receive telemtry of ETCAM" );
                        break;
                    default:
                        ROS_ERROR( "Unknown Error" );
                }
                ROS_ERROR( "Error code: %d", kl_ERROR );
            }

            ros::spinOnce();

            r.sleep();
        }

    }
    catch( const int32_t &kl_ERROR )
    {
        l_main_status = kl_ERROR;

        switch( kl_ERROR )
        {
            case kl_UNABLE_TO_OPEN_DEVICE:
                ROS_FATAL( "Unable to open ETCAM" );
                break;
            default:
                ROS_FATAL( "Unknown Error" );
        }

        ROS_FATAL( "Error code: %d", kl_ERROR );
    }

    return l_main_status;
}
