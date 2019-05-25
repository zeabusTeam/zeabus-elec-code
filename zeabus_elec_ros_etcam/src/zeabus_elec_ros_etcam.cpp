/*
 * Author: Natchanan Thongtem
 * Created on: 25/05/2019
 */

#include <array>
#include <string>
#include <vector>
#include <memory>

#include "boost/array.hpp"
#include "ros/ros.h"
#include "zeabus/serial/synchronous_port.hpp"
#include "zeabus_elec_ros_etcam/CommandSetThrusterThrottle.h"

#include "libetcam.hpp"

static std::shared_ptr< zeabus::serial::SynchronousPort > px_dev_etcam;

static bool set_thruster_throttle( zeabus_elec_ros_etcam::CommandSetThrusterThrottle::Request &kx_REQUEST,
                                zeabus_elec_ros_etcam::CommandSetThrusterThrottle::Response &kx_RESPONSE)
{

    kx_RESPONSE.b_result = false;

    boost::array< uint16_t, libetcam::ku_THRUSTER_NUMBER > kau_BOOST_THRUSTER_THROTTLE;
    std::array< uint16_t, libetcam::ku_THRUSTER_NUMBER > kau_thruster_throttle;

    kau_BOOST_THRUSTER_THROTTLE = kx_REQUEST.aus_thruster_throttle;
    std::memcpy( kau_thruster_throttle.begin(), kau_BOOST_THRUSTER_THROTTLE.begin(), libetcam::ku_THRUSTER_NUMBER );

    std::array< uint8_t, libetcam::ku_THROTTLE_PACKET_SIZE > kau_PACKET_THRUSTER_THROTTLE = libetcam::au_throttle_pack( kau_thruster_throttle );

    std::vector< uint8_t > kx_PAYLOAD_THRUSTER_THROTTLE ( kau_PACKET_THRUSTER_THROTTLE.begin(), kau_PACKET_THRUSTER_THROTTLE.end() );

    if( px_dev_etcam->write_data( &kx_PAYLOAD_THRUSTER_THROTTLE, libetcam::ku_THROTTLE_PACKET_SIZE ) == libetcam::ku_THROTTLE_PACKET_SIZE )
    {
        kx_RESPONSE.b_result = true;
    }

    return kx_RESPONSE.b_result;
}

int main( int argc, char **argv )
{
    static const int32_t kl_UNABLE_TO_OPEN_DEVICE = -1;

    static ros::ServiceServer x_service_server_set_thruster_throttle;

    static int32_t l_main_status = 0U;

    ros::init( argc, argv, "Zeabus_Elec_ETCAM" );
    ros::NodeHandle x_node_handle( "/zeabus/elec" );

    px_dev_etcam = std::make_shared< zeabus::serial::SynchronousPort >( "/dev/ttyUSB0" );

    try
    {
        if( px_dev_etcam->open_port() != true )
        {
            return kl_UNABLE_TO_OPEN_DEVICE;
        }
        
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::baud_rate( 115200U ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::character_size( 8U ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::flow_control( boost::asio::serial_port_base::flow_control::none ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none ) );
        px_dev_etcam->set_option_port( boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one ) );

        x_service_server_set_thruster_throttle = x_node_handle.advertiseService( "/etcam/set_thruster_throttle", set_thruster_throttle );

        ros::spin();
    }
    catch( const int32_t &kl_ERROR )
    {
        l_main_status = kl_ERROR;

        switch( kl_ERROR )
        {
            case kl_UNABLE_TO_OPEN_DEVICE:
                ROS_FATAL( "unable to open ETCAM" );
                break;
            default:
                ROS_FATAL( "Unknown Error" );
        }

        ROS_FATAL( " Error code: %d", kl_ERROR );
    }

    return l_main_status;
}
