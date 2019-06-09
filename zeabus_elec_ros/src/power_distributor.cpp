/*
 * Author: Natchanan Thongtem
 * Created on: 08/06/2019
 */

#include <cstdint>
#include <memory>
#include <string>

#include "ros/ros.h"
#include "ftdi_impl.h"
#include "zeabus_elec_ros/MessageHardwareError.h"
#include "zeabus_elec_ros/ServicePowerSwitch.h"

static const int ki_ERROR_NONE =                    0U;
static const int ki_ERROR_UNABLE_TO_OPEN_DEVICE =   1U;
static const int ki_ERROR_UNABLE_TO_INIT_GPIO =     2U;
static const int ki_ERROR_UNABLE_TO_SET_GPIO =      10U;

static const std::string kx_POWER_DISTRIBUTOR_DESCRIPTION = "PowerDist";

static const uint16_t kus_INITIAL_IO_DIRECTION =    0xFFFFU;
static const uint16_t kus_INITIAL_IO_PIN_STATE =    0x0000U;

static std::shared_ptr<Zeabus_Elec::ftdi_mpsse_impl> px_power_dist;

static ros::Publisher x_publisher_error_message;

static ros::ServiceServer x_service_server_power_switch;

bool b_set_swtich(  zeabus_elec_ros::ServicePowerSwitch::Request &x_request,
                    zeabus_elec_ros::ServicePowerSwitch::Response &x_response )
{
    bool b_result = true;
    int i_status_power_dist;
    uint8_t u_switch_state, u_swtich_state_current, u_switch_mask;
    
    // mark requested GPIO pin
    u_switch_mask = 1U << ( x_request.u_switch_index );

    // get current GPIO pin state
    u_swtich_state_current = px_power_dist->ReadHiGPIOData();

    if( x_request.is_switch_high )
    {
        // make requested GPIO pin state HI
        u_switch_state = ( u_swtich_state_current | u_switch_mask );
    }
    else
    {
        // make requested GPIO pin state LO
        u_switch_state = ( u_swtich_state_current & ~( u_switch_mask ) );
    }
    
    // set new GPIO pin state
    i_status_power_dist = px_power_dist->SetHiGPIOData( u_switch_state );

    if( i_status_power_dist != 0U )
    {
        // unable to set GPIO pin state of power distributor
        zeabus_elec_ros::MessageHardwareError x_error_message;

        b_result = false;

        // prepare error message
        x_error_message.header.stamp = ros::Time::now();

        x_error_message.message = std::string( "error code ");
        x_error_message.message += std::to_string( ki_ERROR_UNABLE_TO_SET_GPIO );
        x_error_message.message += std::string( " : " );
        x_error_message.message += std::string( "power distributor status code ");
        x_error_message.message += std::to_string( i_status_power_dist );
        x_error_message.message += std::string( " : " );
        x_error_message.message += std::string( "unable to set GPIO pin state of power distributor" );

        // publish error message
        x_publisher_error_message.publish( x_error_message );

        // print error message to ROS console
        ROS_ERROR( x_error_message.message.c_str() );
    }

    return b_result;
}

int main( int argc, char **argv )
{
    static int i_main_status, i_param_initial_io_direction, i_param_initial_io_pin_state;
    static uint16_t us_initial_io_direction, us_initial_io_pin_state;

    i_main_status = ki_ERROR_NONE;

    // init ROS node
    ros::init( argc, argv, "zeabus_elec_power_dist" );
    ros::NodeHandle node_handle( "/elec" );

    // retrive parameters from the launch file
    node_handle.param<int>("/zeabus_elec_power_dist/io_direction", i_param_initial_io_direction, kus_INITIAL_IO_DIRECTION);
    node_handle.param<int>("/zeabus_elec_power_dist/io_pin_state", i_param_initial_io_pin_state, kus_INITIAL_IO_PIN_STATE);

    // cast int to uint16_t because NodeHandle::param doesn't support uint16_t
    us_initial_io_direction = uint16_t( i_param_initial_io_direction );
    us_initial_io_pin_state = uint16_t( i_param_initial_io_pin_state );

    // register hardware error publisher to ROS
    x_publisher_error_message = node_handle.advertise<zeabus_elec_ros::MessageHardwareError>( "hardware_error", 1000U );

    // wait for ROS initiation
    ros::Duration( 0.1 ).sleep();

    // create the device manager object, also open the device
    px_power_dist = std::make_shared<Zeabus_Elec::ftdi_mpsse_impl>( Zeabus_Elec::FT232H, kx_POWER_DISTRIBUTOR_DESCRIPTION );
    
    try
    {
        // check the device manager object status after created
        if( px_power_dist->GetCurrentStatus() != 0U )
        {
            // unable to init power distributor
            throw( ki_ERROR_UNABLE_TO_OPEN_DEVICE );
        }
        
        // set initialize GPIO direction and GPIO pin state to all output
        px_power_dist->SetGPIODirection( us_initial_io_direction , us_initial_io_pin_state );
        if( px_power_dist->GetCurrentStatus() != 0U )
        {
            // unable to initialize GPIO direction and GPIO pin state of power distributor
            throw( ki_ERROR_UNABLE_TO_INIT_GPIO );
        }

        // register service server to ROS
        x_service_server_power_switch = node_handle.advertiseService( "power_switch", b_set_swtich );

        // look for callbacks forever
        ros::spin();
    }
    catch( const int &ki_ERROR )
    {
        // catch the fatal error
        zeabus_elec_ros::MessageHardwareError x_error_message;

        i_main_status = ki_ERROR;

        // prepare error message
        x_error_message.header.stamp = ros::Time::now();

        x_error_message.message = std::string( "error code ");
        x_error_message.message += std::to_string( ki_ERROR );
        x_error_message.message += std::string( " : " );
        x_error_message.message += std::string( "power distributor status code " );
        x_error_message.message += std::to_string( px_power_dist->GetCurrentStatus() );
        x_error_message.message += std::string( " : " );

        switch( ki_ERROR )
        {
            case ki_ERROR_UNABLE_TO_OPEN_DEVICE:
                x_error_message.message += std::string( "unable to init power distributor" );
                break;
            case ki_ERROR_UNABLE_TO_INIT_GPIO:
                x_error_message.message += std::string( "unable to initialize GPIO direction and GPIO pin state of power distributor" );
                break;
            default:
                x_error_message.message += std::string( "unknown error" );
        }

        // publish error message
        x_publisher_error_message.publish( x_error_message );

        // print error message to ROS console
        ROS_FATAL( x_error_message.message.c_str() );
    }

    // the program should never reach this point
    return i_main_status;
}
