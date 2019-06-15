/*
 * Author: Natchanan Thongtem
 * Created on: 08/06/2019
 */

#include "power_distributor.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include "zeabus_elec_ros/MessageNodeStatus.h"
#include "zeabus_elec_ros/MessageHardwareError.h"
#include "zeabus_elec_ros/MessageAction.h"
#include "zeabus_elec_ros/ServicePowerSwitch.h"
#include "ftdi_impl.h"
#include "logger.hpp"

extern void v_log_node_status(  const ros::Publisher &x_publisher, 
                                const std::string &x_description_assigned );

extern void v_log_hardware_error(   const ros::Publisher &x_publisher,
                                    const int64_t &l_error_code_assigned, 
                                    const int64_t &l_hardware_status_code_assigned, 
                                    const std::string &x_description_assigned );

extern void v_log_hardware_error_fatal( const ros::Publisher &x_publisher, 
                                        const int64_t &l_error_code_assigned, 
                                        const int64_t &l_hardware_status_code_assigned, 
                                        const std::string &x_description_assigned );

extern void v_log_action(   const ros::Publisher &x_publisher, 
                            const int64_t &l_action_assigned, 
                            const int64_t &l_value1_assigned, 
                            const int64_t &l_value2_assigned, 
                            const std::string &x_description_assigned );

static const int ki_ERROR_NONE =                    0U;
static const int ki_ERROR_UNABLE_TO_OPEN_DEVICE =   1U;
static const int ki_ERROR_UNABLE_TO_INIT_GPIO =     2U;
static const int ki_ERROR_UNABLE_TO_SET_GPIO =      10U;

static const int ki_ACTION_SET_SWTICH_CALLED =      15U;
static const int ki_ACTION_SET_SWTICH_COMPLETE =    16U;

static const std::string kx_POWER_DISTRIBUTOR_DESCRIPTION = "PowerDist";

static const uint16_t kus_INITIAL_IO_DIRECTION =    0xFFFFU;
static const uint16_t kus_INITIAL_IO_PIN_STATE =    0x0000U;

static std::shared_ptr<Zeabus_Elec::ftdi_mpsse_impl> px_power_dist;

static ros::Publisher x_publisher_node_status_message;
static ros::Publisher x_publisher_hardware_error_message;
static ros::Publisher x_publisher_action_message;

static ros::ServiceServer x_service_server_power_switch;

static bool b_set_swtich(   zeabus_elec_ros::ServicePowerSwitch::Request &x_request,
                            zeabus_elec_ros::ServicePowerSwitch::Response &x_response )
{
    bool b_result = true;
    int i_status_power_dist;
    uint8_t u_switch_state, u_swtich_state_current, u_switch_mask;
    std::string x_description;

    // prepare the log description message
    x_description = std::string( "Set switch service requests, swtich index " );
    x_description += std::to_string( x_request.u_switch_index );
    x_description += std::string( " switch state " );
    x_description += std::to_string( x_request.is_switch_high );
    x_description += std::string( " was received" );
    
    // print and publish, service requests was received, the log
    v_log_action(   x_publisher_action_message,
                    ( int64_t )ki_ACTION_SET_SWTICH_CALLED,
                    ( int64_t )x_request.u_switch_index,
                    ( int64_t )x_request.is_switch_high,
                    x_description );

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
        b_result = false;

        // print and publish, unable to set GPIO pin state of power distributor , the log
        v_log_hardware_error(   x_publisher_hardware_error_message,
                                ( int64_t )ki_ERROR_UNABLE_TO_SET_GPIO,
                                ( int64_t )i_status_power_dist,
                                std::string( "Unable to set GPIO pin state of power distributor" ) );
    }

    // prepare, service requests was served, the log
    x_description = std::string( "Set switch service requests, swtich index " );
    x_description += std::to_string( x_request.u_switch_index );
    x_description += std::string( " switch state " );
    x_description += std::to_string( x_request.is_switch_high );
    x_description += std::string( " was served" );

    // print and publish, service requests was servede, the log
    v_log_action(   x_publisher_action_message,
                    ( int64_t )ki_ACTION_SET_SWTICH_COMPLETE,
                    ( int64_t )x_request.u_switch_index,
                    ( int64_t )x_request.is_switch_high,
                    x_description );

    return b_result;
}

int main( int argc, char **argv )
{
    static int i_main_status, i_param_initial_io_direction, i_param_initial_io_pin_state;
    static uint16_t us_initial_io_direction, us_initial_io_pin_state;

    i_main_status = ki_ERROR_NONE;

    // init ROS node
    ros::init( argc, argv, "zeabus_elec_power_dist" );
    ros::NodeHandle x_node_handle( "/elec" );

    // retrive parameters from the launch file
    x_node_handle.param<int>("/zeabus_elec_power_dist/i_initial_io_direction", i_param_initial_io_direction, kus_INITIAL_IO_DIRECTION);
    x_node_handle.param<int>("/zeabus_elec_power_dist/i_initial_io_pin_state", i_param_initial_io_pin_state, kus_INITIAL_IO_PIN_STATE);

    // cast int to uint16_t because NodeHandle::param doesn't support uint16_t
    us_initial_io_direction = uint16_t( i_param_initial_io_direction );
    us_initial_io_pin_state = uint16_t( i_param_initial_io_pin_state );

    // register hardware error publisher to ROS
    x_publisher_node_status_message =       x_node_handle.advertise<zeabus_elec_ros::MessageNodeStatus>( "node_status", 100U );
    x_publisher_hardware_error_message =    x_node_handle.advertise<zeabus_elec_ros::MessageHardwareError>( "hardware_error", 100U );
    x_publisher_action_message =            x_node_handle.advertise<zeabus_elec_ros::MessageAction>( "action", 100U );

    do
    {
        // wait for ROS initiation
        ros::Duration( 0.5 ).sleep();
    }while( !ros::ok() );

    v_log_node_status( x_publisher_node_status_message, std::string( "Power distributor node started" ) );

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
        x_service_server_power_switch = x_node_handle.advertiseService( "power_switch", b_set_swtich );

        // look for callbacks forever
        ros::spin();
    }
    catch( const int &ki_ERROR )
    {
        // catch the fatal error
        std::string x_description;

        i_main_status = ki_ERROR;

        // prepare the log description message
        switch( ki_ERROR )
        {
            case ki_ERROR_UNABLE_TO_OPEN_DEVICE:
                x_description = std::string( "Unable to init power distributor" );
                break;
            case ki_ERROR_UNABLE_TO_INIT_GPIO:
                x_description = std::string( "Unable to initialize GPIO direction and GPIO pin state of power distributor" );
                break;
            default:
                x_description = std::string( "Unknown error" );
        }

        // print and publish the log
        v_log_hardware_error_fatal( x_publisher_hardware_error_message,
                                    ( int64_t )ki_ERROR,
                                    ( int64_t )px_power_dist->GetCurrentStatus(),
                                    x_description );
    }

    v_log_node_status( x_publisher_node_status_message, std::string( "Power distribution node is shutting down" ) );

    // normally, the program should never reach this point
    return i_main_status;
}
