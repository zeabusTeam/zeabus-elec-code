/*
 * Author: Natchanan Thongtem
 * Created on: 17/06/2019
 */

#include "peripheral_bridge.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "zeabus_utility/ServiceDepth.h"
#include "zeabus_elec_ros/MessageNodeStatus.h"
#include "zeabus_elec_ros/MessageHardwareError.h"
#include "zeabus_elec_ros/MessageAction.h"
#include "zeabus_elec_ros/MessageBarometerValue.h"
#include "zeabus_elec_ros/ServiceIOPinState.h"
#include "ftdi_impl.h"
#include "logger.hpp"

static const std::string kx_PERIPHERAL_BRIDGE_DESCRIPTION = "PeripheralBridge";
static const double klf_ONE_ATM_AS_PSI =                    14.6959;
static const double klf_PSI_PER_DEPTH =                     0.6859;

static void v_get_barometer_value( void );
static bool b_service_get_depth(    zeabus_utility::ServiceDepth::Request &x_request,
                                    zeabus_utility::ServiceDepth::Response &x_response );
static bool b_set_io_pin_state( zeabus_elec_ros::ServiceIOPinState::Request &x_request,
                                zeabus_elec_ros::ServiceIOPinState::Response &x_response );
static double lf_barometer_value_to_depth( const uint16_t &us_barometer_value );

static std::shared_ptr<Zeabus_Elec::ftdi_impl> px_peripheral_bridge_a, px_peripheral_bridge_b;

static ros::Publisher x_publisher_node_status_log;
static ros::Publisher x_publisher_hardware_error_log;
static ros::Publisher x_publisher_action_log;

static ros::Publisher x_publisher_barometer_value;
static ros::ServiceServer x_service_server_get_depth;
static ros::ServiceServer x_service_server_set_io_pin_state;

static zeabus_utility::ServiceDepth::Response x_depth_state;
static double lf_atm_pressure, lf_depth_offset;

static void v_get_barometer_value( void )
{
    std::vector<uint8_t> x_buffer( 2U, 0U );
    int i_completed_byte;
    uint16_t us_barometer_value;
    double lf_depth;
    zeabus_elec_ros::MessageBarometerValue x_message_barometer_value;

    // stamp barometer sample time on depth_state
    x_depth_state.header.stamp = ros::Time::now();

    // to read barometer value we need to send 2 bytes of dummy data
    i_completed_byte = px_peripheral_bridge_a->Send( x_buffer );

    if( px_peripheral_bridge_a->GetCurrentStatus() != 0U || i_completed_byte != 11U )
    {
        // unable to request barometer value from peripheral bridge
        throw( ki_ERROR_UNABLE_TO_REQURST_BAROMETER_VALUE );
    }

    // read the barometer value. the resolution of the barometer is 10 bits and arrange as
    // 0 0 0 D D D D D  D D D D D x x x
    // where the first byte is the MSB
    i_completed_byte = px_peripheral_bridge_a->Receive( x_buffer );
    if( px_peripheral_bridge_a->GetCurrentStatus() != 0U || i_completed_byte != 2U )
    {
        // unable to receive barometer value from peripheral bridge
        throw( ki_ERROR_UNABLE_TO_RECEIVE_BAROMETER_VALUE );
    }

    // parse the barometer value from buffer
    us_barometer_value = ( uint16_t )x_buffer[0U];
    us_barometer_value <<= 8U;
    us_barometer_value |= ( uint16_t )x_buffer[1U];
    us_barometer_value >>= 3U;
    
    // shift right by 1 to compensate the invalid first clock from command 0x31
    us_barometer_value >>= 1U;

    // prepare barometer value log 
    x_message_barometer_value.header.stamp = ros::Time::now();
    x_message_barometer_value.us_barometer_value = us_barometer_value;

    // publish barometer value log
    x_publisher_barometer_value.publish( x_message_barometer_value );

    // calculate depth from barometer value
    lf_depth = lf_barometer_value_to_depth( us_barometer_value );

    // write depth into depth_state
    x_depth_state.depth = lf_depth;

    return;
}

static bool b_service_get_depth(    zeabus_utility::ServiceDepth::Request &x_request,
                                    zeabus_utility::ServiceDepth::Response &x_response )
{
    std::string x_description;
    
    // prepare the log description
    x_description = std::string( "Get depth service requests was received" );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ( int64_t )ki_ACTION_GET_DEPTH_CALLED,
                    ( int64_t )0U,
                    ( int64_t )0U,
                    x_description );

    // response with depth_state
    x_response = x_depth_state;

    // prepare the log description
    x_description = std::string( "Get depth service requests was served, depth " );
    x_description += std::to_string( x_depth_state.depth );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ( int64_t )ki_ACTION_GET_DEPTH_COMPLETE,
                    ( int64_t )( x_depth_state.depth * 100U ),
                    ( int64_t )0U,
                    x_description );

    return true;
}

static bool b_set_io_pin_state( zeabus_elec_ros::ServiceIOPinState::Request &x_request,
                                zeabus_elec_ros::ServiceIOPinState::Response &x_response )
{
    uint8_t u_io_pin_state_nibble, u_io_pin_state, u_io_pin_state_current, u_io_pin_state_mask;
    int i_state_peripheral_bridge;
    bool b_return;
    std::string x_description;

    b_return = true;

    // prepare the log description
    x_description = std::string( "Set IO pin state service requests, IO pin index " );
    x_description += std::to_string( x_request.u_io_pin_index );
    x_description += std::string( " IO pin state " );
    x_description += std::to_string( x_request.is_io_pin_state_high );
    x_description += std::string( " was received" );

    // print and publish the log
    v_log_action(   x_publisher_action_log,
                    ( int64_t )ki_ACTION_SET_IO_PIN_STATE_CALLED,
                    ( int64_t )x_request.u_io_pin_index,
                    ( int64_t )x_request.is_io_pin_state_high,
                    x_description );

    try
    {
        // mark the IO pin
        u_io_pin_state_mask = 0x01U << ( x_request.u_io_pin_index );

        // get current GPIO pin state from peripheral bridge
        u_io_pin_state_current = px_peripheral_bridge_a->ReadLoGPIOData() >> 4U;
        u_io_pin_state_current |= px_peripheral_bridge_b->ReadLoGPIOData() & 0xF0U;

        if( px_peripheral_bridge_a->GetCurrentStatus() != 0U )
        {
            // unable to get peripheral bridge A current GPIO pin state
            throw( ki_ERROR_UNABLE_TO_GET_PERIPHERAL_BRIDGE_A_CURRENT_GPIO_PIN_STATE );
        }


        if( px_peripheral_bridge_b->GetCurrentStatus() != 0U )
        {
            // unable to get peripheral bridge B current GPIO pin state
            throw( ki_ERROR_UNABLE_TO_GET_PERIPHERAL_BRIDGE_B_CURRENT_GPIO_PIN_STATE );
        }

        // determine the new GPIO pin state
        if( x_request.is_io_pin_state_high )
        {
            u_io_pin_state = ( u_io_pin_state_current | u_io_pin_state_mask );
        }
        else
        {
            u_io_pin_state = ( u_io_pin_state_current & ~( u_io_pin_state_mask ) );
        }

        // Set LO GPIO pin state of peripheral bridge
        u_io_pin_state_nibble = ( u_io_pin_state << 4U );
        i_state_peripheral_bridge = px_peripheral_bridge_a->SetLoGPIOData( u_io_pin_state_nibble );

        if( i_state_peripheral_bridge != 0U )
        {
            // unable to set peripheral bridge A GPIO
            throw( ki_ERROR_UNABLE_TO_SET_PERIPHERAL_BRIDGE_A_GPIO_PIN_STATE );
        }

        // Set HI GPIO pin state of peripheral bridge
        u_io_pin_state_nibble = ( u_io_pin_state & 0xF0U );
        i_state_peripheral_bridge = px_peripheral_bridge_b->SetLoGPIOData( u_io_pin_state_nibble );

        if( i_state_peripheral_bridge != 0U )
        {
            // unable to set peripheral bridge B GPIO
            throw( ki_ERROR_UNABLE_TO_SET_PERIPHERAL_BRIDGE_B_GPIO_PIN_STATE );
        }

        // prepare the log description
        x_description = std::string( "Set IO pin state service requests, IO pin index " );
        x_description += std::to_string( x_request.u_io_pin_index );
        x_description += std::string( " IO pin state " );
        x_description += std::to_string( x_request.is_io_pin_state_high );
        x_description += std::string( " was served" );

        // print and publish the log
        v_log_action(   x_publisher_action_log,
                        ( int64_t )ki_ACTION_SET_IO_PIN_STATE_COMPLETE,
                        ( int64_t )x_request.u_io_pin_index,
                        ( int64_t )x_request.is_io_pin_state_high,
                        x_description );
    }
    catch( const int &ki_error )
    {
        std::string x_description;
        int i_hardware_error_code;

        b_return = false;

        // prepare the log
        switch( ki_error )
        {
            case ki_ERROR_UNABLE_TO_GET_PERIPHERAL_BRIDGE_A_CURRENT_GPIO_PIN_STATE:
                i_hardware_error_code = px_peripheral_bridge_a->GetCurrentStatus();
                x_description = std::string( "Unable to get peripheral bridge A current GPIO pin state" );
                break;
            case ki_ERROR_UNABLE_TO_GET_PERIPHERAL_BRIDGE_B_CURRENT_GPIO_PIN_STATE:
                i_hardware_error_code = px_peripheral_bridge_b->GetCurrentStatus();
                x_description = std::string( "Unable to get peripheral bridge B current GPIO pin state" );
                break;
            case ki_ERROR_UNABLE_TO_SET_PERIPHERAL_BRIDGE_A_GPIO_PIN_STATE:
                i_hardware_error_code = px_peripheral_bridge_a->GetCurrentStatus();
                x_description = std::string( "Unable to set peripheral bridge A GPIO pin state" );
                break;
            case ki_ERROR_UNABLE_TO_SET_PERIPHERAL_BRIDGE_B_GPIO_PIN_STATE:
                i_hardware_error_code = px_peripheral_bridge_b->GetCurrentStatus();
                x_description = std::string( "Unable to set peripheral bridge B GPIO pin state" );
                break;
            default:
                i_hardware_error_code = -1;
                x_description = std::string( "Unknown error" );
        }

        // print and publish the log
        v_log_hardware_error(   x_publisher_hardware_error_log,
                                ( int64_t ) ki_error,
                                ( int64_t ) i_hardware_error_code,
                                x_description );
    }

    return b_return;
}

static double lf_barometer_value_to_depth( const uint16_t &us_barometer_value )
{
    double lf_depth, lf_barometer_voltage, lf_psi;

    // calculate barometer voltage from barometer value
    lf_barometer_voltage = us_barometer_value * ( 5.0 / 1023.0 );
    // calculcate pressure from baromter voltage
    lf_psi = ( lf_barometer_voltage - 0.5 ) * ( 30.0 / 4.0 );

    // calculate depth from pressure
    lf_depth = ( ( lf_psi - lf_atm_pressure ) * klf_PSI_PER_DEPTH ) + lf_depth_offset;
    
    return lf_depth;
}

int main( int argc, char** argv )
{
    static int i_main_status, i_param_initial_io_direction_a, i_param_initial_io_pin_state_a, i_param_initial_io_direction_b, i_param_initial_io_pin_state_b;
    static uint16_t us_initial_io_direction_a, us_initial_io_pin_state_a, us_initial_io_direction_b, us_initial_io_pin_state_b;

    i_main_status = ki_ERROR_NONE;

    // init ROS node
    ros::init( argc, argv, "zeabus_elec_peripheral_bridge" );
    ros::NodeHandle x_node_handle( "/elec" );
    
    // retrive parameters from the launch file
    x_node_handle.param<int>( "/zeabus_elec_peripheral_bridge/i_initial_io_direction_a", i_param_initial_io_direction_a, 0xFFFFU );
    x_node_handle.param<int>( "/zeabus_elec_peripheral_bridge/i_initial_io_pin_state_a", i_param_initial_io_pin_state_a, 0x0000U );
    x_node_handle.param<int>( "/zeabus_elec_peripheral_bridge/i_initial_io_direction_b", i_param_initial_io_direction_b, 0xFFFFU );
    x_node_handle.param<int>( "/zeabus_elec_peripheral_bridge/i_initial_io_pin_state_b", i_param_initial_io_pin_state_b, 0x0000U );
    x_node_handle.param<double>( "/zeabus_elec_peripheral_bridge/lf_atm_pressure", lf_atm_pressure, klf_ONE_ATM_AS_PSI );
    x_node_handle.param<double>( "/zeabus_elec_peripheral_bridge/lf_depth_offset", lf_depth_offset, 0.0  );

    /* cast int to uint16_t because NodeHandle::param doesn't support uint16_t */
    us_initial_io_direction_a = ( uint16_t )i_param_initial_io_direction_a;
    us_initial_io_pin_state_a = ( uint16_t )i_param_initial_io_pin_state_a;
    us_initial_io_direction_b = ( uint16_t )i_param_initial_io_direction_b;
    us_initial_io_pin_state_b = ( uint16_t )i_param_initial_io_pin_state_b;

    // register log publisher to ROS
    x_publisher_node_status_log =       x_node_handle.advertise<zeabus_elec_ros::MessageNodeStatus>( "node_status", 100U );
    x_publisher_hardware_error_log =    x_node_handle.advertise<zeabus_elec_ros::MessageHardwareError>( "hardware_error", 100U );
    x_publisher_action_log =            x_node_handle.advertise<zeabus_elec_ros::MessageAction>( "action", 100U );

    do
    {
        // wait for ROS initiation
        ros::Duration( 0.5 ).sleep();
    }while( !ros::ok() );

    v_log_node_status( x_publisher_node_status_log, std::string( "Peripheral bridge node started" ) );
    
    // create the device manager object, also open the device
    px_peripheral_bridge_a = std::make_shared<Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl>( Zeabus_Elec::FT4232H, kx_PERIPHERAL_BRIDGE_DESCRIPTION, 1U );
    px_peripheral_bridge_b = std::make_shared<Zeabus_Elec::ftdi_spi_cpol1_cha0_msb_impl>( Zeabus_Elec::FT4232H, kx_PERIPHERAL_BRIDGE_DESCRIPTION, 2U );

    try
    {
        // check the device manager object status after created
        if( px_peripheral_bridge_a->GetCurrentStatus() != 0U )
        {
            // unable to init peripheral bridge a
            throw( ki_ERROR_UNABLE_TO_OPEN_PERIPHERAL_BRIDGE_A );
        }

        if( px_peripheral_bridge_b->GetCurrentStatus() != 0U)
        {
            // unable to init peripheral bridge b
            throw( ki_ERROR_UNABLE_TO_OPEN_PERIPHERAL_BRIDGE_B );
        }

        // set initialize GPIO direction and GPIO pin state
        px_peripheral_bridge_a->SetGPIODirection( us_initial_io_direction_a, us_initial_io_pin_state_a );	
        px_peripheral_bridge_b->SetGPIODirection( us_initial_io_direction_b, us_initial_io_pin_state_b );

        if( px_peripheral_bridge_a->GetCurrentStatus() != 0U )
        {
            // unable to initialize GPIO direction and GPIO pin state of peripheral bridge a 
            throw( ki_ERROR_UNABLE_TO_INIT_PERIPHERAL_BRIDGE_A_GPIO );
        }

        if( px_peripheral_bridge_b->GetCurrentStatus() != 0U)
        {
            // unable to initialize GPIO direction and GPIO pin state of peripheral bridge b
            throw( ki_ERROR_UNABLE_TO_INIT_PERIPHERAL_BRIDGE_B_GPIO );
        }

        // register publisher to ROS
        x_publisher_barometer_value = x_node_handle.advertise<zeabus_elec_ros::MessageBarometerValue>( "barometer_value", 100U );

        // register service server to ROS
        x_service_server_get_depth =        x_node_handle.advertiseService( "/sensor/pressure", b_service_get_depth );
        x_service_server_set_io_pin_state = x_node_handle.advertiseService( "set_io_pin_state", b_set_io_pin_state );
        
        // define frequency of this node
        ros::Rate x_rate( 100U );

        while( ros::ok() )
        {
            try
            {
                v_get_barometer_value();
            }
            catch( const int &ki_error )
            {
                std::string x_description;
                int i_hardware_error_code;

                // prepare the log
                switch( ki_error )
                {
                    case ki_ERROR_UNABLE_TO_REQURST_BAROMETER_VALUE:
                        i_hardware_error_code = px_peripheral_bridge_a->GetCurrentStatus();
                        x_description = std::string( "Unable to request barometer value from peripheral bridge" );
                        break;
                    case ki_ERROR_UNABLE_TO_RECEIVE_BAROMETER_VALUE:
                        i_hardware_error_code = px_peripheral_bridge_a->GetCurrentStatus();
                        x_description = std::string( "Unable to receive barometer value from peripheral bridge" );
                        break;
                    default:
                        i_hardware_error_code = -1;
                        x_description = std::string( "Unknown error" );
                        break;
                }

                // print and publish the log
                v_log_hardware_error(   x_publisher_hardware_error_log,
                                        ( int64_t ) ki_error,
                                        ( int64_t ) i_hardware_error_code,
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
        int i_hardware_error_code;

        i_main_status = ki_error;

        // prepare the log description message
        switch( ki_error )
        {
            case ki_ERROR_UNABLE_TO_OPEN_PERIPHERAL_BRIDGE_A:
                i_hardware_error_code = px_peripheral_bridge_a->GetCurrentStatus();
                x_description = std::string( "Unable to initialize peripheral bridge A" );
                break;
            case ki_ERROR_UNABLE_TO_OPEN_PERIPHERAL_BRIDGE_B:
                i_hardware_error_code = px_peripheral_bridge_b->GetCurrentStatus();
                x_description = std::string( "Unable to initialize peripheral bridge B" );
                break;
            case ki_ERROR_UNABLE_TO_INIT_PERIPHERAL_BRIDGE_A_GPIO:
                i_hardware_error_code = px_peripheral_bridge_a->GetCurrentStatus();
                x_description = std::string( "Unable to initialize GPIO direction and GPIO pin state of peripheral bridge A" );
                break;
            case ki_ERROR_UNABLE_TO_INIT_PERIPHERAL_BRIDGE_B_GPIO:
                i_hardware_error_code = px_peripheral_bridge_b->GetCurrentStatus();
                x_description = std::string( "Unable to initialize GPIO direction and GPIO pin state of peripheral bridge B" );
                break;
            default:
                i_hardware_error_code = -1;
                x_description = std::string( "Unknown error" );
        }

        // print and publish the log
        v_log_hardware_error_fatal( x_publisher_hardware_error_log,
                                    ( int64_t )ki_error,
                                    ( int64_t )i_hardware_error_code,
                                    x_description );
    }

    v_log_node_status( x_publisher_node_status_log, std::string( "Peripheral bridge node is shutting down" ) );
    
    // normally, the program should never reach this point
    return i_main_status;
}
