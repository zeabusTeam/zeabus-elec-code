#include <cstdint>
#include <cstdio>
#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "zeabus_elec_ros_internal_interface/srv/power_dist_set_power_switch.hpp"
#include "zeabus_elec_ros_power_dist/srv/power_dist.hpp"

#include "ftdi_impl.h"

/* ===================================================
 * ROS node defination
 * ===================================================
 */
namespace
{
    class PowerDistNode : public rclcpp::Node
    {
        public:
            explicit PowerDistNode( const std::string &kx_node_name, const std::string &kx_service_name );

            std::shared_ptr<Zeabus_Elec::ftdi_mpsse_impl> px_mssp_;

        private:
            rclcpp::Service<zeabus_elec_ros_power_dist::srv::PowerDist>::SharedPtr px_srv_;
    };
}

PowerDistNode::PowerDistNode( const std::string &kx_node_name, const std::string &kx_service_name ) : rclcpp::Node( kx_node_name )
{
    auto v_handle_elec_power_dist = [this]( const std::shared_ptr<zeabus_elec_ros_power_dist::srv::PowerDist::Request> kx_req,
                                            std::shared_ptr<zeabus_elec_ros_power_dist::srv::PowerDist::Response> x_res ) -> void
    {
        int ft_state;
        uint8_t u_switch_state, u_current_switch_state, u_switch_mask;

        x_res->b_result = true;
        u_switch_mask = 0x01U << ( kx_req->u_switch_index );
        u_current_switch_state = this->px_mssp_->ReadHiGPIOData();

        if( kx_req->is_switch_high )
        {
            u_switch_state = ( u_current_switch_state | u_switch_mask );
        }
        else
        {
            u_switch_state = ( u_current_switch_state & ~( u_switch_mask ) );
        }

        ft_state = this->px_mssp_->SetHiGPIOData( u_switch_state );

        if( ft_state != 0 )
        {
            RCLCPP_ERROR( this->get_logger(), "Unable to set power switch of power_distributor" );
            x_res->b_result = false;
        }

        return;
    };

    /* Register ROS service server to Elec/Power_switch topic */
    this->px_srv_ = this->create_service<zeabus_elec_ros_power_dist::srv::PowerDist> ( kx_service_name, v_handle_elec_power_dist );
    
    return;
}

/* ===================================================
 * main
 * ===================================================
 */
int main( int argc, char *argv[] )
{
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );

    static const std::string kac_POWER_DISTRIBUTOR_DESCRIPTION( "PowerDist" );
    static const std::string kx_POWER_DISTRIBUTOR_NODE_NAME( "PowerDist" );
    static const std::string kx_SET_POWER_SWITCH_SERVICE_NAME( "elec/power_dist/set_power_swtich" );
    static const std::string kx_PARAMETER_IO_DIRECTION_NAME( "ul_io_direction" );
    static const std::string kx_PARAMETER_IO_STATE_NAME( "ul_io_state" );
    static const std::map<const std::string, const uint16_t> kx_DEFAULT_PARAMETER_LIST { { kx_PARAMETER_IO_DIRECTION_NAME, 0xFFFFU }, { kx_PARAMETER_IO_STATE_NAME, 0x0000U } };    /* bit=1 means output, 0 means input,
                                                                                                                                                                                        All bits are output, initial pin state is low */

    static std::shared_ptr<PowerDistNode> px_node;
    static std::map<const std::string, const uint16_t> x_parameter_list;

    /* Initialize ROS functionalities */	
    rclcpp::init( argc, argv );

    px_node = std::make_shared<PowerDistNode>( kx_POWER_DISTRIBUTOR_NODE_NAME, kx_SET_POWER_SWITCH_SERVICE_NAME );

    /* Get parameters from launch file */

    std::shared_ptr<rclcpp::SyncParametersClient> px_parameters_client = std::make_shared<rclcpp::SyncParametersClient>( px_node );

    while( !px_parameters_client->wait_for_service( std::chrono::seconds( 1 ) ) )
    {
        if( !rclcpp::ok() )
        {
            RCLCPP_FATAL( px_node->get_logger(), "Interrupted while waiting for the service");
            // TODO: Proper exception handling
            return( -1 );       // Let's pretend this return statement is throwing an exception
        }
    }

    for( auto x_default_parameter : kx_DEFAULT_PARAMETER_LIST )
    {
        const std::string x_parameter_name = x_default_parameter.first;
        uint16_t us_parameter;

        if( px_parameters_client->has_parameter( x_parameter_name ) )
        {
            auto x_parameter = px_parameters_client->get_parameters( { x_parameter_name } );

            us_parameter = static_cast<uint16_t>( x_parameter.front().as_int() );
        }
        else
        {
            us_parameter = kx_DEFAULT_PARAMETER_LIST.at( x_parameter_name );
        }
        x_parameter_list.insert( { { x_parameter_name, us_parameter } } );
    }

    /*=================================================================================
      Discover the Power Distributor and also open handles for it.
      =================================================================================*/

    /* Create the device manager class to implement chip functions */
    px_node->px_mssp_ = std::make_shared<Zeabus_Elec::ftdi_mpsse_impl>( Zeabus_Elec::FT232H, kac_POWER_DISTRIBUTOR_DESCRIPTION.c_str() );
    
    if( px_node->px_mssp_->GetCurrentStatus() != 0U )
    {
        /* Fail - unable to initialize Power Distribution module */
        RCLCPP_FATAL( px_node->get_logger(), "Unable to initialize power_distributor" );
        // TODO: Proper exception handling
        return( -1 );       // Let's pretend this return statement is throwing an exception
    }
    
    /* Set GPIO direction and pin intial state to all output */
    px_node->px_mssp_->SetGPIODirection( x_parameter_list[ kx_PARAMETER_IO_DIRECTION_NAME ] , x_parameter_list[ kx_PARAMETER_IO_STATE_NAME ] );
    if( px_node->px_mssp_->GetCurrentStatus() != 0U )
    {
        /* Fail - unable to initialize Power Distribution module */
        RCLCPP_FATAL( px_node->get_logger(), "Unable to setup GPIO direction of power_distributor" );
        // TODO: Proper exception handling
        return( -1 );       // Let's pretend this return statement is throwing an exception
    }
    
    /*=================================================================================
      Now the FTDI chip is opened and hooked. We can continue ROS registration process 
      =================================================================================*/
    
    /* Main-loop. Just a spin-lock */
    rclcpp::spin( px_node );
    
    /*=================================================================================
      At this point of code. ROS has some fatal errors or just normal shutdown. Also us. 
      =================================================================================*/
    rclcpp::shutdown();
    return 0;
}
