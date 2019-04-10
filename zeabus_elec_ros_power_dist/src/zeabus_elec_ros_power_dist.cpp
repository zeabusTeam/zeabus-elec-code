#include <cstdint>
#include <cstdio>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
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

    static const std::string kac_POWER_DISTRIBUTOR_DESCRIPTION = std::string( "PowerDist" );
    static const std::string kx_POWER_DISTRIBUTOR_NODE_NAME = std::string( "PowerDist" );
    static const std::string kx_SET_POWER_SWITCH_SERVICE_NAME = std::string( "elec/power_dist/set_power_swtich" );

    static std::shared_ptr<PowerDistNode> px_node;

    /* Initialize ROS functionalities */	
    rclcpp::init( argc, argv );

    px_node = std::make_shared<PowerDistNode>( kx_POWER_DISTRIBUTOR_NODE_NAME, kx_SET_POWER_SWITCH_SERVICE_NAME );

    // TODO: Retrieve parameter from launch file

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
    
    /* Set GPIO direction and pin intial state to all output, bit=1 means output, 0 means input */
    px_node->px_mssp_->SetGPIODirection( 0xFFFFU , 0x0000U );       /* All bits are output, initial pin state is low */
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
