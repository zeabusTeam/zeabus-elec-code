#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zeabus_elec_ros_power_dist/power_dist.h>
#include <ftdi_impl.h>

#include <memory>
#include <sstream>

/* ===================================================
 * File-scope global variables
 * ===================================================
 */
static const char* cstPowerDistSerial = "PowerDist";

static std::shared_ptr<Zeabus_Elec::ftdi_mpsse_impl> pxMssp;
static ros::Publisher errMsgPublisher;	/* Publisher for error message */
static ros::ServiceServer switchServiceServer; /* Service server for switch-controlling */

/* ===================================================
 * ROS service subroutines
 * ===================================================
 */
bool ZeabusElec_SetSwitch( zeabus_elec_ros_power_dist::power_dist::Request &req,
                            zeabus_elec_ros_power_dist::power_dist::Response &res )
{
	int ftStat;

        res.result = true;
	
	ftStat = pxMssp->SetHiGPIOData( req.switchState );
	if( ftStat != 0 )
	{
		/* Some Error occurred. So, we publish the error message */
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Error in power-distributor controller with the code " << ftStat;
		msg.data = ss.str();
		
		errMsgPublisher.publish( msg );

                res.result = false;
	}

        return res.result;
}

/* ===================================================
 * main
 * ===================================================
 */
int main( int argc, char** argv )
{
	/* Initialize ROS functionalities */	
	ros::init(argc, argv, "Zeabus_Elec_Power_dist");
 	ros::NodeHandle nh("/zeabus/elec");

	/*=================================================================================
	  Discover the Power Distributor and also open handles for it.
	  =================================================================================*/

	/* Create the device manager class to implement chip functions */
	pxMssp = std::make_shared<Zeabus_Elec::ftdi_mpsse_impl>( Zeabus_Elec::FT232H, cstPowerDistSerial );
	
	if( pxMssp->GetCurrentStatus() != 0 )
	{
		/* Fail - unable to initialize Power Distribution module */
		ROS_FATAL( "Unable to initialize Power Distribution module" );
		return( -1 );
		
	}
	
	/* Set GPIO direction and pin intial state to all output, bit=1 means output, 0 means input */
	pxMssp->SetGPIODirection( 0xFFFF , 0x0000 );	/* All bits are output, initial pin state is low */
	if( pxMssp->GetCurrentStatus() != 0 )
	{
		/* Fail - unable to initialize Power Distribution module */
		ROS_FATAL( "Unable to initialize set-up GPIO direction" );
		return( -1 );
		
	}
	
	/*=================================================================================
	  Now the FTDI chip is opened and hooked. We can continue ROS registration process 
	  =================================================================================*/
	
	/* Register ROS publisher to Elec/Hw_error topic */
	errMsgPublisher = nh.advertise<std_msgs::String>( "hw_error", 1000 );
	/* Register ROS service server to Elec/Power_switch topic */
	switchServiceServer = nh.advertiseService( "power_switch", ZeabusElec_SetSwitch );

	/* Main-loop. Just a spin-lock */
	ros::spin();
	
	/*=================================================================================
	  At this point of code. ROS has some fatal errors or just normal shutdown. Also us. 
	  =================================================================================*/
	return( 0 );
}
