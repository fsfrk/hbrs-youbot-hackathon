#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <raw_srvs/SetPoseStamped.h>
#include <tf/transform_datatypes.h>
#include <LinearMath/btMatrix3x3.h>
#include <XmlRpcValue.h>
#include "HomogenousTransform.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <raw_base_placement_to_platform_in_front/OrientToBaseAction.h>


using namespace std;

double Velocity = 0.01;


class BaseMotionController

{

   private:

   // Preferred Displacement and Orientation
   float xval;
   float yval;
   float rollval;
   float pitchval;
   float yawval;

   // Init Odom value
   float x_initodom;
   float y_initodom;
   float theta_initodom;

   // Init Odom value
   float x_tempodom;
   float y_tempodom;
   float theta_tempodom;


   // Current Odom value
   float x_currentodom;
   float y_currentodom;
   float theta_currentodom;

   //temporary variables
   double roll, pitch, yaw;
   btQuaternion q;

   bool odom_received;
   // Odometry subscriber and Base Velocity Publisher
   ros::Publisher   base_velocities_publisher;   
   ros::Subscriber  base_odom;
   
   //base velcoity topic message
   geometry_msgs::Twist youbot_base_velocities;

   ros::NodeHandle node_handler;

   public:
   BaseMotionController( ros::NodeHandle &n ,float x,float y, float roll,float pitch,float yaw):node_handler(n)
   {
        xval = x;
        yval = y;
        rollval=roll;
        pitchval=pitch;
        yawval = yaw;
        // Velocity control for the YouBot base.
        base_velocities_publisher = node_handler.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );
        base_odom = node_handler.subscribe("/odom", 1, &BaseMotionController::OdomCallback, this);
        odom_received = false;
   }
   ~BaseMotionController()
   {
        base_odom.shutdown(); 
        base_velocities_publisher.shutdown(); 
   }

   void movebase()
   {
      bool xstatus = moveX();
      if(xstatus == true)
      {
        moveY();
      }   
   } 
   bool moveX()
   {
        bool isReached=false;

	    while(!odom_received)
        {
        	ros::spinOnce();   
        }
        odom_received = false;
        
        x_initodom = x_tempodom;
        y_initodom = y_tempodom;
        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;
        cout<<x_initodom<<endl;
        while(isReached != true)
        {
ROS_INFO("run");
            float valuediff = (x_currentodom-x_initodom);
            if( abs(valuediff) >= abs(xval)) 
            {
               isReached =true; 
            }
            else
            {
             xval>0?(youbot_base_velocities.linear.x = Velocity):(youbot_base_velocities.linear.x = -Velocity);
             base_velocities_publisher.publish( youbot_base_velocities );  
              
            }
            while(!odom_received)
            {
            	ros::spinOnce();   
            }
            odom_received = false;

            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom;  
        }   

        geometry_msgs::Twist zero_vel;
        base_velocities_publisher.publish(zero_vel);  
        cout<<x_currentodom<<endl;

        return true;
   }
   bool moveY()
   {

        bool isReached=false;

        while(!odom_received)
        {
        	ros::spinOnce();   
        }
        odom_received = false;

        x_initodom = x_tempodom;
        y_initodom = y_tempodom;


        std::cout << "shift in y: " << yval << std::endl;
        std::cout << "init odom: x:" << x_initodom << " y: " << y_initodom << std::endl;

        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;

        while(isReached != true)
        {
            //ROS_INFO("Run");  
            float valuediff = (y_currentodom-y_initodom);
            
            //std::cout << "diff y: " << valuediff << std::endl;
            
            if( abs(valuediff) >= abs(yval)) 
            {
                std::cout << "pose reached, current odom: " << y_currentodom <<  std::endl;
               isReached =true; 
            }
            else
            {
                yval>0?(youbot_base_velocities.linear.y = Velocity):(youbot_base_velocities.linear.y = -Velocity);
                
                base_velocities_publisher.publish( youbot_base_velocities );  
              
            }
            while(!odom_received)
            {
            	ros::spinOnce();   
            }
            odom_received = false;
            
            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom;
  
        }  

        geometry_msgs::Twist zero_vel;
        base_velocities_publisher.publish(zero_vel);  
        

        return true; 

   }
   bool rotate()
   {
        bool isReached=false;

        while(!odom_received)
        {
           	ros::spinOnce();   
        }
        odom_received = false;

        x_initodom = x_tempodom;
        y_initodom = y_tempodom;
        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;
        while(isReached != true)
        {
            float valuediff = (theta_currentodom-theta_initodom);
            if(abs(valuediff)>3.1416){valuediff=theta_currentodom+theta_initodom;}
            if( abs(valuediff) >= yawval) 
            {
               isReached =true; 
            }
            else
            {
                yawval>0?(youbot_base_velocities.angular.z = Velocity):(youbot_base_velocities.angular.z = -Velocity);
                base_velocities_publisher.publish( youbot_base_velocities );  
              
            }
            
            while(!odom_received)
            {
            	ros::spinOnce();   
            }
            odom_received = false;

            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom;
  
        }   

        geometry_msgs::Twist zero_vel;
        base_velocities_publisher.publish(zero_vel);
       return true;
   } 

    void OdomCallback( const nav_msgs::Odometry& Odom )
    {
        x_tempodom = Odom.pose.pose.position.x ;
        y_tempodom  = Odom.pose.pose.position.y ;
        tf::quaternionMsgToTF(Odom.pose.pose.orientation, q);
        btMatrix3x3(q).getRPY(roll, pitch, yaw);  
        theta_tempodom = yaw ;
	    odom_received = true; 
    } 
      
   bool moveoptimal()
   {
        ros::spinOnce();
        x_initodom = x_tempodom;
        y_initodom = y_tempodom;
        theta_initodom = theta_tempodom;

        actionlib::SimpleActionClient<raw_base_placement_to_platform_in_front::OrientToBaseAction> ac("/scan_front_orientation", true);
        ac.waitForServer();
        raw_base_placement_to_platform_in_front::OrientToBaseActionGoal goal;
        goal.goal.distance = 0.1;
        ac.sendGoal(goal.goal);
        bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
  
            ros::spinOnce();
            x_currentodom = x_tempodom;
            y_currentodom = y_tempodom;
            theta_currentodom = theta_tempodom;
            HomogenousTransform Objecttf = ht_from_xyzrpy(xval,yval,0,rollval,pitchval,yawval);
            float xdiff = x_initodom-y_currentodom;
            float ydiff = y_initodom-y_currentodom;
            float yawdiff = theta_initodom-theta_currentodom;
            HomogenousTransform OdomTf = ht_from_xyzrpy(xdiff,ydiff,0,0,0,yawdiff);
            HomogenousTransform finalTf = OdomTf*Objecttf;
            yval = (finalTf.translation())(1); 
            bool base_reached = moveY();
        }
        else  
        {
            ROS_INFO("Action did not finish before the time out.");
        } 

   }


};

bool shiftbase(raw_srvs::SetPoseStamped::Request  &req, raw_srvs::SetPoseStamped::Response &res)
{
    btQuaternion q;
    double roll, pitch, yaw;

    float X_dist = req.pose.pose.position.x ;
    float Y_dist = req.pose.pose.position.y ;

    Velocity = req.pose.pose.position.z;

    tf::quaternionMsgToTF(req.pose.pose.orientation, q);
    btMatrix3x3(q).getRPY(roll, pitch, yaw);  

    float Theta = yaw ;
 
    ros::NodeHandle node;
    
    BaseMotionController bm(node,X_dist,Y_dist,0,0,Theta);

    bm.movebase();


    return true;
} 

bool moveoptimalbase(raw_srvs::SetPoseStamped::Request  &req, raw_srvs::SetPoseStamped::Response &res)
{
    btQuaternion q;
    double roll, pitch, yaw;

    float X = req.pose.pose.position.x ;

    float Y = req.pose.pose.position.y ;

    tf::quaternionMsgToTF(req.pose.pose.orientation, q);
    btMatrix3x3(q).getRPY(roll, pitch, yaw);  

    ros::NodeHandle node;

    BaseMotionController bm(node,X,Y,(float)roll,(float)pitch,(float)yaw);

    bool status = bm.moveoptimal();

    return true;
}


 

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle n;

  ros::ServiceServer shift_base = n.advertiseService( "shiftbase", shiftbase);

  ros::ServiceServer move_optimal_base = n.advertiseService( "movetooptimalbase", moveoptimalbase);

  ROS_INFO("Ready to move base position");

  ros::spin();
  return 0;
}
