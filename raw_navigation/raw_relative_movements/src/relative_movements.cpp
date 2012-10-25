#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <raw_srvs/SetPoseStamped.h>
#include <raw_srvs/SetMarkerFrame.h>
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"
#include <LinearMath/btMatrix3x3.h>
#include <XmlRpcValue.h>
#include "HomogenousTransform.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <raw_base_placement/OrientToBaseAction.h>
#include <angles/angles.h>

using namespace std;

double Velocity = 0.1;


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
   tf::Quaternion q;

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
	   if( rotate() )
		   if( moveX() )
			   moveY();

   } 
   bool moveX()
   {
        bool isReached=false;
        geometry_msgs::Twist zero_vel; 

        base_velocities_publisher.publish(zero_vel);

	    while(!odom_received)
        {
        	ros::spinOnce();   
        }
        odom_received = false;
        
        x_initodom = x_tempodom;
        y_initodom = y_tempodom;


        std::cout << "shift in x: " << xval << std::endl;
        std::cout << "init odom: x:" << x_initodom << " y: " << y_initodom << std::endl;

        theta_initodom = theta_tempodom;
        x_currentodom = x_tempodom;
        y_currentodom = y_tempodom;
        theta_currentodom = theta_tempodom;
        cout<<x_initodom<<endl;
        while(isReached != true)
        {
            base_velocities_publisher.publish(zero_vel);

            float valuediff = (x_currentodom-x_initodom);
            if( fabs(valuediff) >= fabs(xval))
            {
            	youbot_base_velocities.linear.x = 0.0;
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

        
        base_velocities_publisher.publish(zero_vel);

        cout<<x_currentodom<<endl;

        return true;
   }
   bool moveY()
   {
        geometry_msgs::Twist zero_vel;

        base_velocities_publisher.publish(zero_vel);

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
            base_velocities_publisher.publish(zero_vel);
            float valuediff = (y_currentodom-y_initodom);
            
            std::cout << "diff y: " << valuediff << std::endl;
            
            if( fabs(valuediff) >= fabs(yval))
            {
                std::cout << "pose reached, current odom: " << y_currentodom <<  std::endl;
                youbot_base_velocities.linear.y = 0.0;
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
            if(fabs(valuediff)>3.1416){valuediff=theta_currentodom+theta_initodom;}
            if( fabs(valuediff) >= fabs(yawval))
            {
            	youbot_base_velocities.angular.z = 0.0;
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
        while(!odom_received)
        {
           	ros::spinOnce();   
        }
        odom_received = false;
        x_initodom = x_tempodom;
        y_initodom = y_tempodom;
        theta_initodom = theta_tempodom;

        actionlib::SimpleActionClient<raw_base_placement::OrientToBaseAction> ac("/scan_front_orientation", true);

        ac.waitForServer();

        raw_base_placement::OrientToBaseActionGoal goal;

        goal.goal.distance = 0.1;

        ac.sendGoal(goal.goal);
        
        bool finished_before_timeout = ac.waitForResult(ros::Duration(1.0));

        while(!finished_before_timeout)
        {
           finished_before_timeout = ac.waitForResult(ros::Duration(0.5));  
        }

        bool base_reached = false;

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

            base_reached = moveY();
        }
        else  
        {
            ROS_INFO("Action did not finish before the time out.");
        } 
        
        return base_reached;

   }


};

bool move_base_relative(raw_srvs::SetPoseStamped::Request  &req, raw_srvs::SetPoseStamped::Response &res)
{
    tf::Quaternion q;
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
    tf::Quaternion q;
    double roll, pitch, yaw;

    float X = req.pose.pose.position.x ;

    float Y = req.pose.pose.position.y ;

    tf::quaternionMsgToTF(req.pose.pose.orientation, q);
    btMatrix3x3(q).getRPY(roll, pitch, yaw);  

    ros::NodeHandle node;

    BaseMotionController bm(node,X,Y,(float)roll,(float)pitch,(float)yaw);

    bm.moveoptimal();

    return true;
}

bool alignwithmarker(raw_srvs::SetMarkerFrame::Request  &req, raw_srvs::SetMarkerFrame::Response &res)
{

    bool is_rotation_done = false;
    bool is_x_done = false;
    bool is_y_done = false;
    ros::NodeHandle alignmarker;

    ros::Publisher base_velocities_publisher = alignmarker.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );
    
    tf::TransformListener listener;
  
    ros::Duration rate(20.0);

    bool isreached = false;

    geometry_msgs::Twist zero;

    ros::Duration max_time(50);
   
    ros::Time stamp = ros::Time::now();

    while (!isreached)
   {

    tf::StampedTransform transform;

    double roll, pitch, yaw;

       try
       {
           
           listener.lookupTransform(req.marker_frame, "/base_link", ros::Time(0), transform);
         
           geometry_msgs::Twist cmd;

           btMatrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

           

           double x = transform.getOrigin().x();
 
           
           double y = transform.getOrigin().y();

          
         
           cmd = zero;

           yaw = angles::normalize_angle(yaw);
           yaw = angles::shortest_angular_distance(0, yaw);

           ROS_INFO("yaw %lf, x %lf, y %lf",yaw,x,y); 
/*
           if(fabs(yaw) > 0.1 && !is_rotation_done)
           {
                
                if(yaw>0)
                    cmd.angular.z = -0.025; 
                else
                    cmd.angular.z = 0.025;
                
                ROS_INFO("Anglular displacement"); 

           }
           else if (fabs(yaw) <= 0.1)*/
                is_rotation_done = true;
           
            if(fabs(y)>0.01 && !is_y_done && is_rotation_done)
           {
                
                if(y>0)
                cmd.linear.y = -0.02;
                else
                cmd.linear.y = 0.02;

                ROS_INFO("y displacement");

           } 
            else  if (fabs(y) <= 0.01)
            {
                is_y_done = true;
          
              if(fabs(x)>0.01 && !is_x_done && is_rotation_done && is_y_done)
               {          
                    if(x>0)
                    cmd.linear.x = -0.02;
                    else
                    cmd.linear.x = +0.02;

                    ROS_INFO("x displacement");
               }
               else if (fabs(x) <= 0.01)
                {    

                    is_x_done = true;
                    
               }
            }

            if(!is_rotation_done || !is_x_done || !is_y_done)
                base_velocities_publisher.publish(cmd);
        
            if(is_rotation_done && is_x_done && is_y_done)
            {
                base_velocities_publisher.publish(zero);
            
                isreached =  true;

                ROS_INFO(" Base reached Marker Target frame");
           
                return true; 
            }

       }
       catch (tf::TransformException ex)
       {
          base_velocities_publisher.publish(zero);  
          ROS_ERROR("%s",ex.what());
       }


        if  (stamp + max_time < ros::Time::now()) {
        base_velocities_publisher.publish(zero);
        ROS_INFO("Marker alignment Time out");
		return false;
		break;
		}
       
    }

    return true;

 }


 

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle n;

  ros::ServiceServer shift_base = n.advertiseService( "move_base_relative", move_base_relative);

  ros::ServiceServer move_optimal_base = n.advertiseService( "movetooptimalbase", moveoptimalbase);

  ros::ServiceServer align_with_marker = n.advertiseService( "alignwithmarker", alignwithmarker);

  ROS_INFO("Ready to move base position");

  ros::spin();
  return 0;
}
