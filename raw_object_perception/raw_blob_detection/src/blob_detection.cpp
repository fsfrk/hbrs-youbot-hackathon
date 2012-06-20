//---------------------------------------------------------- blob_detection.cpp
//-----------------------------------------------------------------------------
//  Description: 
//
//    This is a ROS node that is designed to perform blob detection to look for
//  objects in images that are provided through a web camera interface. It 
//  requires the use of the b-it-bots raw_usb_cam ROS node in order to properly
//  recieve the data to use.
//-----------------------------------------------------------------------------
//  Author: Matthew S Roscoe [mat.roscoe@unb.ca]
//-----------------------------------------------------------------------------
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h" 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "geometry_msgs/Twist.h"

// BOOST
#include <boost/units/systems/si.hpp>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

// cvBlobsLib Includes.
#include <cvblobs/BlobResult.h>

#include <std_srvs/Empty.h>

// Arm Movement Stuff
#include <arm_navigation_msgs/JointLimits.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

class ImageConverter 
{

//-----------------------------------------------------------------------------
//------------------------------ PUBLIC FUNCTIONS -----------------------------
//-----------------------------------------------------------------------------
public:

  //------------------------------------------------------------ ImageConverter
  //---------------------------------------------------------------------------
  //    This function is used for all of the incoming and outgoing ROS messages
  //---------------------------------------------------------------------------
  ImageConverter(ros::NodeHandle &n) : n_(n), it_(n_)
  {
    XmlRpc::XmlRpcValue param_list;
    n_.getParam("/arm_1/arm_controller/joints", param_list);
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < param_list.size(); ++i)
    {
      ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      arm_joint_names_.push_back(static_cast<std::string>(param_list[i]));
    }

    //read joint limits
    for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
    {
      arm_navigation_msgs::JointLimits limit;
      limit.joint_name = arm_joint_names_[i];
      n_.getParam("/arm_1/arm_controller/limits/" + arm_joint_names_[i] + "/min", limit.min_position);
      n_.getParam("/arm_1/arm_controller/limits/" + arm_joint_names_[i] + "/max", limit.max_position);
      arm_joint_limits_.push_back(limit);

    }

    // Velocity control for the YouBot base.
    base_movement = n_.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 ); 

    // Rotational Control for the YouBot arm. 
    pub_arm_vel = n_.advertise<brics_actuator::JointVelocities>( "/arm_1/arm_controller/velocity_command", 1 );

    // Service commands to allow this node to be started and stopped externally
    _start_srv = n_.advertiseService("start", &ImageConverter::Start, this);
    _stop_srv = n_.advertiseService("stop", &ImageConverter::Stop, this);
    ROS_INFO( "Advertised 'start' and 'stop' service for raw_blob_detection" );

    ROS_INFO( "Blob Detection Started" );
  }

  //----------------------------------------------------------- ~ImageConverter
  //---------------------------------------------------------------------------
  //   Standard Destructor.
  //--------------------------------------------------------------------------- 
  ~ImageConverter()
  {
    //  OpenCV calls to destroy any HighGUI windows that may have been opened using
    //  the provided names. 
    cvDestroyWindow( "Original" );
    cvDestroyWindow( "Thresholding" ); 
    cvDestroyWindow( "Found Blobs" ); 
  }

  //------------------------------------------------------------- imageCallBack
  //---------------------------------------------------------------------------
  //    This function is the call back function for the ROS Image Message. 
  //  Each time a new image comes from the USB Camera (through raw_usb_cam)
  //  this function will be called. It is responsible for all of the blob
  //  detection as well as any processing that is applied to the images.
  //---------------------------------------------------------------------------
  void imageCallback( const sensor_msgs::ImageConstPtr& msg_ptr )
  {
    int master_image_width = 0; 
    int master_image_height = 0; 

    double x_offset = 0; 
    double y_offset = 0; 
    double rot_offset = 0; 

    bool done_rotational_adjustment = false; 
    bool done_base_movement_adjustment = false; 

    IplImage* cv_image = NULL;
    IplImage* blob_image = NULL; 

    CBlob* currentBlob; 

    // Covert the image from a ROS image message to a OpenCV Image (IplImage) type.
    try
    {
      cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR( "Error converting from ROS image message to OpenCV IplImage" );
    }

    //  Obtain image properties that we require. 
    master_image_width = cv_image->width; 
    master_image_height = cv_image->height; 

    IplImage* gray = cvCreateImage( cvGetSize( cv_image ), 8, 1 );
    cvCvtColor( cv_image, gray, CV_BGR2GRAY );

    cvSmooth( cv_image, cv_image, CV_GAUSSIAN, 7, 7 );

    blob_image = cvCreateImage( cvGetSize( cv_image ), 8, 3 ); 

    cvThreshold( gray, gray, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

    // Find any blobs that are not white. 
    CBlobResult blobs = CBlobResult( gray, NULL, 0 );

    //  Make sure they are big enough to really be considered.
    //  In this case we will use an area of AT LEAST 100 px. 
    int minimum_blob_area = 100; 
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, minimum_blob_area ); 

    int blob_number = blobs.GetNumBlobs(); 
    std::cout << "\nNumber of Blobs Present: " << blob_number << std::endl; 

    CBlob largest_blob; 
    blobs.GetNthBlob( CBlobGetPerimeter(), 0, largest_blob ); 
    //  Add the found blobs to the blob_image.
    for ( int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
      CBlobGetOrientation get_orientation; 
      CBlobGetArea get_blob_area; 

      currentBlob = blobs.GetBlob(i);
      currentBlob->FillBlob( blob_image, CV_RGB( 0, 0, 255 ) );

      // Obtain distance information about the blob. This in particular looks at the distance between the
      // centroid of the blob (est) and the center of the image being captured (intersection of the axis).
      double maxx = currentBlob->MaxX(); 
      double minx = currentBlob->MinX(); 
      double maxy = currentBlob->MaxY(); 
      double miny = currentBlob->MinY(); 
      double blob_x = ( ( minx + maxx ) / 2 );
      double blob_y = ( ( miny + maxy ) / 2 ); 
      double dist_x = ( blob_x ) - ( master_image_width / 2 ); 
      double dist_y = ( blob_y ) - ( master_image_height / 2 ); 
      double distance = sqrt( ( dist_x * dist_x ) + ( dist_y * dist_y ) ); 

      double rotation = 0.0; 
      rotation = get_orientation( *currentBlob );

      //  DEBUGGING
      std::cout << "Blob #:\t\t\t" << i << std::endl; 
      std::cout << "Blob Center x:\t\t" << blob_x << std::endl; 
      std::cout << "Blob Center y:\t\t" << blob_y << std::endl; 
      std::cout << "Distance to Center:\t" << distance << std::endl; 
      std::cout << "Horizontal Offset:\t" << dist_x << std::endl; 
      std::cout << "Vertical Offset:\t" << dist_y << std::endl; 
      std::cout << "Rotational Offset:\t" << rotation << std::endl; 
      std::cout << "\n" << std::endl; 


      // Base adjustment stuff. This code assumes that the largest blob is the one that you
      // want to work with. This being the case the arm/camera will need to be guided to the
      // object in question before it can be turned on as this system will direct the arm to
      // interact with the largest blob that it can find. This module should only be used once
      // it is simply the object and its background in the frame of view of the camera. 
      if( get_blob_area( largest_blob ) == get_blob_area( *currentBlob ) )
      {
        x_offset = dist_x; 
        y_offset = dist_y; 
        rot_offset = rotation; 

        //---------------------------------------------------------------------
        //-------------------- base movement control --------------------------
        //---------------------------------------------------------------------
        if( x_offset != 0 )
        {
          double move_speed = 0.0; 

          // added a buffer for a "good enough" region of interest. [14.06.2012]
          if( x_offset >= 10 )
          {
            // move the robot base right
            move_speed = -0.005; 
            done_base_movement_adjustment = false; 
          }
          else if( x_offset <= -10 )
          {
            // move the robot left
            move_speed = 0.005; 
            done_base_movement_adjustment = false; 
          }
          else if( x_offset > -10 && x_offset < 10 )
          {
            move_speed = 0.0;
            done_base_movement_adjustment = true;  
          }
          else
          {
            // should never happen but just in case.
            move_speed = 0.0; 
          }

          // Prepare and then send the base movement commands.
          base_velocity.linear.y = move_speed; 
          base_movement.publish( base_velocity ); 
        }
        //------------------ END OF BASE MOVEMENT CONTROL ---------------------

        
        //---------------------------------------------------------------------
        //--------------------- arm rotation control --------------------------
        //---------------------------------------------------------------------
        if( rot_offset != 90 || rot_offset != 270 )
        {
          double rotational_speed = 0.0; 

	// TODO fix offsets to move in the proper directions.
          if( ( rot_offset < 87 && rot_offset >= 0 ) || ( rot_offset < 267 && rot_offset >= 235 ) )
          {
            rotational_speed = -0.2; 
            done_rotational_adjustment = false; 
          }
          else if( rot_offset > 93 && rot_offset < 235 )
          {
            rotational_speed = 0.1; 
            done_rotational_adjustment = false; 
          }
          else
          {
            rotational_speed = 0.0; 
            done_rotational_adjustment = true; 
          }

          arm_vel_.velocities.clear();
          for(unsigned int i=0; i < arm_joint_names_.size(); ++i)
          {
            brics_actuator::JointValue joint_value;

            joint_value.timeStamp = ros::Time::now();
            joint_value.joint_uri = arm_joint_names_[i];
            joint_value.unit = to_string(boost::units::si::radian_per_second);
            
            if( i == 4 )
            {
              joint_value.value = rotational_speed;
            }
            else
            {
              joint_value.value = 0.0; 
            }

            arm_vel_.velocities.push_back(joint_value);
            // Publish the arm velocity commands.
            pub_arm_vel.publish( arm_vel_ );
          }
        }
        //------------------- END OF ARM ROTATION CONTROL ---------------------

        // make sure the last thing we do is paint one centroid for debugging.
        cvCircle( blob_image, cvPoint( blob_x, blob_y ), 10, CV_RGB( 255, 0, 0 ), 2 );
      } 
    }

    //-------------------------------------------------------------------------
    //---------------- VISUAL OUTPUT FOR DEBUGGING ONLY -----------------------
    //-------------------------------------------------------------------------
    //    When this is being used on the robot all of the code below may be 
    //  commented out as there is no need to see what the robot is seeing. This
    //  was meant only for development purposes. Commenting it out will consume
    //  fewer resources on the robot.
    //-------------------------------------------------------------------------
    
    // Setting up fonts for overlay information.
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

    cvLine( blob_image,   cvPoint( 0, (master_image_height/2) ), cvPoint( master_image_width, (master_image_height / 2) ), CV_RGB( 255, 0, 0 ), 2, 0 ); 
    cvLine( blob_image,   cvPoint( (master_image_width/2), 0 ), cvPoint( (master_image_width/2), master_image_height ), CV_RGB( 255, 0, 0 ), 2, 0 );
    cvRectangle( blob_image, cvPoint( 0, blob_image->height-40 ), cvPoint( blob_image->width, blob_image->height ), CV_RGB( 0, 0, 0 ), -1 );

    std::string x_str = "X: "; 
    x_str += boost::lexical_cast<std::string>( x_offset ); 

    std::string y_str = "Y: "; 
    y_str += boost::lexical_cast<std::string>( y_offset ); 

    std::string rot_str = "Rotation: "; 
    rot_str += boost::lexical_cast<std::string>( rot_offset ); 

    cvPutText( blob_image, x_str.c_str(), cvPoint( 50, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
    cvPutText( blob_image, y_str.c_str(),  cvPoint( 200, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
    cvPutText( blob_image, rot_str.c_str(), cvPoint( 350, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );

    cvShowImage( "Found Blobs", blob_image ); 

    //-------------------------------------------------------------------------
    //----------------------- END OF VISUAL OUTPUT ----------------------------
    //-------------------------------------------------------------------------

    //  Wait for user interaction.
    cvWaitKey(3);
  }

  //--------------------------------------------------------------------- Start
  //---------------------------------------------------------------------------
  //   Used to start up the processing of the web camera images once the node 
  //  has been told to start.
  //--------------------------------------------------------------------------- 
  bool Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
     //  Incoming message from raw_usb_cam. This must be running in order for this ROS node to run.
    image_sub_ = it_.subscribe( "/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this );

    ROS_INFO("Blob Detection Enabled");

    return true;
  }

  //---------------------------------------------------------------------- Stop
  //---------------------------------------------------------------------------
  //   Used to stop the processing of the web camera images once the node has
  //  been asked to stop. Note this does not remove the nodes service it only
  //  halts the processing.
  //--------------------------------------------------------------------------- 
  bool Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    image_sub_.shutdown(); 

	// TODO Fix so that it stops movements.

    ROS_INFO("Blob Detection Disabled");

    return true;
  }

//-----------------------------------------------------------------------------
//--------------------- PROTECTED FUNCTIONS / VARIABLES -----------------------
//-----------------------------------------------------------------------------
protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;

  // Topics that this node publishes to.
  ros::Publisher base_movement; 
  ros::Publisher arm_movement;

  ros::Publisher pub_arm_vel;

  // base movement topic.
  geometry_msgs::Twist base_velocity;

  // Arm Joint Names.
  std::vector<std::string> arm_joint_names_;
  std::vector<arm_navigation_msgs::JointLimits> arm_joint_limits_;
  brics_actuator::JointVelocities arm_vel_;

  // Stop and start services for this ROS node.
  ros::ServiceServer _start_srv; 
  ros::ServiceServer _stop_srv;
};

//------------------------------------------------------------------------ main
//-----------------------------------------------------------------------------
//    Main Function. Should be self evident.
//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::spin();
  return 0;
}
