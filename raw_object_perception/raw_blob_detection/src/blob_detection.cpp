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

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

// cvBlobsLib Includes.
#include <cvblobs/BlobResult.h>

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
    //  Incoming message from raw_usb_cam. This must be running in order for this ROS node to run.
    image_sub_ = it_.subscribe( "/usb_cam/image_raw", 1, &ImageConverter::imageCallback, this );

    //  TODO: Add publishing messages to the YouBot Arm controllers.
    base_movement = n_.advertise<geometry_msgs::Twist>( "/cmd_vel", 1); 
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
    cvDestroyWindow( "Blob Detection" ); 
  }

  //------------------------------------------------------------- imageCallBack
  //---------------------------------------------------------------------------
  //    This function is the call back function for the ROS Image Message. 
  //  Each time a new image comes from the USB Camera (through raw_usb_cam)
  //  this function will be called. It is responsible for all of the blob
  //  detection as well as any processing that is applied to the images.
  //---------------------------------------------------------------------------
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    int master_image_width = 0; 
    int master_image_height = 0; 
    int master_image_center_x = 0;
    int master_image_center_y = 0;  

    double x_offset = 0; 
    double y_offset = 0; 
    double rot_offset = 0; 

    IplImage* cv_image = NULL;
    IplImage* blob_image = NULL;
    IplImage* display_image = NULL; 

    CBlob* currentBlob; 

    // Covert the image from a ROS image message to a OpenCV Image (IplImage) type.
    try
    {
      cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }

    //  Obtain image properties that we require. 
    master_image_width = cv_image->width; 
    master_image_height = cv_image->height; 
    master_image_center_x = ( master_image_width / 2 );
    master_image_center_y = ( master_image_height / 2 );

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
      //double dist_x = ( blob_x - master_image_center_x ) - ( master_image_width / 2 ); 
      //double dist_y = ( blob_y - master_image_center_y ) - ( master_image_height / 2 );
      double dist_x = ( blob_x ) - ( master_image_width / 2 ); 
      double dist_y = ( blob_y ) - ( master_image_height / 2 ); 
      double distance = sqrt( ( dist_x * dist_x ) + ( dist_y * dist_y ) ); 

      double rotation = 0.0; 
      //double rotation = tan( get_orientation( currentBlob ) * ( 3.1415926535 / 180 ) ); 

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
      //if( largest_blob == currentBlob )
      //{
        x_offset = dist_x; 
        y_offset = dist_y; 
        rot_offset = rotation; 

        // + = left / - = right
        if( x_offset != 0 )
        {
          double move_speed = 0.0; 

          // added a buffer for a "good enough" region of interest. [14.06.2012]
          if( x_offset > 0 && x_offset <= 20 )
          {
            // move the robot base left
            move_speed = -0.1; 
          }
          else if( x_offset < 0 && x_offset >= -20 )
          {
            // move the robot right
            move_speed = 0.1; 
          }
          else
          {
            // should never happen but just in case.
            move_speed = 0.0; 
          }

          // Prepare and then send the base movement commands.
          base_velocity.linear.y = move_speed; 
          base_movement.publish( base_velocity ); 

          cvCircle( blob_image, cvPoint( blob_x, blob_y ), 10, CV_RGB( 255, 0, 0 ), 2 );
        }
      //}
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
    //cvPutText( gray, "Hello World!", cvPoint( 10, gray->height - 10 ), &font, cvScalar( 255, 1, 1 ) );
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

    //cvShowImage( "Original", cv_image ); 
    //cvShowImage( "Thresholding", gray ); 
    cvShowImage( "Found Blobs", blob_image ); 
    //cvShowImage( "Blob Detection", display_image ); 

    //-------------------------------------------------------------------------
    //----------------------- END OF VISUAL OUTPUT ----------------------------
    //-------------------------------------------------------------------------

    //  Wait for user interaction.
    cvWaitKey(3);
  }

//-----------------------------------------------------------------------------
//--------------------- PROTECTED FUNCTIONS / VARIABLES -----------------------
//-----------------------------------------------------------------------------
protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  ros::Publisher base_movement; 
  geometry_msgs::Twist base_velocity;
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