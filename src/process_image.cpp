#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <numeric>
#include <cmath>

// Define a global client that can request services
ros::ServiceClient client;
float max_lin_x = 0.3;
float max_ang_z = 0.3;
float target_distance = 0.5;
float FOV = 4.0 / 9.0 * M_PI;
float ball_r = 0.1;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    
    int white_pos = 0;
    int white_count = 0;

    float lin_x, ang_z;

    for (int i = 0; i < img.height; i++)
    {
      for (int j = 0; j < img.width; j++)
      {
        if ((img.data[(i*img.width+j)*3]==white_pixel) && (img.data[(i*img.width+j)*3+1]==white_pixel) && (img.data[(i*img.width+j)*3+2]==white_pixel))
        {
          white_pos += j;
          white_count++;
        }
      }
    }

    if (white_count == 0)
    {
      lin_x = 0.0; ang_z = 0.0;
    }
    else
    {
      float ball_pos_x = (float) ball_r*img.width/(2.0*tan(FOV/2.0)*sqrt((float)white_count/M_PI));
      float ball_pos_y = ((float) -white_pos/white_count/img.width*2.0-1.0) * ball_pos_x * tan(FOV/2.0);
      ROS_INFO("x_pos: %1.2f, y_pos: %1.2f", ball_pos_x, ball_pos_y);
    }
    
    

    if (white_count > 0)
    {
      if (white_pos/white_count < img.width/3)
      {
        // send turn left srv
        // ROS_INFO("Turn left - position: %d, count: %d", white_pos/white_count, white_count);
        drive_robot(0.1, 0.2);
      }
      else if (white_pos/white_count < img.width/3*2)
      {
        // send go straight srv
        // ROS_INFO("Go straight - position: %d, count: %d", white_pos/white_count, white_count);
        drive_robot(0.125, 0.0);
      }
      else
      {
        // send turn right srv
        // ROS_INFO("Turn right - position: %d, count: %d", white_pos/white_count, white_count);
        drive_robot(0.1, -0.2);
      }
    }
    else
    {
      // send stop srv
      ROS_INFO("No white_ball, stop");
      drive_robot(0.0, 0.0);
    }
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
