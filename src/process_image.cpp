#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <numeric>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    
    int white_pos = 0;
    int white_count = 0;

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

    if (white_count > 0)
    {
      if (white_pos/white_count < img.width/3)
      {
        // send turn left srv
        ROS_INFO("Turn left - position: %d, count: %d", white_pos/white_count, white_count);
      }
      else if (white_pos/white_count < img.width/3*2)
      {
        // send go straight srv
        ROS_INFO("Go straight - position: %d, count: %d", white_pos/white_count, white_count);
      }
      else
      {
        // send turn right srv
        ROS_INFO("Turn right - position: %d, count: %d", white_pos/white_count, white_count);
      }
    }
    else
    {
      // send stop srv
      ROS_INFO("No white_ball, stop");
    }
    
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
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
