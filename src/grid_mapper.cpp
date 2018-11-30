//written by jeffrey knoll 11/29/18

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>


using namespace boost::posix_time;


class GridMapper {
public:
  int odom;
  ros::Time time = ros::Time::now();
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height) :
      canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("scan", 1,&GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("odom", 1,&GridMapper::poseCallback, this);
      
    // Create resizeable named window
    //cv::namedWindow("Occupancy Grid Canvas", \
   //   CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };
  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + jknoll + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
	 
  float temp_state;
  float tempangle_Min = msg->angle_min;
  float tempangle_Max = msg->angle_max;
  float tempangle_Inc = msg->angle_increment;
  float max_Range = msg->range_max;
  int NumOfIters = ((tempangle_Max - tempangle_Min) / tempangle_Inc);
  float scans[NumOfIters]; 
  for(int i=0; i< NumOfIters; i++)
  {
    scans[i] = msg -> ranges[i];
  }

  for(int i=0; i < NumOfIters; i=i+9)
  {
    float _angle = (i * tempangle_Inc) + (tempangle_Min) + heading;

    while(_angle > M_PI || _angle < (-M_PI))
    {
      if(_angle > M_PI)
        {
	  _angle = _angle - 2 * M_PI;
	}
	if(_angle < (-M_PI))
	{
	  _angle = _angle + 2 * M_PI;
	}
     }

     float temp_2 = 10;
     if(odom == 1)
     {
       for(int j = 0; j < (max_Range * temp_2); j++)
         {
	   int temp_X;
	   int temp_Y;
	   //setting up the scaling factor for the grid
	   temp_X = - (sin(_angle) * j / temp_2) * Inc_Fac + x;
	   temp_Y = Inc_Fac * (cos(_angle) * j/ temp_2) + y;
           if((scans[i]) > ((float)j / temp_2))
	   {
	     canvasMutex.lock();//unmutes the canvas plotting
             int temp_X2 = temp_X;//setting up temp ints
	     int temp_Y2 =temp_Y;
	     temp_X2 += canvas.rows / 2;//setting temp_X2 to the canvas row
    	     temp_Y2 += canvas.cols / 2;//setting temp_Y2 to the canvas col
	     char t_Cell = canvas.at<char>(temp_X2, temp_Y2);
	     canvasMutex.unlock();
             if(t_Cell == CELL_ROBOT)
	     {
	       //if the cell being scanned comes up as the cell that 
	       plot((int)temp_X,(int)temp_Y, CELL_ROBOT);
	     }
	     else
	     {
 	       //free space 
	       plot((int)temp_X,(int)temp_Y,	
	       CELL_FREE);		
	     }
	   }
           else if((scans[i] )< ((float)j /temp_2) && (scans[i] + (1 / (1.5 *Inc_Fac))) > ((float)j / temp_2))
	   {
             //if the cell being scanned comes up the cell is marked as occupoed
	     plot(temp_X, temp_Y, CELL_OCCUPIED);
	   }
	}
      }
    }
	 
  };
  
  
  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = VFACTOR*-msg->pose.pose.position.y;
    y = VFACTOR*msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation);
	  odom=1;  //acknowledges odometry is initialized
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove following demo code and make robot move around the environment
	if(odom == 1)
	{
      		plot(x, y, CELL_ROBOT);
		if(time.toSec() <= ros::Time::now().toSec() - 30)
		{
			saveSnapshot();
			time = ros::Time::now();
		}
	}
      
      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      //commenting out the line below because file cannot be generated on actual robot
      ros::spinOnce();
      key = cv::waitKey(1000/SPIN_RATE_HZ); //used so no bad snapshots
      if (key == 'x' || key == 'X') 
      {
        break;
      } 
      else if (key == ' ') 
      {
        saveSnapshot();
      }
    }
    
    break;
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 0.5;
  const static double ROTATE_SPEED_RADPS = M_PI/4;
  
  const static int SPIN_RATE_HZ = 30;
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;
  
  const static double VFACTOR = 10.0;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

  double x;
  double y; 
  double heading;
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robot(n, width, height); // Create new grid mapper object
  robot.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
