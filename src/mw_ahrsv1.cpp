#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <iostream>

using namespace std;

#pragma pack(1)
typedef struct _MwAhrsV1 {
	float AccX;
	float AccY;
	float AccZ;
	float RateRoll;
	float RatePitch;
	float RateYaw;
	float DegRoll;
	float DegPitch;
	float DegYaw;
	char cr;
	char lf;
} MwAhrsV1;

class MwAhrsV1ForROS
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;

	ros::Publisher imu_data_raw_pub_;
	ros::Publisher imu_data_pub_;

	tf::TransformBroadcaster broadcaster_;

	pthread_mutex_t lock_;

	std::string parent_frame_id_;
	std::string frame_id_;
	double linear_acceleration_stddev_;		// need check.
	double angular_velocity_stddev_;		// need check.

	//Define constants
	const char* COMM_PORT = "/dev/ttyUSB1";
	// 8 bytes ascii floating point * 9 + space * 8 + cr * 1 + lf * 1
	// 72 + 8 + 1 + 1
	const static int PACKET_SIZE = 82;

	//Define global variables
	int fd;
	char data_packet[PACKET_SIZE];
	int count = 0;


public:
	MwAhrsV1ForROS(std::string port = "/dev/ttyUSB1", int baud_rate = 115200)
		: nh_priv_("~")
	{
		// dependent on user device
		nh_priv_.setParam("port", port);
		nh_priv_.setParam("baud_rate", baud_rate);
		
		// default frame id
		nh_priv_.param("frame_id", frame_id_, std::string("imu_link"));
		
		// for testing the tf
		nh_priv_.param("parent_frame_id_", parent_frame_id_, std::string("base_link"));
		
		// publisher for streaming
		imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
		imu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
	}

	~MwAhrsV1ForROS()
	{}

	bool initialize()
	{
		if(-1 == (fd = open(COMM_PORT, O_RDWR)))
		{
			cout << "Error opening port \n";
			cout << "Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB0 115200 raw\n";
			cout << "You may need to have ROOT access";
			return false;
		}

		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));

		newtio.c_cflag = B115200;
		newtio.c_cflag |= CS8;
		newtio.c_cflag |= CLOCAL;
		newtio.c_cflag |= CREAD;
		newtio.c_iflag = 0;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		newtio.c_cc[VTIME] = 0; 
		newtio.c_cc[VMIN] = 1; 

		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);

		cout << "MW-AHRSv1 communication port is ready\n";

		lock_ = PTHREAD_MUTEX_INITIALIZER;

		return true;
	}

	void closeSensor()
	{
		close(fd);
		cout << "Closing MW-AHRSv1 Sensor" << endl;
	}

	bool receiveData()
	{
		short header;
		short check_sum;
		
		// pthread_mutex_lock(&lock_);
	
		// memset(data_packet, '\0', sizeof(data_packet));
		int recv_size = read(fd, data_packet, PACKET_SIZE);
		cout << "recv_size: " << recv_size << endl;
		if (PACKET_SIZE != recv_size) {
			cout << "Receive Fail !!!" << endl;
			return false;
		}

		// cr + lf check
		// cout << "bytes " << std::hex << (0xff & data_packet[sizeof(data_packet)-1]) << " " << (0xff & data_packet[sizeof(data_packet)-2]) << endl;

		// cout << data_packet << endl;
		// cout << "bytes : " << std::hex << (0xff & data_packet[80]) << " " << (0xff & data_packet[81]) << endl;
		if (!(data_packet[80] == 0x0D && data_packet[81] == 0x0A)) {
			cout << "EOL is wrong" << endl;
			return false;
		}
		
		publishTopic();

		// pthread_mutex_unlock(&lock_);

		return true;
	}

	void publishTopic()
	{
		MwAhrsV1 mwAhrsV1;
		// memcpy(&mwAhrsV1, data_packet, sizeof(MwAhrsV1));
		sscanf(data_packet, "%8f %8f %8f %8f %8f %8f %8f %8f %8f%c%c", &mwAhrsV1.AccX, &mwAhrsV1.AccY, &mwAhrsV1.AccZ, 
																		&mwAhrsV1.RateRoll, &mwAhrsV1.RatePitch, &mwAhrsV1.RateYaw, 
																		&mwAhrsV1.DegRoll, &mwAhrsV1.DegPitch, &mwAhrsV1.DegYaw,
																		&mwAhrsV1.cr, &mwAhrsV1.lf);

		// cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.AccX << " " << setw(8) << mwAhrsV1.AccY << " " << setw(8) << mwAhrsV1.AccZ << endl;
		// cout << "Rates [deg/sec]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.RateRoll << " " << setw(8) << mwAhrsV1.RatePitch << " " << setw(8) << mwAhrsV1.RateYaw << endl;
		// cout << "Attitude  [deg]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.DegRoll << " " << setw(8) << mwAhrsV1.DegPitch << " " << setw(8) << mwAhrsV1.DegYaw << endl;

		cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.AccX << " " << setw(8) << mwAhrsV1.AccY << " " << setw(8) << mwAhrsV1.AccZ << " ";
		cout << "Rates [deg/sec]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.RateRoll << " " << setw(8) << mwAhrsV1.RatePitch << " " << setw(8) << mwAhrsV1.RateYaw << " ";
		cout << "Attitude  [deg]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.DegRoll << " " << setw(8) << mwAhrsV1.DegPitch << " " << setw(8) << mwAhrsV1.DegYaw << " ";

	
		// Publish ROS msgs.
		sensor_msgs::Imu imu_data_raw_msg;
		sensor_msgs::Imu imu_data_msg;

		// Set covariance value of each measurements.
		imu_data_raw_msg.linear_acceleration_covariance[0] =
		imu_data_raw_msg.linear_acceleration_covariance[4] =
		imu_data_raw_msg.linear_acceleration_covariance[8] =
		imu_data_msg.linear_acceleration_covariance[0] =
		imu_data_msg.linear_acceleration_covariance[4] =
		imu_data_msg.linear_acceleration_covariance[8] = -1;

		imu_data_raw_msg.angular_velocity_covariance[0] =
		imu_data_raw_msg.angular_velocity_covariance[4] =
		imu_data_raw_msg.angular_velocity_covariance[8] =
		imu_data_msg.angular_velocity_covariance[0] =
		imu_data_msg.angular_velocity_covariance[4] =
		imu_data_msg.angular_velocity_covariance[8] = -1;

		imu_data_msg.orientation_covariance[0] =
		imu_data_msg.orientation_covariance[4] =
		imu_data_msg.orientation_covariance[8] = -1;

		static double convertor_d2r = M_PI / 180.0; // for angular_velocity (degree to radian)
		static double convertor_r2d = 180.0 / M_PI; // for easy understanding (radian to degree)

		double roll, pitch, yaw;
		roll = mwAhrsV1.DegRoll * convertor_d2r;
		pitch = mwAhrsV1.DegPitch * convertor_d2r;
		yaw = mwAhrsV1.DegYaw * convertor_d2r;

		// Get Quaternion fro RPY.
		tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

		ros::Time now = ros::Time::now();

		imu_data_raw_msg.header.stamp =
		imu_data_msg.header.stamp = now;
		
		imu_data_raw_msg.header.frame_id =
		imu_data_msg.header.frame_id = frame_id_;

		// orientation
		imu_data_msg.orientation.x = orientation[0];
		imu_data_msg.orientation.y = orientation[1];
		imu_data_msg.orientation.z = orientation[2];
		imu_data_msg.orientation.w = orientation[3];

		// original data used the g unit, convert to m/s^2
		imu_data_raw_msg.linear_acceleration.x =
		imu_data_msg.linear_acceleration.x = 0;
		imu_data_raw_msg.linear_acceleration.y =
		imu_data_msg.linear_acceleration.y = 0;
		imu_data_raw_msg.linear_acceleration.z =
		imu_data_msg.linear_acceleration.z = 0;
		imu_data_raw_msg.linear_acceleration.x =
		imu_data_msg.linear_acceleration.x = mwAhrsV1.AccX;
		imu_data_raw_msg.linear_acceleration.y =
		imu_data_msg.linear_acceleration.y = mwAhrsV1.AccY;
		imu_data_raw_msg.linear_acceleration.z =
		imu_data_msg.linear_acceleration.z = mwAhrsV1.AccZ;

		// imu.gx gy gz.
		// original data used the degree/s unit, convert to radian/s
		imu_data_raw_msg.angular_velocity.x =
		imu_data_msg.angular_velocity.x = 0;
		imu_data_raw_msg.angular_velocity.y =
		imu_data_msg.angular_velocity.y = 0;
		imu_data_raw_msg.angular_velocity.z =
		imu_data_msg.angular_velocity.z = 0;
		imu_data_raw_msg.angular_velocity.x =
		imu_data_msg.angular_velocity.x = mwAhrsV1.RateRoll * convertor_d2r;
		imu_data_raw_msg.angular_velocity.y =
		imu_data_msg.angular_velocity.y = mwAhrsV1.RatePitch * convertor_d2r;
		imu_data_raw_msg.angular_velocity.z =
		imu_data_msg.angular_velocity.z = mwAhrsV1.RateYaw * convertor_d2r;

		// publish the IMU data
		imu_data_raw_pub_.publish(imu_data_raw_msg);
		imu_data_pub_.publish(imu_data_msg);

		// publish tf
		broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
			tf::Vector3(0.0, 0.0, 0.0)),
			ros::Time::now(), parent_frame_id_, frame_id_));
	}

};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mw_ahrsv1");

  std::string port = std::string("/dev/ttyUSB1");
  int baud_rate    = 115200;

  ros::param::get("~port", port);
  ros::param::get("~baud_rate", baud_rate);

  MwAhrsV1ForROS sensor(port, baud_rate);

  if(sensor.initialize() == false)
  {
    ROS_ERROR("Initialize() returns false, please check your devices.\n");
	ROS_ERROR("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB? 115200 raw\n");
	ROS_ERROR("You may need to have ROOT access\n");
    return 0;
  }
  else
  {
    ROS_INFO("MW-AHRSv1 Initialization OK!\n");
  }

  // ros::Rate loop_rate(10);

  while (ros::ok())
  {
	  sensor.receiveData();

	  ros::spinOnce();
  }

  //ros::spin();

  return 0;
}

//------------------------------------------------------------------------------
