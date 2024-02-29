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
#include <queue>

#include <serial/serial.h>

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

#define ACC_X_IDX 0
#define SP_ACC_X_IDX 8
#define ACC_Y_IDX 9
#define SP_ACC_Y_IDX 17
#define ACC_Z_IDX 18
#define SP_ACC_Z_IDX 26
#define RATE_X_IDX 27
#define SP_RATE_X_IDX 35
#define RATE_Y_IDX 36
#define SP_RATE_Y_IDX 44
#define RATE_Z_IDX 45
#define SP_RATE_Z_IDX 53
#define DEG_X_IDX 54
#define SP_DEG_X_IDX 62
#define DEG_Y_IDX 63
#define SP_DEG_Y_IDX 71
#define DEG_Z_IDX 72
#define CR_IDX 80
#define LF_IDX 81

#define FIRST_CR_SIZE 1
#define FIRST_LF_SIZE 1
#define ACC_X_SIZE 8
#define SP_ACC_X_SIZE 1
#define ACC_Y_SIZE 8
#define SP_ACC_Y_SIZE 1
#define ACC_Z_SIZE 8
#define SP_ACC_Z_SIZE 1
#define RATE_X_SIZE 8
#define SP_RATE_X_SIZE 1
#define RATE_Y_SIZE 8
#define SP_RATE_Y_SIZE 1
#define RATE_Z_SIZE 8
#define SP_RATE_Z_SIZE 1
#define DEG_X_SIZE 8
#define SP_DEG_X_SIZE 1
#define DEG_Y_SIZE 8
#define SP_DEG_Y_SIZE 1
#define DEG_Z_SIZE 8
#define CR_SIZE 1
#define LF_SIZE 1

#define RX_CR_VAL '\r'	// 0x0D
#define RX_LF_VAL '\n'	// 0x0a
#define RX_SP_VAL ' '	// 0x20

#define RX_SIZE 82

std::string node_name;

class MwAhrsV1ForROS
{
private:
	ros::NodeHandle nh_;

	ros::Publisher imu_data_pub_;

	pthread_mutex_t lock_;

	std::string serial_port_;
	int baud_rate_;
	queue<unsigned char> que;

	std::string frame_id_;
	double linear_acceleration_stddev_;		// need check.
	double angular_velocity_stddev_;		// need check.


	//Define global variables
	serial::Serial *ser;
	unsigned char rx_packet[BUFSIZ];
	int count = 0;


public:
	MwAhrsV1ForROS(std::string serial_port, int baud_rate)
		: nh_("~")
	{
		// dependent on user device
		nh_.setParam("serial_port", serial_port);
		nh_.setParam("baud_rate", baud_rate);
		
		serial_port_ = serial_port;
		baud_rate_ = baud_rate;
		this->ser = new serial::Serial();
		
		// default frame id
		nh_.param("frame_id", frame_id_, std::string("imu_link"));
		
		// publisher for streaming
		imu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1);
	}

	~MwAhrsV1ForROS()
	{}

	bool initialize()
	{
		const char* COMM_PORT = serial_port_.c_str();

		ser->setPort(serial_port_);
		ser->setBaudrate(baud_rate_);
		#define SERIAL_TIMEOUT_MS 3000
		serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
		ser->setTimeout(to);

		ser->open();

		if (!ser->isOpen()) {
			printf("error opening port[%s] baudrate[%d]\n", COMM_PORT, baud_rate_);
			printf("you may need to have ROOT access\n");
			return false;
		}

		ser->flush();

		cout << "MW-AHRSv1 communication port is ready\n";

		lock_ = PTHREAD_MUTEX_INITIALIZER;

		return true;
	}

	void closeSensor()
	{
		ser->close();
		cout << "Closing MW-AHRSv1 Sensor" << endl;
	}

	bool receiveData()
	{
		#if 1
		static int rx_size;

		memset(rx_packet, '\0', sizeof(rx_packet));

		rx_size = ser->available();
		if (rx_size) {
			rx_size = ser->read(rx_packet, rx_size);
		}

		for (int i=0; i<rx_size; i++) {
			que.push(rx_packet[i]);
		}

		return true;
		#else
		static int err_cnt = 0;
		short header;
		short check_sum;
		
		// pthread_mutex_lock(&lock_);
	
		// memset(rx_packet, '\0', sizeof(rx_packet));
		int recv_size = read(fd, rx_packet, PACKET_SIZE);
		#if 0
		cout << "recv_size: " << recv_size << endl;
		if (PACKET_SIZE != recv_size) {
			cout << "Receive Fail !!!" << endl;
			return false;
		}
		#endif

		// cr + lf check
		// cout << "bytes " << std::hex << (0xff & rx_packet[sizeof(rx_packet)-1]) << " " << (0xff & rx_packet[sizeof(rx_packet)-2]) << endl;

		// cout << rx_packet << endl;
		// cout << "bytes : " << std::hex << (0xff & rx_packet[80]) << " " << (0xff & rx_packet[81]) << endl;
		if (!(rx_packet[80] == 0x0D && rx_packet[81] == 0x0A)) {
			#if 0
			cout << "EOL is wrong, #" << ++err_cnt << endl;
			#endif

			return false;
		}
		
		publishTopic();

		// pthread_mutex_unlock(&lock_);

		return true;
		#endif
	}

	enum STATE {FIRST_CR, FIRST_LF,
		ACC_X, SP_ACC_X, ACC_Y, SP_ACC_Y, ACC_Z, SP_ACC_Z,
		RATE_X, SP_RATE_X, RATE_Y, SP_RATE_Y, RATE_Z, SP_RATE_Z,
		DEG_X, SP_DEG_X, DEG_Y, SP_DEG_Y, DEG_Z, SP_DEG_Z, 
		CR, LF,
		OK};
	bool parseData() {
		static int state = FIRST_CR;
		static unsigned char packet[RX_SIZE] = {'\0', };
		int rx_size;
		short header;
		short check_sum;
		static int count = 0;

		#if 1
		switch (state) {
			case FIRST_CR:
				if (que.size() >= FIRST_CR_SIZE) {
					if (que.front() == RX_CR_VAL) {
						state = FIRST_LF;
					} else {
						printf("%s : FIRST_CR not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case FIRST_LF:
				if (que.size() >= FIRST_LF_SIZE) {
					if (que.front() == RX_LF_VAL) {
						state = ACC_X;
					} else {
						printf("%s : FIRST_LF not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case ACC_X:
				// size check
				if (que.size() >= ACC_X_SIZE) {
					for (int i=0; i<ACC_X_SIZE; i++) {
						packet[ACC_X_IDX+i] = que.front();
						que.pop();
					}

					state = SP_ACC_X;
				}
				break;
			case SP_ACC_X:
				if (que.size() >= SP_ACC_X_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_ACC_X_IDX] = que.front();
						state = ACC_Y;
					} else {
						printf("%s : SP_ACC_X not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case ACC_Y:
				// size check
				if (que.size() >= ACC_Y_SIZE) {
					for (int i=0; i<ACC_Y_SIZE; i++) {
						packet[ACC_Y_IDX+i] = que.front();
						que.pop();
					}

					state = SP_ACC_Y;
				}
				break;
			case SP_ACC_Y:
				if (que.size() >= SP_ACC_Y_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_ACC_Y_IDX] = que.front();
						state = ACC_Z;
					} else {
						printf("%s : SP_ACC_Y not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case ACC_Z:
				// size check
				if (que.size() >= ACC_Z_SIZE) {
					for (int i=0; i<ACC_Z_SIZE; i++) {
						packet[ACC_Z_IDX+i] = que.front();
						que.pop();
					}

					state = SP_ACC_Z;
				}
				break;
			case SP_ACC_Z:
				if (que.size() >= SP_ACC_Z_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_ACC_Z_IDX] = que.front();
						state = RATE_X;
					} else {
						printf("%s : SP_ACC_Z not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case RATE_X:
				// size check
				if (que.size() >= RATE_X_SIZE) {
					for (int i=0; i<RATE_X_SIZE; i++) {
						packet[RATE_X_IDX+i] = que.front();
						que.pop();
					}

					state = SP_RATE_X;
				}
				break;
			case SP_RATE_X:
				if (que.size() >= SP_RATE_X_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_RATE_X_IDX] = que.front();
						state = RATE_Y;
					} else {
						printf("%s : SP_RATE_X not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case RATE_Y:
				// size check
				if (que.size() >= RATE_Y_SIZE) {
					for (int i=0; i<RATE_Y_SIZE; i++) {
						packet[RATE_Y_IDX+i] = que.front();
						que.pop();
					}

					state = SP_RATE_Y;
				}
				break;
			case SP_RATE_Y:
				if (que.size() >= SP_RATE_Y_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_RATE_Y_IDX] = que.front();
						state = RATE_Z;
					} else {
						printf("%s : SP_RATE_Y not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case RATE_Z:
				// size check
				if (que.size() >= RATE_Z_SIZE) {
					for (int i=0; i<RATE_Z_SIZE; i++) {
						packet[RATE_Z_IDX+i] = que.front();
						que.pop();
					}

					state = SP_RATE_Z;
				}
				break;
			case SP_RATE_Z:
				if (que.size() >= SP_RATE_Z_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_RATE_Z_IDX] = que.front();
						state = DEG_X;
					} else {
						printf("%s : SP_RATE_Z not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case DEG_X:
				// size check
				if (que.size() >= DEG_X_SIZE) {
					for (int i=0; i<DEG_X_SIZE; i++) {
						packet[DEG_X_IDX+i] = que.front();
						que.pop();
					}

					state = SP_DEG_X;
				}
				break;
			case SP_DEG_X:
				if (que.size() >= SP_DEG_X_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_DEG_X_IDX] = que.front();
						state = DEG_Y;
					} else {
						printf("%s : SP_DEG_X not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case DEG_Y:
				// size check
				if (que.size() >= DEG_Y_SIZE) {
					for (int i=0; i<DEG_Y_SIZE; i++) {
						packet[DEG_Y_IDX+i] = que.front();
						que.pop();
					}

					state = SP_DEG_Y;
				}
				break;
			case SP_DEG_Y:
				if (que.size() >= SP_DEG_Y_SIZE) {
					if (que.front() == RX_SP_VAL) {
						packet[SP_DEG_Y_IDX] = que.front();
						state = DEG_Z;
					} else {
						printf("%s : SP_DEG_Y not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case DEG_Z:
				// size check
				if (que.size() >= DEG_Z_SIZE) {
					for (int i=0; i<DEG_Z_SIZE; i++) {
						packet[DEG_Z_IDX+i] = que.front();
						que.pop();
					}

					state = CR;
				}
				break;
			case CR:
				if (que.size() >= CR_SIZE) {
					if (que.front() == RX_CR_VAL) {
						packet[CR_IDX] = que.front();
						state = LF;
					} else {
						printf("%s : CR not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case LF:
				if (que.size() >= LF_SIZE) {
					if (que.front() == RX_LF_VAL) {
						packet[LF_IDX] = que.front();
						state = OK;
					} else {
						printf("%s : LF not Match(%d)\n", node_name.c_str(), count++);
						state = FIRST_CR;
					}
					que.pop();
				}
				break;
			case OK:
				publishTopic(packet);

				memset(packet, '\0', RX_SIZE);
				
				state = ACC_X;

				break;
			default:
				state = FIRST_CR;

				break;
		}
		#else
		switch (state) {
			case ADDRESS:
				// size check
				if (que.size() >= RX_ID_SIZE) {
					// valid check
					if (que.front() == RX_ID_VAL) {
						packet[RX_ID_IDX] = que.front();

						state = FUNCTION;
					} else {
						printf("%s : ID not Match(%d)\n", node_name.c_str(), count++);
						state = ADDRESS;
					}
					que.pop();
				}
				break;
			case FUNCTION:
				// size check
				if (que.size() >= RX_FUNC_SIZE) {
					// valid check
					if (que.front() == RX_FUNC_VAL) {
						packet[RX_FUNC_IDX] = que.front();

						state = COUNT;
					} else {
						printf("%s : FUNCTION not Match(%d): 0x%02x \n", node_name.c_str(), count++, que.front());
						state = ADDRESS;
					}
					que.pop();
				}
				break;
			case COUNT:
				// size check
				if (que.size() >= RX_LEN_SIZE) {
					// valid check
					if (que.front() == RX_LEN_VAL) {
						packet[RX_LEN_IDX] = que.front();

						state = VALUE;
					} else {
						printf("%s : LENGTH not Match(%d)\n", node_name.c_str(), count++);
						state = ADDRESS;
					}
					que.pop();
				}
				break;
			case VALUE:
				// size check
				if (que.size() >= RX_VAL_SIZE) {
					for (int i=0; i<RX_VAL_SIZE; i++) {
						packet[RX_VAL_IDX+i] = que.front();
						que.pop();
					}

					state = CRC;
				}
				break;
			case CRC:
				// size check
				if (que.size() >= RX_CRC_SIZE) {
					for (int i=0; i<RX_CRC_SIZE; i++) {
						packet[RX_CRC_IDX+i] = que.front();
						que.pop();
					}

					unsigned short crc16 = CRC16(packet, RX_SIZE-RX_CRC_SIZE);
					unsigned char crc16hi = *(((char*)&crc16)+0);
					unsigned char crc16lo = *(((char*)&crc16)+1);

					if (!(crc16hi == packet[RX_SIZE-2] && crc16lo == packet[RX_SIZE-1])) {
						cout << node_name << " : CRC Not Match !!!" << endl;
						memset(packet, '\0', RX_SIZE);
					
						state = ADDRESS;

						return false;
					} else {
						state = OK;
					}
				}

				break;
			case OK:
				publishTopic(packet);

				memset(packet, '\0', RX_SIZE);
				
				state = ADDRESS;

				return true;

				break;
			default:
				state = ADDRESS;

				break;
		}
		#endif
		
		return false;
	}

	void publishTopic(unsigned char* packet)
	{
		MwAhrsV1 mwAhrsV1;
		// memcpy(&mwAhrsV1, packet, sizeof(MwAhrsV1));
		#if 0
		for (int i=0; i<sizeof(packet); i++) {
			printf("%02x", packet[i]);
		}
		printf("\n");
		#endif 
		sscanf((char*)packet, "%8f %8f %8f %8f %8f %8f %8f %8f %8f%c%c", &mwAhrsV1.AccX, &mwAhrsV1.AccY, &mwAhrsV1.AccZ, 
																		&mwAhrsV1.RateRoll, &mwAhrsV1.RatePitch, &mwAhrsV1.RateYaw, 
																		&mwAhrsV1.DegRoll, &mwAhrsV1.DegPitch, &mwAhrsV1.DegYaw,
																		&mwAhrsV1.cr, &mwAhrsV1.lf);

		// cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.AccX << " " << setw(8) << mwAhrsV1.AccY << " " << setw(8) << mwAhrsV1.AccZ << endl;
		// cout << "Rates [deg/sec]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.RateRoll << " " << setw(8) << mwAhrsV1.RatePitch << " " << setw(8) << mwAhrsV1.RateYaw << endl;
		// cout << "Attitude  [deg]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.DegRoll << " " << setw(8) << mwAhrsV1.DegPitch << " " << setw(8) << mwAhrsV1.DegYaw << endl;

		#if 0
		cout << "Accel [m/sec^2]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.AccX << " " << setw(8) << mwAhrsV1.AccY << " " << setw(8) << mwAhrsV1.AccZ << " ";
		cout << "Rates [deg/sec]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.RateRoll << " " << setw(8) << mwAhrsV1.RatePitch << " " << setw(8) << mwAhrsV1.RateYaw << " ";
		cout << "Attitude  [deg]: " << fixed << setprecision(3) << setw(8) << mwAhrsV1.DegRoll << " " << setw(8) << mwAhrsV1.DegPitch << " " << setw(8) << mwAhrsV1.DegYaw << " ";
		#endif
	
		// Publish ROS msgs.
		sensor_msgs::Imu imu_data_msg;

		// Set covariance value of each measurements.
		imu_data_msg.linear_acceleration_covariance[0] =
		imu_data_msg.linear_acceleration_covariance[4] =
		imu_data_msg.linear_acceleration_covariance[8] = -1;

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

		imu_data_msg.header.stamp = now;
		
		imu_data_msg.header.frame_id = frame_id_;

		// orientation
		imu_data_msg.orientation.x = orientation[0];
		imu_data_msg.orientation.y = orientation[1];
		imu_data_msg.orientation.z = orientation[2];
		imu_data_msg.orientation.w = orientation[3];

		// original data used the g unit, convert to m/s^2
		imu_data_msg.linear_acceleration.x = 0;
		imu_data_msg.linear_acceleration.y = 0;
		imu_data_msg.linear_acceleration.z = 0;
		imu_data_msg.linear_acceleration.x = mwAhrsV1.AccX;
		imu_data_msg.linear_acceleration.y = mwAhrsV1.AccY;
		imu_data_msg.linear_acceleration.z = mwAhrsV1.AccZ;

		// imu.gx gy gz.
		// original data used the degree/s unit, convert to radian/s
		imu_data_msg.angular_velocity.x = 0;
		imu_data_msg.angular_velocity.y = 0;
		imu_data_msg.angular_velocity.z = 0;
		imu_data_msg.angular_velocity.x = mwAhrsV1.RateRoll * convertor_d2r;
		imu_data_msg.angular_velocity.y = mwAhrsV1.RatePitch * convertor_d2r;
		imu_data_msg.angular_velocity.z = mwAhrsV1.RateYaw * convertor_d2r;

		// publish the IMU data
		imu_data_pub_.publish(imu_data_msg);
	}

	int getQueueSize() {
		return que.size();
	}
};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mw_ahrsv1");

  std::string serial_port;
  int baud_rate;

  ros::param::get("~serial_port", serial_port);
  ros::param::get("~baud_rate", baud_rate);
  ros::param::get("~node_name", node_name);

  MwAhrsV1ForROS sensor(serial_port, baud_rate);

  if(sensor.initialize() == false) {
    ROS_ERROR("Initialize() returns false, please check your devices.\n");
	ROS_ERROR("Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB? 115200 raw\n");
	ROS_ERROR("You may need to have ROOT access\n");
    return 0;
  } else {
    ROS_INFO("MW-AHRSv1 Initialization OK!\n");
  }

  ros::Rate r(1000);

#define STEP_TIME 1.0
  double time_cur = ros::Time::now().toSec();
  double time_pre = time_cur;
  double time_diff;

  bool result;

  while (ros::ok()) {
    time_cur = ros::Time::now().toSec();
    time_diff = time_cur - time_pre;
    if ( time_diff > STEP_TIME ) {
    }

	sensor.receiveData(); // stack smashing detected(because of BUFSIZ, read(BUFSIZ))
	while( sensor.getQueueSize() >= RX_SIZE ) {
		result = sensor.parseData();

		if (result) {
			break;
		}
	}
	// printf("%s : size(%d) \n", node_name.c_str(), sensor.getQueueSize());

	ros::spinOnce();

	r.sleep();
  }

  sensor.closeSensor();

  //ros::spin();

  return 0;
}

//------------------------------------------------------------------------------
