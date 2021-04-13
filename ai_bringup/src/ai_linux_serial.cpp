#include <ai_bringup/ai_linux_serial.h>
#include <math.h>

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ai");
boost::system::error_code err;

unsigned char header[2] = {0xA5,0x5a};

const double ROBOT_LENGTH = 0.34;                       //m
const double dt = 0.035;                                //读取脉冲增量的时间间隔
const double ticksPerMeter = 1800;                      //前进一米的脉冲增量   

union receiveData
{
	int d;
	unsigned char data[4];
}encoderLeft,encoderRight;


void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
	
}


void writeSpeed(double RobotV,double RobotYawRate)
{
	//期望转向,期望转向角度,期望挡位
	unsigned char TargetSpeed = {0};
	unsigned char TargetStrAngle = {0x3c};
	unsigned char buf[11] = {0};
	
	
	          
	//直行
	if(RobotYawRate == 0)
	{
		TargetStrAngle = 0x3c;
		TargetSpeed =  RobotV*dt*ticksPerMeter;	
	}
	
	//左转
	else if(RobotYawRate > 0)
	{
		TargetSpeed = RobotV*dt*ticksPerMeter;
		TargetStrAngle = 0x3c + atan2(RobotYawRate * ROBOT_LENGTH , RobotV)*57.3*490/(60*14);;
		
	}
	
	//右转
	else
	{
		TargetSpeed = RobotV*dt*ticksPerMeter;
		TargetStrAngle = 0x3c - atan2(-RobotYawRate * ROBOT_LENGTH , RobotV)*57.3*490/(60*14);
	}
	
	
	
	buf[0] = header[0];
	buf[1] = header[1];
	buf[2] = 0x06;
	buf[3] = 0x00;
	buf[4] = TargetStrAngle;                //angle
	buf[5] = TargetSpeed;                //linear_speed
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = 0x00;
	buf[9] = 0x00;
	buf[10] = 0x00;
	
	//发送数据
	boost::asio::write(sp, boost::asio::buffer(buf));
	
}


bool readSpeed(double &vx,double &vth,double &delta_Distance,double &delta_th)
{
	unsigned char buf[60] = {0};
	sp.read_some(buffer(buf, 60));
	
	if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        //ROS_ERROR("Received message header error!");
        return false;
    }
	

    
	 for(int i = 0; i < 4; i++)
    {
        encoderLeft.data[i]  =  buf[i + 3];       //buf[3] -buf[6]
        encoderRight.data[i] =  buf[i + 7];     //buf[7]- buf[10]
       
    }
    
    //计算得到线速度、角速度
    encoderRight.d = -encoderRight.d;
    double delta_Encode = (encoderLeft.d + encoderRight.d)/2;
    delta_Distance = delta_Encode/ticksPerMeter;             
    delta_th = (encoderRight.d - encoderLeft.d)*2*3.1415926/4477;
    
    vx = delta_Distance/dt;              //m/s
    vth = delta_th/dt;                   //rad/s
    
    
    //ROS_INFO("vx:%f\n",vx);
    //ROS_INFO("vth:%f\n",vth);
}
