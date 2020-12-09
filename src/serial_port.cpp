
#include <ros/ros.h>
#include <serial/serial.h> //ROS已經內置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <stdlib.h>
#include <time.h>
#include <string.h>

#define delta 1

using namespace std;

serial::Serial ser; //聲明串口對象

int tx_count = 0;
int tx_len = 6;
int32_t* tx = (int32_t *)malloc((tx_len + 2) * sizeof(int32_t));
int rx_len = 6;
int32_t* indata = (int32_t *)malloc((rx_len + 3) * sizeof(int32_t));
int32_t tmp;
	
int rcv_count = 0;
int error_count = 0;
int success_rate = 0;
int pi_2_ST1_rate = 0;
int reset_count = 0;

// crc only for stm
uint32_t modified_crc32_mpeg_2(uint8_t *data, uint8_t length)
{
    uint8_t i;
    int8_t j = 3;
    //int8_t j = 0;
    uint32_t crc = 0xffffffff;  // Initial value
    int8_t l = length;

    while(length--)
    {	
		crc ^= (uint32_t)(*(j+ data++)) << 24;
        j-=2;
        if(j == -5) j=3;
        for (i = 0; i < 8; ++i)
        {
            if ( crc & 0x80000000 )
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}

//回調函數
void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ser.write(msg->data); //發送串口數據
}

void fromAgent_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    for(int i = 1; i < tx_len; i++){
		tx[i] = msg->data[i];
	}
	tx[tx_len] = modified_crc32_mpeg_2((uint8_t*)tx, 16);
    //tx[0] = msg->data[0];
    //tx[1] = msg->data[1];
    //tx[2] = msg->data[2];
    //tx[3] = msg->data[3];
    //tx[4] = msg->data[0];
    //tx[5] = msg->data[0];
    //tx[6] = msg->data[0];
    //tx[4] = modified_crc32_mpeg_2((uint8_t*)tx, 16);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher write_pub = nh.advertise<std_msgs::String>("write", 1000);
    
    ros::Subscriber from_agent = nh.subscribe("txST1", 1, fromAgent_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::Int32MultiArray>("rxST1", 1);
    ros::Publisher rate_pub = nh.advertise<std_msgs::Int32>("success_rate", 1000);
    
    ros::Publisher pi2ST1_pub = nh.advertise<std_msgs::Int32>("pi2ST1_rate", 1000);

    try
    {
        ser.setPort("/dev/STM1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(12);
        ser.setTimeout(to);
        ser.open();
    }
    
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;                                                                       
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(400);

    //uint32_t tx[mode, R1, R2, R3, crc];
    // mode(hex):
    //  - 1000: set position;   R1=x,       R2=y,       R3=theta
    //  - 2000: run script;     R1=num_of_script
    //  - 300x: speed mode;     R1=speed,   R2=theta,   x=dir(1:fwd, 2:bwd, 0:auto)
    //  - 40xx: pos mode;       R1=x,       R2=y,       R3=theta,       TD=turn_dir(1:L, 2:R, 3:auto),      LD=lin_dir(1:fwd, 2:bwd, 3:auto)   
    //  - 5000: stop;
    //  - 6000: idle, disable motor driver;
    
    std_msgs::Int32MultiArray rx_msg;
    string test;			    
	std_msgs::Int32 success_rate_msg;
    std_msgs::String tx_str;
    std_msgs::Int32 pi_2_ST1_rate_msg;	
	
	tx[0] = 10;
    for(int count = 1; count < tx_len; count++){
		tx[count] = 0;
	}
			
    while (ros::ok())
    {
		bool flag;
        if (ser.available()>=4*(rx_len+2+1))
		{
            test = ser.readline(4*(rx_len+2+1), ">?");
			rx_msg.data.clear();
			for(int i=0; i<rx_len+1+1; i++){
				tmp = (int32_t)test[4*i] + (int32_t)test[4*i+1]*256 +
					(int32_t)test[4*i+2]*65536 +(int32_t)test[4*i+3]*16777216;//+(int32_t)test[4*i+4]*4294967296+(int32_t)test[4*i+5]*1099511627776	
				rx_msg.data.push_back(tmp);
				indata[i] = tmp;
			}
			
			pi_2_ST1_rate = rx_msg.data[3];
			pi_2_ST1_rate_msg.data = pi_2_ST1_rate;
			pi2ST1_pub.publish(pi_2_ST1_rate_msg);
			
			// crc ckeck
			rcv_count ++;
			if (indata[rx_len] == modified_crc32_mpeg_2((uint8_t*)indata, 4*rx_len)) {
                read_pub.publish(rx_msg);
                if(indata[rx_len+1] == 255) flag = 1;
                else if(indata[rx_len+1] == 0) flag = 0;
			}
			else {
				//read_pub.publish(rx_msg);
			    error_count++;
			}
			
			if (rcv_count >= 100) { //50
			    success_rate = (int)(100*(rcv_count-error_count)/rcv_count);
                success_rate_msg.data = success_rate;
                rate_pub.publish(success_rate_msg);
                
			    rcv_count = 0;
			    error_count = 0;
			   
			}
        }
        
	// serial transmit 
	if(flag == 0){
        if (++tx_count %2 == 0) {
            tx[tx_len] = modified_crc32_mpeg_2((uint8_t*)tx, 4*tx_len);
            tx_str.data = ser.write((const uint8_t*)tx, 4 *tx_len + 7 );
            write_pub.publish(tx_str);
        }
	}
        
       
        ros::spinOnce();
        loop_rate.sleep();
    }
}
