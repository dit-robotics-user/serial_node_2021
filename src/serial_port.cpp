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
int32_t* tx = (int32_t *)malloc((tx_len + 1) * sizeof(int32_t));
int rx_len = 6;
int32_t* indata = (int32_t *)malloc((rx_len + 3) * sizeof(int32_t));
int32_t tmp;
	
int rcv_count = 0;
int error_count = 0;
int success_rate = 0;
int reset_count = 0;
bool flag;

// crc only for stm
uint32_t modified_crc32_mpeg_2(uint8_t *data, uint8_t length){
    uint8_t i;
    int8_t j = 3;
    uint32_t crc = 0xffffffff;  // Initial value
    int8_t l = length;

    while(length--){
        crc ^= (uint32_t)(*(j+ data++)) << 24;
        j-=2;
        if(j == -5) j=3;
        for (i = 0; i < 8; ++i){
            if ( crc & 0x80000000 )
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void write_callback(const std_msgs::String::ConstPtr &msg){
    // ser.write(msg->data); //發送串口數據
}

void fromAgent_callback(const std_msgs::Int32MultiArray::ConstPtr &msg){
    for(int i = 1; i < tx_len; i++){
        tx[i] = msg->data[i-1];
    }
    tx[0] = 0x31;//ST1:0x31,ST2:0x32
    tx[tx_len] = modified_crc32_mpeg_2((uint8_t*)tx, 4*tx_len);
    // serial transmit 
    if(ros::ok()){
	if(flag == 0){
	    if(++tx_count % 2 == 0){
		ser.write((const uint8_t*)tx, 4 * tx_len + 4);
            }
	}
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    // ros::Publisher write_pub = nh.advertise<std_msgs::String>("write", 1000);
    ros::Subscriber from_agent = nh.subscribe("txST1", 1, fromAgent_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::Int32MultiArray>("rxST1", 1);
    ros::Publisher rate_pub = nh.advertise<std_msgs::Int32>("success_rate", 1000);

    try{
        ser.setPort("/dev/STM1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(12);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException &e){
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else{
        return -1;
    }
    
    ros::Rate loop_rate(400);
   
    std_msgs::Int32MultiArray rx_msg;
    string test;
    std_msgs::Int32 success_rate_msg;
    std_msgs::String tx_str;

    while(ros::ok()){
        if(ser.available()>=4*(rx_len+2+1)){
            test = ser.readline(4*(rx_len+2+1), ">?");
            rx_msg.data.clear();
            for(int i=0; i<rx_len+1+1; i++){
                tmp = (int32_t)test[4*i] + (int32_t)test[4*i+1]*256 + (int32_t)test[4*i+2]*65536 +(int32_t)test[4*i+3]*16777216;
                indata[i] = tmp;
            }
            // crc ckeck
            rcv_count ++;
            if(indata[rx_len] == modified_crc32_mpeg_2((uint8_t*)indata, 4*rx_len)){
                for(int i = 1; i < rx_len; i++){
                    rx_msg.data.push_back(indata[i]);
                }
                read_pub.publish(rx_msg);
                if(indata[rx_len + 1] == 255) flag = 1;
                else if(indata[rx_len + 1] == 0) flag = 0;
            }
            else{
                error_count++;
            }

            if(rcv_count >= 100){
                success_rate = (int)(100*(rcv_count-error_count)/rcv_count);
                success_rate_msg.data = success_rate;
                rate_pub.publish(success_rate_msg);
                rcv_count = 0;
                error_count = 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
