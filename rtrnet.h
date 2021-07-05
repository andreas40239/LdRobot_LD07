#ifndef _R_TR_NET_H
#define _R_TR_NET_H

#include <stdint.h>
#include <vector>
#include <iostream>
#include <array>
#include "cmd_interface_linux.h"
#include "trnet.h"
#include <sensor_msgs/LaserScan.h>

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59/180000)

typedef enum 
{
	PACK_GET_DISTANCE = 0x02,  /*Frame ID of distance data*/
	PACK_STOP = 0x0F,		   /*Frame ID of stop distance data transmission*/
	PACK_GET_COE = 0x12,	   /*Frame ID of Get parameters*/
	PACK_VIDEO_SIZE = 0x15,	   /*Frame ID of Get camera resolution*/
	PACK_CONFIG_ADDRESS = 0x16 /*Frame ID of Configure address*/
}PackageIDTypeDef;

struct PointData
{
	float angle;
	uint16_t distance;
	uint8_t confidence;
	double x;
    double y;
	PointData(float angle, uint16_t distance, uint8_t confidence , double x = 0, double y = 0)
	{
		this->angle = angle;
		this->distance = distance;
		this->confidence = confidence;
		this->x = x;
        this->y = y;
	}
	PointData(){}
	friend std::ostream& operator<<(std::ostream &os , const PointData &data)
    {
        os << data.angle << " "<< data.distance << " " << (int)data.confidence << " "<<data.x << " "<<data.y;
        return  os;
    }
};

class RTRNet 
{
public:
	RTRNet();
	void SendCmd(CmdInterfaceLinux &port, uint8_t address, uint8_t id);
	bool UnpackData(const uint8_t *data, uint32_t len);
	void ResetFrameReady() {frame_ready_ = false;}
	sensor_msgs::LaserScan GetLaserScan() {return output;}
	bool IsParametersReady(void) { return parameters_ready_; }
	bool IsFrameReady(void) { return frame_ready_; } /*Lidar data frame is ready*/

private:
	bool Transform(const TRData *tr_data);			   /*transform raw data to stantard data */
	void TransformSignlePoint(uint16_t dist, int n, uint8_t confidence,std::vector<PointData> &dst);
    std::vector<uint8_t> data_tmp_;
	uint32_t coe_[4];
	uint16_t coe_u_;
	uint16_t coe_v_;
	bool parameters_ready_;
	bool frame_ready_;
	sensor_msgs::LaserScan output;
	void ToLaserscan(std::vector<PointData> src);
};

#endif // _TR_NET_H
