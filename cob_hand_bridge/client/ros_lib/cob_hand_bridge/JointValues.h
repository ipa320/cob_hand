#ifndef _ROS_cob_hand_bridge_JointValues_h
#define _ROS_cob_hand_bridge_JointValues_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_hand_bridge
{

  class JointValues : public ros::Msg
  {
    public:
      int16_t position_cdeg[2];
      int16_t velocity_cgeg[2];
      uint16_t current_100uA[2];

    JointValues():
      position_cdeg(),
      velocity_cgeg(),
      current_100uA()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_position_cdegi;
      u_position_cdegi.real = this->position_cdeg[i];
      *(outbuffer + offset + 0) = (u_position_cdegi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_cdegi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position_cdeg[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_velocity_cgegi;
      u_velocity_cgegi.real = this->velocity_cgeg[i];
      *(outbuffer + offset + 0) = (u_velocity_cgegi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_cgegi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->velocity_cgeg[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->current_100uA[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_100uA[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_100uA[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_position_cdegi;
      u_position_cdegi.base = 0;
      u_position_cdegi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_cdegi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position_cdeg[i] = u_position_cdegi.real;
      offset += sizeof(this->position_cdeg[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_velocity_cgegi;
      u_velocity_cgegi.base = 0;
      u_velocity_cgegi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_cgegi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->velocity_cgeg[i] = u_velocity_cgegi.real;
      offset += sizeof(this->velocity_cgeg[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      this->current_100uA[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->current_100uA[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->current_100uA[i]);
      }
     return offset;
    }

    const char * getType(){ return "cob_hand_bridge/JointValues"; };
    const char * getMD5(){ return "8aab4213f33d6f17f952e56b8cad1628"; };

  };

}
#endif