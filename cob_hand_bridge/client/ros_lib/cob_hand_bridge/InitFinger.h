#ifndef _ROS_SERVICE_InitFinger_h
#define _ROS_SERVICE_InitFinger_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_hand_bridge
{

static const char INITFINGER[] = "cob_hand_bridge/InitFinger";

  class InitFingerRequest : public ros::Msg
  {
    public:
      const char* port;
      uint16_t min_pwm0;
      uint16_t min_pwm1;
      uint16_t max_pwm0;
      uint16_t max_pwm1;

    InitFingerRequest():
      port(""),
      min_pwm0(0),
      min_pwm1(0),
      max_pwm0(0),
      max_pwm1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_port = strlen(this->port);
      memcpy(outbuffer + offset, &length_port, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->port, length_port);
      offset += length_port;
      *(outbuffer + offset + 0) = (this->min_pwm0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_pwm0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->min_pwm0);
      *(outbuffer + offset + 0) = (this->min_pwm1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_pwm1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->min_pwm1);
      *(outbuffer + offset + 0) = (this->max_pwm0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_pwm0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->max_pwm0);
      *(outbuffer + offset + 0) = (this->max_pwm1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_pwm1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->max_pwm1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_port;
      memcpy(&length_port, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_port; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_port-1]=0;
      this->port = (char *)(inbuffer + offset-1);
      offset += length_port;
      this->min_pwm0 =  ((uint16_t) (*(inbuffer + offset)));
      this->min_pwm0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->min_pwm0);
      this->min_pwm1 =  ((uint16_t) (*(inbuffer + offset)));
      this->min_pwm1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->min_pwm1);
      this->max_pwm0 =  ((uint16_t) (*(inbuffer + offset)));
      this->max_pwm0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->max_pwm0);
      this->max_pwm1 =  ((uint16_t) (*(inbuffer + offset)));
      this->max_pwm1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->max_pwm1);
     return offset;
    }

    const char * getType(){ return INITFINGER; };
    const char * getMD5(){ return "4b3d29c9d84d56c826054700c9f79709"; };

  };

  class InitFingerResponse : public ros::Msg
  {
    public:
      bool success;

    InitFingerResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return INITFINGER; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class InitFinger {
    public:
    typedef InitFingerRequest Request;
    typedef InitFingerResponse Response;
  };

}
#endif
