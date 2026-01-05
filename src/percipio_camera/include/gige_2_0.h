#pragma once

#include "percipio_device.h"

namespace percipio_camera {

class GigE_2_0 : public GigEBase {
  public:
    GigE_2_0(const TY_DEV_HANDLE dev);
    ~GigE_2_0() {};

};

}