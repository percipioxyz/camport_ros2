#pragma once

#include "percipio_device.h"

namespace percipio_camera {

class GigE_2_1 : public GigEBase {
public:
    GigE_2_1(const TY_DEV_HANDLE dev);
    ~GigE_2_1() {};

    virtual TY_STATUS init();
    virtual TY_STATUS dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes);
    virtual TY_STATUS image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode);

    virtual TY_STATUS work_mode_init(percipio_dev_workmode mode, const bool fix_rate, const float rate);

    virtual void device_load_parameters();

    virtual TY_STATUS stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data);

    virtual void depth_stream_distortion_check(bool& has_undist_data);

    virtual TY_STATUS depth_scale_unit_init(float& dept_scale_unit);

    virtual TY_STATUS color_stream_aec_roi_init(const TY_AEC_ROI_PARAM& ROI);

    virtual TY_STATUS send_soft_trigger_signal();

private:
    TY_STATUS fix_device_frame_rate(float& rate);
    TY_STATUS fix_device_frame_rate_in_soft_trigger_mode(const float rate);
};

}