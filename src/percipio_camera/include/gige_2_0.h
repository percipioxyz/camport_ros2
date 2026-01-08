#pragma once

#include "percipio_device.h"

namespace percipio_camera {

class GigE_2_0 : public GigEBase {
public:
    GigE_2_0(const TY_DEV_HANDLE dev);
    ~GigE_2_0() {};

    virtual TY_STATUS init();
    virtual TY_STATUS dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes);
    virtual TY_STATUS image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode);

    virtual void device_load_parameters();

    virtual TY_STATUS stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data);

    virtual void depth_stream_distortion_check(bool& has_undist_data);

    virtual TY_STATUS depth_scale_unit_init(float& dept_scale_unit);

    virtual TY_STATUS color_stream_aec_roi_init(const TY_AEC_ROI_PARAM& ROI);

    virtual TY_STATUS set_tof_depth_quality(const std::string& qua);
    virtual TY_STATUS set_tof_modulation_threshold(int threshold);
    virtual TY_STATUS set_tof_jitter_threshold(int threshold);
    virtual TY_STATUS set_tof_filter_threshold(int threshold);
    virtual TY_STATUS set_tof_channel(int chan);
    virtual TY_STATUS set_tof_HDR_ratio(int ratio);

private:
    TY_AEC_ROI_PARAM aec_roi;
    bool load_default_parameter();
    bool parameter_init(const std::string& source, const std::string& feat, const std::string& str_val);
};

}