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

    virtual TY_STATUS work_mode_init(percipio_dev_workmode mode, const bool fix_rate, const float rate);

    virtual void device_load_parameters();

    virtual TY_STATUS stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data);

    virtual TY_STATUS EnableHwIRUndistortion();

    virtual TY_STATUS getIRLensType(TYLensOpticalType& type);
    virtual TY_STATUS getIRRectificationMode(percipio_rectification_mode& mode);
    virtual TY_STATUS getLeftIRRotation(TY_CAMERA_ROTATION& rotation);
    virtual TY_STATUS getLeftIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr);
    virtual TY_STATUS getRightIRRotation(TY_CAMERA_ROTATION& rotation);
    virtual TY_STATUS getRightIRRectifiedIntr(TY_CAMERA_INTRINSIC& rectified_intr);

    virtual void depth_stream_distortion_check(bool& has_undist_data);

    virtual TY_STATUS depth_scale_unit_init(float& dept_scale_unit);

    virtual TY_STATUS color_stream_aec_roi_init(const TY_AEC_ROI_PARAM& ROI);

    virtual TY_STATUS send_soft_trigger_signal();

    virtual void reset();
private:
    TY_AEC_ROI_PARAM aec_roi;
    bool load_default_parameter();
    bool parameter_init(const std::string& source, const std::string& feat, const std::string& str_val);
};

}