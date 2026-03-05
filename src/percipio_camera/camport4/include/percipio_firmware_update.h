#ifndef _PERCIPIO_FIRMWARE_UPDATE_
#define _PERCIPIO_FIRMWARE_UPDATE_
#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
    #ifdef PERCIPIO_FW_UPDATE_EXPORTS
        #define PERCIPIO_FW_UPDATE_API __declspec(dllexport)
    #else
        #define PERCIPIO_FW_UPDATE_API __declspec(dllimport)
    #endif
#else
    #define PERCIPIO_FW_UPDATE_API
#endif

enum ty_dev_type {
    ty_dev_genicam = 0,
    ty_dev_usb3vision,
    ty_dev_invalid = -1
};


enum FW_UPDATE_STATUS
{
    UPDATE_STATUS_OK = 0,

    UPDATE_STATUS_FW_UPGRADE_FALED           = -1001,

    UPDATE_STATUS_CFG_UPGRADE_FALED          = -1002,

    UPDATE_STATUS_BACKUP_FAILED              = -1003,
    
    UPDATE_STATUS_RESTORE_FAILED             = -1004,

    UPDATE_STATUS_U3V_CMD_ERROR              = -1005,
    
    UPDATE_STATUS_SN_UPGRADE_FALED           = -1006,
};

struct ty_dev_base_info {
    ty_dev_type type;       //Device interface type
    char        sn[32];     //Device Serial Number
    char        ip[32];     //Device IP Address (Note: Only valid for network interface cameras) 
};

typedef void (*UpdateLogCallback)(const char* dev, const char* log);

PERCIPIO_FW_UPDATE_API void upgrade_get_device_count(uint32_t* count);

PERCIPIO_FW_UPDATE_API void upgrade_get_device_list(ty_dev_base_info* devices, uint32_t count, uint32_t* filledCount);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS upgrade_device_firmware_from_file(const ty_dev_base_info* dev, const char* file, const bool force = false);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS upgrade_device_configuration_from_file(const ty_dev_base_info* dev, const char* file);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS upgrade_device_SN_from_file(const ty_dev_base_info* dev, const char* file);

PERCIPIO_FW_UPDATE_API const char* get_last_error(const ty_dev_base_info* dev);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS export_net_device_user_data_to_file(const ty_dev_base_info* dev, const char* file);

PERCIPIO_FW_UPDATE_API FW_UPDATE_STATUS restore_net_device_user_data_from_file(const ty_dev_base_info* dev, const char* file, const bool force = false);

PERCIPIO_FW_UPDATE_API const char* device_run_cmd(const ty_dev_base_info* dev, const char* cmd);

PERCIPIO_FW_UPDATE_API void set_global_log_callback(UpdateLogCallback cb);

#ifdef __cplusplus
}
#endif
#endif
