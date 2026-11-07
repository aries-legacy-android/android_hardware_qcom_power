# Thermal daemon for the boards that ship local temperature sensors.
#
# The modem mitigation paths were removed: they need the QMI framework and the
# modem API headers, neither of which is part of this tree.  Local control via
# tsens, bcl and adc is unaffected.
#
# The configuration file is shipped by the device tree, not installed here:
# gingerbread's prebuilt rules ignore the ETC module class.

BOARD_PLATFORM_LIST := msm8660
BOARD_PLATFORM_LIST += msm8960
BOARD_PLATFORM_LIST += msm7627a
BOARD_PLATFORM_LIST += copper

ifeq ($(call is-board-platform-in-list,$(BOARD_PLATFORM_LIST)),true)

IS_MSM8960 := $(strip $(call is-board-platform,msm8960))

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
include $(TOP)/hardware/qcom/common/qcom_headers.mk

# Only the vendor headers: the daemon reads sysfs and unix sockets, and the
# kernel uapi socket headers drop the AF_*/sa_family_t definitions that
# bionic's own copy provides for the rest of the tree.
LOCAL_C_INCLUDES := $(QCOM_COMMON_INC)

LOCAL_SRC_FILES := \
        thermal.c \
        thermal_config.c \
        thermal_monitor.c \
        thermal_actions.c \
        thermal_util.c \
        thermal_server.c \
        thermal_lib_common.c

LOCAL_SHARED_LIBRARIES := libcutils

LOCAL_CFLAGS := -DCONFIG_FILE_DEFAULT='"/system/etc/thermald.conf"'
LOCAL_CFLAGS += -DUSE_ANDROID_LOG

ifneq ($(IS_MSM8960),)
LOCAL_SRC_FILES += tsens-sensor.c adc-sensor.c sensors-8960.c gen-sensor.c bcl.c
LOCAL_SRC_FILES += qmi_ts_stub.c
LOCAL_CFLAGS += -DSENSORS_8960
LOCAL_CFLAGS += -DENABLE_TSENS_INTERRUPT
endif

LOCAL_MODULE := thermald
LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)

endif
