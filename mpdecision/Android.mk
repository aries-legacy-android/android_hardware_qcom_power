MPDCVS_BOARD_PLATFORM_LIST := msm8660
MPDCVS_BOARD_PLATFORM_LIST += msm8960
MPDCVS_BOARD_PLATFORM_LIST += msm7627a
MPDCVS_BOARD_PLATFORM_LIST += msm8625
MPDCVS_BOARD_PLATFORM_LIST += copper

ifeq ($(call is-board-platform-in-list,$(MPDCVS_BOARD_PLATFORM_LIST)),true)

IS_MSM7627A:=$(strip $(call is-board-platform-in-list,msm7627a))
IS_MSM8625:=$(strip $(call is-board-platform-in-list,msm8625))

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := decision.c

LOCAL_SHARED_LIBRARIES := libcutils
LOCAL_SHARED_LIBRARIES += libutils

LOCAL_MODULE := mpdecision

LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS += \
	-DUSE_ANDROID_LOG \

ifneq ($(IS_MSM7627A),)
LOCAL_CFLAGS += \
	-DUSE_PM2
endif

ifneq ($(IS_MSM8625),)
LOCAL_CFLAGS += \
        -DUSE_PM2
endif

USE_MPCTL_SOCKET := true
ifeq ($(USE_MPCTL_SOCKET),true)
    LOCAL_CFLAGS += -DMPCTL_SERVER
    LOCAL_STATIC_LIBRARIES += libqc-mpctl
    LOCAL_SHARED_LIBRARIES += libdl
endif

include $(BUILD_EXECUTABLE)

endif
