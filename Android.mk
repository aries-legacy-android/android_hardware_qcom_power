# findleaves stops at the first Android.mk it finds, so the subdirectories
# have to be included explicitly.

LOCAL_PATH := $(call my-dir)

include $(call all-makefiles-under,$(LOCAL_PATH))
