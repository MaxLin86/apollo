#!/bin/sh
insmod /lib/modules/$(uname -r)/kernel/drivers/media/v4l2-core/v4l2-common.ko 
insmod /lib/modules/$(uname -r)/kernel/drivers/media/v4l2-core/videodev.ko 
insmod /lib/modules/$(uname -r)/kernel/drivers/media/media.ko 