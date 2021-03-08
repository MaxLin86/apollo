root权限终端输入：
1、make install
2、make load
3、检测驱动是否加载
输入命令 ls /dev/video0 到video3可以看到下面信息说明设备驱动安装正确
root@yu-desktop:/home/yu# ls /dev/video0
/dev/video0
root@yu-desktop:/home/yu# ls /dev/video1
/dev/video1
root@yu-desktop:/home/yu# ls /dev/video2
/dev/video2
root@yu-desktop:/home/yu# ls /dev/video3
/dev/video3


4、安装测试软件mplayer

   command: sudo apt-get update
   command: sudo apt-get install mplayer
6.运行测试软件
在终端中输入

测试第一路   device=/dev/video0定义选择视频卡设备通道   width=1920:height=1080:fps=25定义视频流尺寸大小和帧率

mplayer tv:// -tv driver=v4l2:device=/dev/video0:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2

测试第二路
mplayer tv:// -tv driver=v4l2:device=/dev/video1:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2

测试第三路
mplayer tv:// -tv driver=v4l2:device=/dev/video2:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2

测试第四路
mplayer tv:// -tv driver=v4l2:device=/dev/video3:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2


7、安装测试软件vlc
   command: sudo apt-get update
  command: sudo apt-get install vlc
8、适用内核版本
System-Product-Name:/home/yu# uname -a
Linux yu-System-Product-Name 4.4.32-apollo-2-RT #2 SMP PREEMPT RT Wed Sep 20 11:45:34 CST 2017 x86_64 x86_64 x86_64 GNU/Linux




9、采集卡属性设置调用接口和参考代码具体参考ioc_input.c文件