rootȨ���ն����룺
1��make install
2��make load
3����������Ƿ����
�������� ls /dev/video0 ��video3���Կ���������Ϣ˵���豸������װ��ȷ
root@yu-desktop:/home/yu# ls /dev/video0
/dev/video0
root@yu-desktop:/home/yu# ls /dev/video1
/dev/video1
root@yu-desktop:/home/yu# ls /dev/video2
/dev/video2
root@yu-desktop:/home/yu# ls /dev/video3
/dev/video3


4����װ�������mplayer

   command: sudo apt-get update
   command: sudo apt-get install mplayer
6.���в������
���ն�������

���Ե�һ·   device=/dev/video0����ѡ����Ƶ���豸ͨ��   width=1920:height=1080:fps=25������Ƶ���ߴ��С��֡��

mplayer tv:// -tv driver=v4l2:device=/dev/video0:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2

���Եڶ�·
mplayer tv:// -tv driver=v4l2:device=/dev/video1:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2

���Ե���·
mplayer tv:// -tv driver=v4l2:device=/dev/video2:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2

���Ե���·
mplayer tv:// -tv driver=v4l2:device=/dev/video3:input=0:norm=PAL-DK:width=1920:height=1080:fps=25:outfmt=yuy2


7����װ�������vlc
   command: sudo apt-get update
  command: sudo apt-get install vlc
8�������ں˰汾
System-Product-Name:/home/yu# uname -a
Linux yu-System-Product-Name 4.4.32-apollo-2-RT #2 SMP PREEMPT RT Wed Sep 20 11:45:34 CST 2017 x86_64 x86_64 x86_64 GNU/Linux




9���ɼ����������õ��ýӿںͲο��������ο�ioc_input.c�ļ�