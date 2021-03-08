#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>           
#include <fcntl.h>            
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>


#include <asm/types.h>        
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#define V4L2_CID_I2C_IO							( 0x08000000 )
#define V4L2_CID_I2C_IO_PRINT					( 0x08000001 )
#define V4L2_CID_SIGNAL_STATUS					( 0x08000002 )
#define V4L2_CID_SWAP_HOR						( 0x08000003 )
#define V4L2_CID_SWAP_VER						( 0x08000004 )
#define V4L2_CID_GAIN_R							( 0x08000005 )
#define V4L2_CID_GAIN_G							( 0x08000006 )
#define V4L2_CID_GAIN_B							( 0x08000007 )


#define CAMERA_DEVICE "/dev/video0"


#define CMD_GET   1
#define CMD_SET   2
#define PRINT_HELP_EXIT do {printf("\nUsage: %s \n\tcmd:\n\t[gs](get signal status)\n\t[g|s][i](get|set input channel) index(hex) \n\t[g|s][h|v](get|set  horizontal|vertical mirror) index(hex) ]\n\t[g|s][r|g|b](get|set RGB gain value) index(hex) ]\n\n\tcmd example:si 1 (set input channel:1)\n\n", argv[0]); return -1;} while(0)

#define CMD_SIGNAL_STATUS   	1
#define CMD_INPUT   			2
#define CMD_H_MIRROR   			3
#define CMD_V_MIRROR   			4
#define CMD_R_GAIN   			5
#define CMD_G_GAIN   			6
#define CMD_B_GAIN   			7

int main(int argc, char **argv)
{
	struct v4l2_input input;
	struct v4l2_control con;
	
	memset(&input, 0, sizeof(input));
    

    int fd;

	
	int value;
	int cmdGetSet = 0;
	int cmdType = 0;
	
	if(argc >= 2 ){
		if(argv[1][0] == 'g')
			cmdGetSet = CMD_GET;
		else if(argv[1][0] == 's')
			cmdGetSet = CMD_SET;
		else
			PRINT_HELP_EXIT;
		
		switch( argv[1][1] )
		{
			case 's':
				cmdType = CMD_SIGNAL_STATUS;
				break;
			case 'i':
				cmdType = CMD_INPUT;
				break;
			case 'h':
				cmdType = CMD_H_MIRROR;
				break;
			case 'v':
				cmdType = CMD_V_MIRROR;
				break;
			case 'r':
				cmdType = CMD_R_GAIN;
				break;
			case 'g':
				cmdType = CMD_G_GAIN;
				break;
			case 'b':
				cmdType = CMD_B_GAIN;
				break;
			default:
				PRINT_HELP_EXIT;
				break;
		}
		if( (cmdGetSet == CMD_SET ) && (argc >= 3 ) )
			sscanf(argv[2],"%x", &value);
		else if( cmdGetSet == CMD_SET )
			PRINT_HELP_EXIT;
	}
	else
		PRINT_HELP_EXIT;
		//printf("\nUsage: %s cmd:\n\t[g][s](get signal status)\n\t[g|s][i](get|set input channel) index \n\t[g|s][h|v](get|set horizontal|vertical mirror) index ]\n\t[g|s][r|g|b](get|set RGB gain value) index ]\n\tcmd example:si 1 (set input channel:1)\n\n", argv[0]);

	fd = open(CAMERA_DEVICE, O_RDWR, 0);
	if (fd < 0) {
		perror("Open /dev/video failed\n");
		return -1;
	}

	switch( cmdType )
	{
		case CMD_SIGNAL_STATUS:
			if(cmdGetSet == CMD_GET )
			{
				//获取信号连接状态
				con.id = V4L2_CID_SIGNAL_STATUS; 
				ioctl(fd, VIDIOC_G_CTRL, &con);
				printf("Current signal status is :%s\n", con.value ? "Connectd" : "Disconnect");
			}
			break;
		case CMD_INPUT:
			if(cmdGetSet == CMD_SET )
			{
				//设置视频输入源序号
				printf("Set input sel is :0x%x\n", value);
				if (-1 == ioctl(fd, VIDIOC_S_INPUT, &value)) {
				    perror("VIDIOC_S_INPUT");
				    exit(EXIT_FAILURE);
				}
			}
			else
			{
				//获取视频输入源序号
				if (-1 == ioctl(fd, VIDIOC_G_INPUT, &value)) {
				    perror("VIDIOC_G_INPUT");
				    exit(EXIT_FAILURE);
				}
				printf("Current input sel is :%d\n", value);
				
				//获取视频输入源序号对应的名称
				input.index = value;  //check index 3 's name 
				if (-1 == ioctl(fd, VIDIOC_ENUMINPUT, &input)) {
				    perror("VIDIOC_ENUMINPUT");
				    exit(EXIT_FAILURE);
				}
				printf("Input index %d's name is :%s\n",input.index , input.name);
			}
			break;
		case CMD_H_MIRROR:
			if(cmdGetSet == CMD_SET )
			{
				//设置水平镜像:  0 : 无镜像     1   水平镜像
				con.id = V4L2_CID_SWAP_HOR; 
				con.value = value;   
				ioctl(fd, VIDIOC_S_CTRL, &con);
				printf("Set V4L2_CID_SWAP_HOR is :%d\n", con.value );
			}
			else
			{
				//获取水平镜像:  0 : 无镜像     1   水平镜像
				con.id = V4L2_CID_SWAP_HOR; 
				ioctl(fd, VIDIOC_G_CTRL, &con);
				printf("Get V4L2_CID_SWAP_HOR is :%d\n", con.value );
			}
			break;
		case CMD_V_MIRROR:
			if(cmdGetSet == CMD_SET )
			{
				//设置垂直镜像:  0 : 无镜像     1   垂直镜像
				con.id = V4L2_CID_SWAP_VER; 
				con.value = value;   
				ioctl(fd, VIDIOC_S_CTRL, &con);
				printf("Set V4L2_CID_SWAP_VER is :%d\n", con.value );
			}
			else
			{
				//获取垂直镜像:  0 : 无镜像     1   垂直镜像
				con.id = V4L2_CID_SWAP_VER; 
				ioctl(fd, VIDIOC_G_CTRL, &con);
				printf("Get V4L2_CID_SWAP_VER is :%d\n", con.value );
			}
			break;
		case CMD_R_GAIN:
			if(cmdGetSet == CMD_SET )
			{
				//设置gain_r
				con.id = V4L2_CID_GAIN_R; 
				con.value = value; 
				ioctl(fd, VIDIOC_S_CTRL, &con);
				printf("Set V4L2_CID_GAIN_R is :0x%x\n", con.value );
			}
			else
			{
				//获取gain_r
				con.id = V4L2_CID_GAIN_R; 
				ioctl(fd, VIDIOC_G_CTRL, &con);
				printf("Get V4L2_CID_GAIN_R is :0x%x\n", con.value );
			}
			break;
		case CMD_G_GAIN:
			if(cmdGetSet == CMD_SET )
			{
				//设置gain_g
				con.id = V4L2_CID_GAIN_G; 
				con.value = value; 
				ioctl(fd, VIDIOC_S_CTRL, &con);
				printf("Set V4L2_CID_GAIN_G is :0x%x\n", con.value );
			}
			else
			{
				//获取gain_g
				con.id = V4L2_CID_GAIN_G; 
				ioctl(fd, VIDIOC_G_CTRL, &con);
				printf("Get V4L2_CID_GAIN_G is :0x%x\n", con.value );
			}
			break;
		case CMD_B_GAIN:
			if(cmdGetSet == CMD_SET )
			{
				//设置gain_b
				con.id = V4L2_CID_GAIN_B; 
				con.value = value; 
				ioctl(fd, VIDIOC_S_CTRL, &con);
				printf("Set V4L2_CID_GAIN_B is :0x%x\n", con.value );
			}
			else
			{
				//获取gain_b
				con.id = V4L2_CID_GAIN_B; 
				ioctl(fd, VIDIOC_G_CTRL, &con);
				printf("Get V4L2_CID_GAIN_B is :0x%x\n", con.value );
			}
			break;
		default:
			break;
	}


	close(fd);

}


