#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "linux/ioctl.h"
#define keyCmdMAGIC		0xEF
#define KEYCMDW         _IOR(keyCmdMAGIC,0,unsigned int)
#define KEYCMDR 		_IOR(keyCmdMAGIC,1,unsigned int)
static void childPro_entry(int fd);
char Rbuf[20];

int main(int argc, char *argv[])
{
	int fd;
	int ret = 0;
	char *filename;
	unsigned int data = 0;
	pid_t pid;

	// unsigned char data;
	
	if (argc != 2) {
		printf("Error Usage!\r\n");
		return -1;
	}

	filename = argv[1];
	fd = open(filename, O_RDWR);
	if (fd < 0) {
		printf("Can't open file %s\r\n", filename);
		return -1;
	}
	pid = fork();
	switch (pid)
	{
	case -1:
		perror("fork");
		exit(1);
		break;
	case 0: /*子进程*/
		childPro_entry(fd);
		break;	
	default:/*父进程*/
		break;
	}

	while (1) { /*父进程*/

		ret = read(fd,Rbuf,sizeof(Rbuf));
		printf("read is : %s\r\n",Rbuf);
		
        sleep(1);
	}
	close(fd);
	return ret;
}
static void childPro_entry(int fd)
{
	int ret;
	char Wbuf[20]="dasfagfdsgfdgsa";
	while(1)
	{
		ret = write(fd,Wbuf,sizeof(Wbuf));
		printf("write ret %d\r\n",ret);
		if(ret > 0)
		printf("write ok\r\n");
		sleep(1);
	}

}