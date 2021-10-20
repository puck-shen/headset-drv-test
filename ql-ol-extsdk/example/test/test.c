
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>


int fd;

int main(int argc, char **argv)
{
	unsigned char gpio_val;
	int ret;
	int Oflags;
	
	fd = open("/dev/headset", O_RDWR);
	if (fd < 0)
	{
		printf("can't open!\n");
		return -1;
	}


	while (1)
	{
		ret = read(fd, &gpio_val, 1);
		printf("gpio_val: 0x%x, ret = %d\n", gpio_val, ret);
		//sleep(5);
	}
	
	return 0;
}

