
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <stdint.h>
#include "lab3_def.h"
#include <linux/types.h>
#include <asm/types.h>
//#include <sys/stdint.h>
#include <sys/types.h>

struct u_kyouko_device{
	unsigned int *u_control_base;
	unsigned int *u_ram_base;
}k3;

#define KYOUKO3_CONTROL_SIZE 65536
#define KYOUKO3_RAM_SIZE (32*1024*1024)
#define Device_RAM 0x0020    


unsigned int U_READ_REG(unsigned int rgist)
{
	return(*(k3.u_control_base+(rgist>>2)));
}

void U_WRITE_REG(unsigned int reg, unsigned int val)
{
	*(k3.u_control_base +(reg>>2)) = val;
}

void U_WRITE_FB(unsigned int reg, unsigned int val)
{
	*(k3.u_ram_base + reg) = val;
}

struct FIFO_entry
{
	__u32 command;
	__u32 value;
}Fifo_entry;
struct dma_header_struct
{
	uint32_t address:14;
	uint32_t count:10;
	uint32_t opcode:8;
};

struct dma_header_struct dma_header;


int main(void)
{
	int i,fd,result,p1,ret;
	char dummy;
	struct FIFO_entry my_entry;
	struct FIFO_entry try_entry;
	unsigned int *arg=NULL;
	float var = 1.0f;
	float bytwo = 0.5;
	float not_bytwo = -bytwo;
	float byeight = 0.125;
	int int_var = *((unsigned int*)&var);
	int int_bytwo = *((unsigned int*)&bytwo);
	int int_not_bytwo = *((unsigned int*)&not_bytwo);
	int int_byeight = *((unsigned int*)&byeight);
	
	printf("Entering User Code \n");
//	printf("Opening device Code \n");
	fd = open("/dev/kyouko3" , O_RDWR);
	printf("open done fd = %d\n",fd);
	
	ioctl(fd,VMODE,GRAPHICS_ON);
	printf("Only DMA\n");
	ret = ioctl(fd, BIND_DMA , &arg);
	printf("Ret=%d",ret);
	if(ret<0)
	{
		printf("error bind_dma");
		return -1;
	}
	dma_header.address = arg;
	dma_header.opcode = 0x14;
	dma_header.count = 3;
	p1 = 0;
	arg[p1++] = *(unsigned int *)&dma_header;
	
	arg[p1++] = Primitive;
    arg[p1++] = 1;
		arg[p1++] = Vertex_Color + 0;
	arg[p1++] = int_var;
	//done with second vertex
	arg[p1++] = Vertex_Color + 4;
	arg[p1++] =  0;
	arg[p1++] = Vertex_Color + 8;
	arg[p1++] = 0;
	arg[p1++] = Vertex_Color + 12;
	arg[p1++] = 0;
	//
	arg[p1++] = Vertex_Coordinate + 0;
	arg[p1++] = int_not_bytwo;
	
	arg[p1++] = Vertex_Coordinate + 4;
	arg[p1++] = int_not_bytwo;
	//done with one vertex
	arg[p1++] = Vertex_Coordinate + 8;
	arg[p1++] =  0;
	arg[p1++] = Vertex_Coordinate + 12;
	arg[p1++] = int_var;

	arg[p1++] = Vertex_Emit;
	arg[p1++] = 0;
	
	arg[p1++] = Vertex_Coordinate + 0;
	arg[p1++] = int_bytwo;
	arg[p1++] = Vertex_Coordinate + 4;
	arg[p1++]= 0;
	arg[p1++]= Vertex_Coordinate + 8;
	arg[p1++]= 0;
	arg[p1++] = Vertex_Coordinate + 12;
	arg[p1++] = int_var;
	arg[p1++] = Vertex_Emit;
	arg[p1++] = 0;

	
		arg[p1++] = Vertex_Coordinate + 0;
	arg[p1++] = int_bytwo;
	arg[p1++] = Vertex_Coordinate + 4;
	arg[p1++]= 0;
	arg[p1++]= Vertex_Coordinate + 8;
	arg[p1++]= 0;
	arg[p1++] = Vertex_Coordinate + 12;
	arg[p1++] = 0;
	arg[p1++] = Vertex_Emit;
	arg[p1++] = 0;
	
	arg[p1++] = Primitive;
    arg[p1++] = 0;
	
	printf("DMA completed ? ");
	ioctl(fd,START_DMA,&arg);
	sleep(5);
	ioctl(fd,VMODE,GRAPHICS_OFF);

	ioctl(fd,FIFO_FLUSH);
	printf("User exiting");
	close(fd);
	return 0;
	
	k3.u_control_base = mmap(0, KYOUKO3_CONTROL_SIZE, PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);

	if((long)k3.u_control_base == -1) 
	{
		printf("mmap error\n");
	}
	//printf("mmap ret = %d",(long)k3.u_control_base);
	result = U_READ_REG(Device_RAM);
	printf("Ram Size in MB is: %d\n",result);
	//scanf("%c",&dummy);
	k3.u_ram_base = mmap(0, 1024 * 768 * 4, PROT_READ|PROT_WRITE,MAP_SHARED,fd,0x80000000);

	ioctl(fd,VMODE,GRAPHICS_ON);
//	for( i = (200*1024) ; i < (201*1024) ; i++) 
//	U_WRITE_FB(i,0x00ff00);

	

	

	my_entry.command = Primitive;
	my_entry.value = 1;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 0;
	my_entry.value = int_not_bytwo;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 4;
	my_entry.value = int_not_bytwo;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 8;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 12;
	my_entry.value = int_var;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 0;
	my_entry.value = int_var;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 4;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 8;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 12;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Emit;
	my_entry.value = 0;
	ioctl(fd,FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 0;
	my_entry.value = int_bytwo;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 4;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 8;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 12;
	my_entry.value = int_var;
	ioctl(fd, FIFO_QUEUE,&my_entry);
//color
	my_entry.command = Vertex_Color + 0;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 4;
	my_entry.value = int_var;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 8;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 12;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Emit;
	my_entry.value = 0;
	ioctl(fd,FIFO_QUEUE,&my_entry);

//
	my_entry.command = Vertex_Coordinate + 0;
	my_entry.value = int_byeight;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 4;
	my_entry.value = int_bytwo;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 8;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Coordinate + 12;
	my_entry.value = int_bytwo;
	ioctl(fd, FIFO_QUEUE,&my_entry);

//
//color
//
	my_entry.command = Vertex_Color + 0;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 4;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 8;
	my_entry.value = int_var;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Color + 12;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);

	my_entry.command = Vertex_Emit;
	my_entry.value = 0;
	ioctl(fd,FIFO_QUEUE,&my_entry);

// Write Fifo 0 to 
	my_entry.command = Primitive;
	my_entry.value = 0;
	ioctl(fd, FIFO_QUEUE,&my_entry);
	my_entry.command = Flush;
	my_entry.value=0x00;
	ioctl(fd,FIFO_QUEUE,&my_entry);
	//FIFO_FLUSH
	//fifo_flush();
	//printf("U:Queue done\nStart Graphics?");
	//scanf("%c",&dummy);
	ioctl(fd, FIFO_FLUSH,0);
	sleep(10);
	ioctl(fd, VMODE,GRAPHICS_OFF);
	//printf("Successfully exited graphics mode :D");
	//scanf("%c",&dummy);
	close(fd);
	//U_WRITE_REG(Reboot,1);
	printf("Exiting User Code \n");
	//scanf("%c",&dummy);
	return 0;

}
