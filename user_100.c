/*This user code can now draw 100 triangles.
 *But maybe addition of Sync ioctl call and some other funcs required?
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/ioctl.h>
#include "defs.h"

struct fifo_entry{
  uint32_t command;
  uint32_t value;
}fifo_entry;

struct u_kyouko_device {
  unsigned int *u_control_base;
  unsigned int *u_card_ram_base;
  struct fifo_entry entry;
  struct kyouko3_dma_hdr header; 
} kyouko3;

unsigned int U_READ_REG(unsigned int rgister)
{
  return (*(kyouko3.u_control_base+(rgister>>2)));
}

void U_WRITE_REG(unsigned int rgister, unsigned int value)
{
  (*(kyouko3.u_control_base+(rgister>>2))) = value; 
}

void U_WRITE_FB(unsigned int i, unsigned int value)
{
  (*(kyouko3.u_card_ram_base+i)) = value; 
}

float randF(void)
{
  return (float)rand()/(float)RAND_MAX;
}

float randF_pos(void)
{
  return 2*((float)rand()/(float)RAND_MAX)-1;
}

void triangle(unsigned int* buffer)
{
  float r, g, b, x, y;
  float z = 0.0;

  *buffer++ = *(unsigned int*)&kyouko3.header;
  //vertex1
  r = randF();
  g = randF();
  b = randF();
  x = randF_pos();
  y = randF_pos();

  *buffer++ = *(unsigned int*)&r;
  *buffer++ = *(unsigned int*)&g;
  *buffer++ = *(unsigned int*)&b;
  *buffer++ = *(unsigned int*)&x;
  *buffer++ = *(unsigned int*)&y;
  *buffer++ = *(unsigned int*)&z;

  //vertex2
  r = randF();
  g = randF();
  b = randF();
  x = randF_pos();
  y = randF_pos();

  *buffer++ = *(unsigned int*)&r;
  *buffer++ = *(unsigned int*)&g;
  *buffer++ = *(unsigned int*)&b;
  *buffer++ = *(unsigned int*)&x;
  *buffer++ = *(unsigned int*)&y;
  *buffer++ = *(unsigned int*)&z;

  //vertex3
  r = randF();
  g = randF();
  b = randF();
  x = randF_pos();
  y = randF_pos();

  *buffer++ = *(unsigned int*)&r;
  *buffer++ = *(unsigned int*)&g;
  *buffer++ = *(unsigned int*)&b;
  *buffer++ = *(unsigned int*)&x;
  *buffer++ = *(unsigned int*)&y;
  *buffer++ = *(unsigned int*)&z;
}

int main()
{
  int fd;
  int result;
  unsigned int i;
  unsigned long arg;
  srand(time(NULL));

  fd = open("/dev/kyouko3", O_RDWR);
  result = ioctl(fd, VMODE, GRAPHICS_ON);  //turn graphics mode on

  //ioctl call to sync which checks FIFO_DEPTH

  //bind the DMA buffer
  ioctl(fd,FIFO_FLUSH);
  result = ioctl(fd, BIND_DMA, &arg);
	ioctl(fd,FIFO_FLUSH);
  kyouko3.header.stride = 5;
  kyouko3.header.w = 0;
  kyouko3.header.a = 1;
  kyouko3.header.unknown = 0;
  kyouko3.header.known = 1;
  kyouko3.header.unknown2 = 0;
  kyouko3.header.count = 3;        //single triangle
  kyouko3.header.opcode = 0x14;    //for triangle

  unsigned int* buffer = (unsigned int*)(arg);

  for(i = 0; i < 100; i++)
  {
    //calling the function to implement a triangle
    triangle(buffer);

    //start the DMA buffer
	
    unsigned int count = 19;
    
	arg = *(unsigned int*)&count; //7 for one point
    ioctl(fd,FIFO_FLUSH);
	result = ioctl(fd, START_DMA, &arg);
	ioctl(fd,FIFO_FLUSH);
	buffer = (unsigned int *) arg;
	// Point buffer to next addr!
    //result = ioctl(fd, FIFO_FLUSH, 0);

    //U_WRITE_REG(Flush, 0);
  }

  

  //sleep
  sleep(5);
	result = ioctl(fd, UNBIND_DMA, 0);
  result = ioctl(fd, VMODE, GRAPHICS_OFF);  //turn graphics mode off
  close(fd);

  return 0;
}
