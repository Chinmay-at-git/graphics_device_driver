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

void triangle(unsigned int *buffer)
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

  //bind the DMA buffer
  result = ioctl(fd, BIND_DMA, &arg);

  kyouko3.entry.command = Flush;
  kyouko3.entry.value = 0x00;
  result = ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
  result = ioctl(fd, FIFO_FLUSH, 0);

  kyouko3.header.stride = 5;
  kyouko3.header.w = 0;
  kyouko3.header.a = 1;
  kyouko3.header.unknown = 0;
  kyouko3.header.known = 1;
  kyouko3.header.unknown2 = 0;
  kyouko3.header.count = 3;        //single triangle
  kyouko3.header.opcode = 0x14;    //for triangle

  unsigned int* buffer = (unsigned int *)arg;
  unsigned int count = 19;
    
  for(i = 0; i < 100; i++)
  {
    //calling the function to implement a triangle
    triangle(buffer);

    //start the DMA buffer
    arg = *(unsigned int*)&count;
    result = ioctl(fd, START_DMA, &arg);

    kyouko3.entry.command = Flush;
    kyouko3.entry.value = 0x00;
    result = ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
    result = ioctl(fd, FIFO_FLUSH, 0);

    // Point buffer to next addr!
    buffer = (unsigned int *) arg;
  }

  result = ioctl(fd, UNBIND_DMA, 0);

  //sleep
  sleep(5);

  kyouko3.entry.command = Flush;
  kyouko3.entry.value = 0x00;
  result = ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
  result = ioctl(fd, FIFO_FLUSH, 0);

  result = ioctl(fd, VMODE, GRAPHICS_OFF);  //turn graphics mode off
  close(fd);

  return 0;
}
