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

/***
 Random float value for color
***/

float random_float(void)
{
  return (float)rand()/(float)RAND_MAX;
}

/***
 Random float value for position
***/

float random_position(void)
{
  return 2*((float)rand()/(float)RAND_MAX)-1;
}

/***
 Routine to draw a traingle with random vertices and colors
***/

void draw_triangle(unsigned int *buffer)
{
  float r, g, b, x, y;
  float z = 0.0;

  /* Write the header to the buffer */
  *buffer++ = *(unsigned int*)&kyouko3.header;

  /* Write vertex 1 */
  r = random_float();
  g = random_float();
  b = random_float();
  x = random_position();
  y = random_position();

  *buffer++ = *(unsigned int*)&r;
  *buffer++ = *(unsigned int*)&g;
  *buffer++ = *(unsigned int*)&b;
  *buffer++ = *(unsigned int*)&x;
  *buffer++ = *(unsigned int*)&y;
  *buffer++ = *(unsigned int*)&z;

  /* Write vertex 2 */
  r = random_float();
  g = random_float();
  b = random_float();
  x = random_position();
  y = random_position();

  *buffer++ = *(unsigned int*)&r;
  *buffer++ = *(unsigned int*)&g;
  *buffer++ = *(unsigned int*)&b;
  *buffer++ = *(unsigned int*)&x;
  *buffer++ = *(unsigned int*)&y;
  *buffer++ = *(unsigned int*)&z;

  /* Write vertex 3 */
  r = random_float();
  g = random_float();
  b = random_float();
  x = random_position();
  y = random_position();

  *buffer++ = *(unsigned int*)&r;
  *buffer++ = *(unsigned int*)&g;
  *buffer++ = *(unsigned int*)&b;
  *buffer++ = *(unsigned int*)&x;
  *buffer++ = *(unsigned int*)&y;
  *buffer++ = *(unsigned int*)&z;
}

/***
 Draws a single triangle by queueing
 commands one-by-one into the FIFO 
***/

int fifo_draw() {
	
 /* Hard-coded floats for drawing the traingle */
 float fh = 0.5,
       fq = 0.125,
	   fhn = -0.5,
	   w = 1.0,
	   z = 0.0; 

 int mmap_ret, ram, i;
 
 /* Map control-region */
 int fd = open("/dev/kyouko3",O_RDWR);
 kyouko3.u_control_base = mmap(0, KYOUKO_CONTROL_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

 /* Map frame buffer region */ 
 mmap_ret = U_READ_REG(0x2000);
 ram = mmap_ret * 1024 * 1024;
 kyouko3.u_card_ram_base = mmap(0, 1024*768*4, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x80000000);
 ioctl(fd,VMODE,GRAPHICS_ON);

 /* Write command primitive to fifo */
 kyouko3.entry.command = Command_Primitive;
 kyouko3.entry.value = 1;
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);

 /* Write vertex 1 */
 kyouko3.entry.command = Vertex_Coordinate;
 kyouko3.entry.value = *((unsigned int *)&fhn);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Coordinate + 4;
 kyouko3.entry.value = *((unsigned int*)&fhn);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Coordinate + 8;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry); 
 kyouko3.entry.command = Vertex_Coordinate + 12;
 kyouko3.entry.value = *((unsigned int*)&w);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color;
 kyouko3.entry.value = *((unsigned int*)&w);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 4;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 8;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 12;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Emit;
 kyouko3.entry.value = 0;
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);

 /* Write vertex 2 */
 kyouko3.entry.command = Vertex_Coordinate;
 kyouko3.entry.value = *((unsigned int *)&fh);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Coordinate + 4;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Coordinate + 8;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry); 
 kyouko3.entry.command = Vertex_Coordinate + 12;
 kyouko3.entry.value = *((unsigned int*)&w);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 4;
 kyouko3.entry.value = *((unsigned int*)&w);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 8;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 12;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Emit;
 kyouko3.entry.value = 0;
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);

 /* Write vertex 3 */
 kyouko3.entry.command = Vertex_Coordinate;
 kyouko3.entry.value = *((unsigned int *)&fq);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Coordinate + 4;
 kyouko3.entry.value = *((unsigned int*)&fh);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Coordinate + 8;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry); 
 kyouko3.entry.command = Vertex_Coordinate + 12;
 kyouko3.entry.value = *((unsigned int*)&w);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 4;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 8;
 kyouko3.entry.value = *((unsigned int*)&w);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Color + 12;
 kyouko3.entry.value = *((unsigned int*)&z);
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
 kyouko3.entry.command = Vertex_Emit;
 kyouko3.entry.value = 0;
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);

 /* Write command primitive to fifo */
 kyouko3.entry.command = Command_Primitive;
 kyouko3.entry.value = 0;
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);

 /* Flushing */
 kyouko3.entry.command = Flush;
 kyouko3.entry.value = 0x00;
 ioctl(fd, FIFO_QUEUE, &kyouko3.entry);

 ioctl(fd, FIFO_FLUSH, 0);
 sleep(5);

 /* Turn graphics off */
 ioctl(fd, VMODE, GRAPHICS_OFF);
 close(fd);
 
 /* Return success */
 return 0;
}

/***
 Draws multiple (100) triangles by loading
 them into multiple (8) DMA buffers
***/

int dma_draw() {
	
  int fd;
  int result;
  unsigned int i;
  unsigned long arg;

  /* Initialize */
  fd = open("/dev/kyouko3", O_RDWR);
  result = ioctl(fd, VMODE, GRAPHICS_ON);
  
  /* Bind DMA buffer */
  result = ioctl(fd, BIND_DMA, &arg);

  kyouko3.entry.command = Flush;
  kyouko3.entry.value = 0x00;
  result = ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
  result = ioctl(fd, FIFO_FLUSH, 0);

  kyouko3.header.stride = 5;
  kyouko3.header.w = 0;
  kyouko3.header.a = 1;
  kyouko3.header.bit3 = 0;
  kyouko3.header.bit4 = 1;
  kyouko3.header.bit5 = 0;
  kyouko3.header.count = 3;
  kyouko3.header.opcode = 0x14;

  unsigned int* buffer = (unsigned int *)arg;
  unsigned int count = 19;
    
  for(i = 0; i < 100; i++)
  {

    draw_triangle(buffer);

    /* Start DMA buffer */
    arg = *(unsigned int*)&count;
    result = ioctl(fd, START_DMA, &arg);

    kyouko3.entry.command = Flush;
    kyouko3.entry.value = 0x00;
    result = ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
    result = ioctl(fd, FIFO_FLUSH, 0);

	/* Points the buffer to the next argument */
    buffer = (unsigned int *) arg;
  }

  

  sleep(5);

  kyouko3.entry.command = Flush;
  kyouko3.entry.value = 0x00;
  result = ioctl(fd, FIFO_QUEUE, &kyouko3.entry);
  result = ioctl(fd, FIFO_FLUSH, 0);
	result = ioctl(fd, UNBIND_DMA, 0);
  result = ioctl(fd, VMODE, GRAPHICS_OFF);
  close(fd);

  /* Return success */
  return 0;
}

int main()
{
 int choice;

  srand(time(NULL));
  
  /* User's choice from command-line */
  printf("Enter 1 for FIFO, any other number for DMA: ");
  scanf("%d",&choice);
 
  /* Draw fifo traingle */
  if (choice == 1) {
	  fifo_draw();
  }
  
  /* Draw dma triangles */
  else {
	  dma_draw();
  }
  
  return 0;
}
