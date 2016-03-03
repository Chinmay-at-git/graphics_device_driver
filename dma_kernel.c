#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/cred.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include "defs.h"

#define BUFF_SIZE (124*1024)

#define pci_vendor_ids_CCORSI 0x1234

#define PCI_DEVICE_ID_CCORCI_KYOUKO3 0x1113
#define Device_RAM 0x0020

#define NO_OF_BUFFS 8
#define FIFO_ENTRIES 1024

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("CS");

long kyouko3_ioctl(struct file *fp,unsigned int cmd, unsigned long arg);
void fifo_flush(void);
void fifo_flush_SMP(void);

DECLARE_WAIT_QUEUE_HEAD(dma_snooze);
DEFINE_SPINLOCK(SMP_lock);

struct dma_buffers_struct
{
	unsigned int *k_buffer_addr;
	dma_addr_t handle;
	unsigned long u_buffer_addr;
	unsigned long count;
} dma_buffers[NO_OF_BUFFS];

struct cdev kyouko3_cdev;

struct FIFO_entry
{
	u32 command;
	u32 value;
};

struct fifo_struct
{
	u64 p_base;
	struct FIFO_entry *k_base;
	u32 head;
	u32 tail_cache;
};

struct kyouko3{
	unsigned int dma_drain;
	unsigned int dma_fill;
	unsigned int flags; 
	uid_t user_id;
	unsigned long p_control_base;
	unsigned int  *k_control_base;
	unsigned long p_card_ram;
	unsigned int  *k_card_ram_base;
	unsigned long len_ctrl;
	unsigned long len_ram;
	struct pci_dev *dev;
	struct fifo_struct fifo;
	unsigned int graphics_on;
	char suspend;         //flag
	char buffers_alloted; //flag
}kyouko3;

unsigned int K_READ_REG( unsigned int reg)
{
	unsigned int value;
	rmb();
	value = *(kyouko3.k_control_base + (reg>>2));
	return(value);
}

void K_WRITE_REG(unsigned int reg,unsigned int val)
{
	*(kyouko3.k_control_base+(reg>>2)) = val;
}

void init_fifo(void)
{
	kyouko3.fifo.k_base = pci_alloc_consistent(kyouko3.dev,8192u,(dma_addr_t*)&kyouko3.fifo.p_base);
	//load FIFOStart with kyouko3.fifo.p_base
	K_WRITE_REG(FifoStart,kyouko3.fifo.p_base); 
	//load FIFOEnd with kyouko3.fifo.p_base +8192
	K_WRITE_REG(FifoEnd,(kyouko3.fifo.p_base) + 8192u);

	kyouko3.fifo.head = 0;
	kyouko3.fifo.tail_cache = 0;
}

int kyouko3_open(struct inode *inode, struct file *fp)
{
	unsigned size_ram;
	printk(" kyouko3 : Opening");
		
	kyouko3.k_control_base = ioremap(kyouko3.p_control_base,65536);//kyouko3.len_ctrl );
	size_ram = K_READ_REG(Device_RAM);
	size_ram = size_ram * 1024* 1024;
	kyouko3.k_card_ram_base = ioremap(kyouko3.p_card_ram,size_ram);
	kyouko3.user_id = get_current_cred()->uid.val; 
//	printk(KERN_ALERT "User ID is %d",kyouko3.user_id);
	init_fifo();
	kyouko3.suspend =0;
	fifo_flush();
	return 0;
}

int kyouko3_release( struct inode *inode, struct file *fp)
{
	kyouko3.buffers_alloted = 0;
	kyouko3.suspend = 0;
    pci_free_consistent(kyouko3.dev,8192u,kyouko3.fifo.k_base,*((dma_addr_t*)&kyouko3.fifo.p_base));
	iounmap(kyouko3.k_control_base);
	iounmap(kyouko3.k_card_ram_base);
	
	pci_clear_master(kyouko3.dev);
	pci_disable_device(kyouko3.dev);
	
	printk(KERN_ALERT " kyouko3_ release : Bye!!\n");	
	return 0;
}

int kyouko3_mmap(struct file *fp,struct vm_area_struct *vma)
{
	int ret=0;

	switch(vma->vm_pgoff << PAGE_SHIFT)
	{
		case 0: //control
				if(kyouko3.user_id!=0)
				{
					printk(KERN_ALERT "Only Root is allowed to mmap");
					return -1;
				}
				ret = io_remap_pfn_range(vma, vma->vm_start, kyouko3.p_control_base>>PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot);
				break;

		case 0x80000000: 
				if(kyouko3.user_id!=0)
				{	
					printk(KERN_ALERT "Only Root is allowed to mmap");
					return -1;
				}
				ret = io_remap_pfn_range(vma, vma->vm_start, kyouko3.p_card_ram>>PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot);
				break;

		default:ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, (vma->vm_end)-(vma->vm_start), vma->vm_page_prot);
//				printk(KERN_ALERT "mmap inside %d:",ret);
				break;
	}
	return(ret);
	
}

void fifo_write(unsigned int reg, unsigned int value)
{
	kyouko3.fifo.k_base[kyouko3.fifo.head].command=reg;
	kyouko3.fifo.k_base[kyouko3.fifo.head].value=value;
	kyouko3.fifo.head++;
	if(kyouko3.fifo.head >= FIFO_ENTRIES)
	{
		kyouko3.fifo.head = 0;
	}
}

void fifo_write_F(unsigned int reg, float value)
{
	kyouko3.fifo.k_base[kyouko3.fifo.head].command=reg;
	kyouko3.fifo.k_base[kyouko3.fifo.head].value=*(unsigned int *)&value;
	kyouko3.fifo.head++;
	if(kyouko3.fifo.head >= FIFO_ENTRIES)
	{
		kyouko3.fifo.head = 0;
	}
}

void fifo_flush(void)
{
	K_WRITE_REG(FifoHead,kyouko3.fifo.head);
//	printk(KERN_ALERT "\n In Fifo flush-");
	while(kyouko3.fifo.tail_cache != kyouko3.fifo.head)
	{
		kyouko3.fifo.tail_cache = K_READ_REG(FifoTail);
		schedule();
	}
//	printk(KERN_ALERT "-Fifo flush Completed\n");
}

void fifo_flush_SMP(void)
{
// SMP is a kind of misnomer, This function is used only in interrupt context and where we take the lock
	K_WRITE_REG(FifoHead,kyouko3.fifo.head);
//		printk(KERN_ALERT "\n-Fifo flush SMP started-");
	kyouko3.fifo.tail_cache = K_READ_REG(FifoTail);
	while(kyouko3.fifo.tail_cache != kyouko3.fifo.head)
	{
		kyouko3.fifo.tail_cache = K_READ_REG(FifoTail);
	//	schedule(); // We need this  
	}
//		printk(KERN_ALERT "-Fifo flush SMP Completed\n");
}

// Interrupt Handler
irqreturn_t k3_irq(int irq,void *dev_id,struct pt_regs *regs)
{
	unsigned int status_read;
	spin_lock(&SMP_lock);
	status_read = K_READ_REG(Status);
	K_WRITE_REG(Status,0xf);
	
	if((status_read & 0x02) == 0)
	{
		spin_unlock(&SMP_lock);
		return IRQ_NONE;  //Spurious interrupt
	}

	// Interrupt is valid now
	kyouko3.dma_drain= (kyouko3.dma_drain + 1) % NO_OF_BUFFS;
			
	// Consider now fill==drain. This suggest that queue is empty. If it is empty, Drain is pointing to something which user has not put on, i.e. garbage or old value.  
	if (kyouko3.dma_fill != kyouko3.dma_drain)
	{
		fifo_write(BufferA_Address, (unsigned int)dma_buffers[kyouko3.dma_drain].handle);
		fifo_write(BufferA_Config, (unsigned int)(dma_buffers[kyouko3.dma_drain].count*sizeof(float)));
		fifo_flush_SMP();
	}
			
	
	if(kyouko3.suspend)
	{
		// If kyouko3.suspend is exposed outside of spinlock: After the wake_up, If user goes and sends one more buffer and sets the suspend? 
		// That is not a problem.
		// One more contention, If suspend == 0, and user goes and sets it up. That, is not also problem.But, suspend ==1, we get in, make it zero, wake user up, 
		// The user goes and does 
		
		kyouko3.suspend = 0; //Clears the  suspension of user
		
	//	spin_unlock(&SMP_lock); 
		wake_up_interruptible(&dma_snooze); 
	// User will contain on spinlock till irq is returned. 
	//	spin_lock(&SMP_lock);
	}
	
	spin_unlock(&SMP_lock);		
	return(IRQ_HANDLED);			
}

// Start DMA function 
void start_transfer(void)
{	
	if(kyouko3.dma_fill == kyouko3.dma_drain)
	{
		// If user calls start dma and fill == drain, queue is empty
		// because we will not  go back to user when buffer is full	
		kyouko3.dma_fill =(kyouko3.dma_fill + 1) % NO_OF_BUFFS;
		fifo_write(BufferA_Address, (unsigned int)dma_buffers[kyouko3.dma_drain].handle);
		fifo_write(BufferA_Config, (unsigned int)(dma_buffers[kyouko3.dma_drain].count*sizeof(float)));
		fifo_flush_SMP();
		// Starting the transfer after buffer is empty..
		return ; 
	}
	
	kyouko3.dma_fill = (kyouko3.dma_fill + 1) % NO_OF_BUFFS;
	if(kyouko3.dma_fill == kyouko3.dma_drain)
	{
		// This is DMA buffers all full.
		kyouko3.suspend=1;
	}
	
	while(kyouko3.suspend)
	{
		spin_unlock_irqrestore(&SMP_lock,kyouko3.flags); // release lock before the sleep;
		// Assume: Interrupt comes right now. It frees a buffer and makes suspend==0
		// But meanwhile, wait_event will continue as normal. User would be put on hold, but as soon as it interrupt finishes with suspend==0, it will come out.
		// Now we will wait on spin_lock below to allow interrupt to finish clean. 
		// Suppose, instead of this, many interrupts come and go between waiting period of user, (for any reason): Buffers will be empty all way. 
		// But that is not a problem. 

		wait_event_interruptible(dma_snooze,kyouko3.suspend==0);
		// Interrupt routine will make suspend == 0. Till then user sleeps

		spin_lock_irqsave(&SMP_lock,kyouko3.flags);
		// Take the lock back before exiting the loop
		// What happens when user comes out?
		// - Its a honorable exit to userspace. User can send more buffers and can choose to go sleep again. Or do the unbind. What if unbind?
		// If we are in unbind , interrupts come and set the buffers. Interrupt can mess things up. There for, interrupts should be disabled 
		// Should we wait till fill==drain in unbind? Implemented.
		// Without the spin lock here: We may have interrupt which clears the suspend and perform wake_up even before user is on sleep;
		// Then our function comes in and put the user to wait forver!!

	}
}

long kyouko3_ioctl(struct file *fp,unsigned int cmd, unsigned long arg)
{
	int ret, i, check;
	struct FIFO_entry cur_entry;
	switch(cmd)
	{
		case UNBIND_DMA:
//						printk(KERN_ALERT "UNBIND: Waiting till buffers empty");
	
						for(i = 0; i < NO_OF_BUFFS; i++)
      					{
      			  			vm_munmap((unsigned long)dma_buffers[i].k_buffer_addr, (size_t) 124*1024);
        					pci_free_consistent(kyouko3.dev, 124*1024, dma_buffers[i].k_buffer_addr ,dma_buffers[i].handle);
      					}
						K_WRITE_REG(InterruptSet,0);
						free_irq(kyouko3.dev->irq,&kyouko3);
						pci_disable_msi(kyouko3.dev);
						kyouko3.buffers_alloted =0;
      					break;
    	
		case BIND_DMA:
//						printk(KERN_DEBUG "Binding DMA");
						for(i=0;i< NO_OF_BUFFS ; i++)
						{
							dma_buffers[i].k_buffer_addr = pci_alloc_consistent(kyouko3.dev, 124*1024, &(dma_buffers[i].handle));
							dma_buffers[i].count=0;
							dma_buffers[i].u_buffer_addr = vm_mmap(fp, ((unsigned long)(dma_buffers[i].handle)),124*1024, PROT_READ|PROT_WRITE, MAP_SHARED, dma_buffers[i].handle);	
						}
		
						kyouko3.dma_fill=0;
						kyouko3.dma_drain=0;
						kyouko3.buffers_alloted = 1;
						ret = pci_enable_msi(kyouko3.dev);
						if (ret)
						{	
//								printk(KERN_EMERG "Bind DMA failed, returning error code");
								return -1;
						}
						ret = request_irq(kyouko3.dev->irq,(irq_handler_t) k3_irq,IRQF_SHARED,"k3_irq",&kyouko3);
						if(ret)
						{
//							printk(KERN_EMERG "Bind DMA failed, returning error code");
							pci_disable_msi(kyouko3.dev);
							return -1;
						}
						K_WRITE_REG(Status,0xffffffff);
						K_WRITE_REG(InterruptSet,2);
						check = copy_to_user((int *) arg, &(dma_buffers[0].u_buffer_addr), sizeof(unsigned int));
						if(check)
						{
								printk(KERN_ALERT "BIND_DMA:Copy to user fails");
						}
						break;
		
		case START_DMA:
		
						spin_lock_irqsave(&SMP_lock,kyouko3.flags);
      					ret = copy_from_user( &dma_buffers[kyouko3.dma_fill].count, (unsigned int *) arg, sizeof(unsigned int));
						if(ret)
						{  
							printk(KERN_ALERT "Start_DMA: copy from user failed");
						}
						start_transfer(); // Actual start dma
						ret = copy_to_user((int *) arg, &(dma_buffers[kyouko3.dma_fill].u_buffer_addr), sizeof(unsigned int));
						if(ret)
						{
							printk(KERN_ALERT "Start_DMA: copy_to_user failed");
						}

						// Note that start_dma() changes dma_fill 
						spin_unlock_irqrestore(&SMP_lock,kyouko3.flags);	
						break;
		
		case VMODE:	
						if(arg == GRAPHICS_ON)
						{
							printk(KERN_ALERT "IN graphics on\n");
							K_WRITE_REG(0x8000+ _FColumns,1024);
							K_WRITE_REG(0x8000+_FRows,768);
							K_WRITE_REG(0x8000+_FRowPitch,1024*4);
							K_WRITE_REG(0x8000+_FFormat,0xf888);
							K_WRITE_REG(0x8000+_FAddress,0);
				
							K_WRITE_REG(0x9000+_DWidth,1024);
							K_WRITE_REG(0x9000+_DHeight,768);
							K_WRITE_REG(0x9000+_DVirtX,0);
							K_WRITE_REG(0x9000+_DVirtY,0);
							K_WRITE_REG(0x9000+_DFrame,0);
							K_WRITE_REG(Acceleration,0x40000000);
				
							udelay(2);
							K_WRITE_REG(ModeSet,0);
							udelay(10);
							fifo_write(Clear_Color,0x3F000000);//load clear color
							fifo_write(Clear_Color+4,0x3F000000);
							fifo_write(Clear_Color+8,0x3F000000);
							fifo_write(Clear_Color+12,0x3F800000);
							fifo_write(Clear_Buffer,0x03);

							fifo_write(Flush,0x00);
							udelay(10);
							fifo_flush();
							kyouko3.graphics_on = 1;
						}
						
						else
						{
							kyouko3.graphics_on = 0;
							fifo_flush();
							K_WRITE_REG(Acceleration,0x80000000);
							K_WRITE_REG(ModeSet,0);
						}
						break;
						
		case FIFO_QUEUE:ret = copy_from_user((void*)&cur_entry,(struct FIFO_entry*) arg,sizeof(struct FIFO_entry));
						if(ret)
						{	
							printk(KERN_EMERG "fifo_queue copy_from_user malfunction!");
						}
						fifo_write(cur_entry.command, cur_entry.value);
						break;

		case FIFO_FLUSH:fifo_flush();
						break;

		default:		break;
	}
	
	return 0;
}

int kyouko3_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
	kyouko3.p_control_base = pci_resource_start(pci_dev,1);
	kyouko3.p_card_ram = pci_resource_start(pci_dev,2);
	kyouko3.len_ctrl = pci_resource_len(pci_dev,1);
	kyouko3.len_ram = pci_resource_len(pci_dev,2);
	kyouko3.dev = pci_dev;
	return 0;
}

struct file_operations kyouko3_fops = {
	.open = kyouko3_open,
	.release = kyouko3_release,
	.unlocked_ioctl = kyouko3_ioctl,
	.mmap = kyouko3_mmap,
	.owner = THIS_MODULE
};

struct pci_device_id kyouko3_dev_ids[]={{PCI_DEVICE(pci_vendor_ids_CCORSI,PCI_DEVICE_ID_CCORCI_KYOUKO3)},{0}};
 
void kyouko3_remove(struct pci_dev* pci_dev)
{
	pci_disable_device(pci_dev);
}

struct pci_driver kyouko3_pci_dev =
{
	.name = "kyouko3",
	.id_table = kyouko3_dev_ids,
	.probe = kyouko3_probe,
	.remove = kyouko3_remove
};

int __init kyouko_init(void)
{
	cdev_init( &kyouko3_cdev, &kyouko3_fops);
	cdev_add(  &kyouko3_cdev, MKDEV(500,127), 1);
	kyouko3_cdev.owner = THIS_MODULE;
	if(pci_register_driver(&kyouko3_pci_dev))
	{
		printk(KERN_ALERT "Error\n");
	}
	if(pci_enable_device(kyouko3.dev))
	{
		printk(KERN_ALERT "PCI ENABLE error\n");
	}
	pci_set_master(kyouko3.dev);
	printk(KERN_ALERT "Kyouko3  Initialized\n");
	return 0;
}

void __exit kyouko_exit(void)
{
	pci_unregister_driver(&kyouko3_pci_dev);
	cdev_del(&kyouko3_cdev);
	printk(KERN_ALERT "Kyouko3 Exiting\n");
}

module_exit(kyouko_exit);
module_init(kyouko_init);
