#include <linux/module.h>

#include <linux/fs.h>
#include <linux/aio.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "wishbone.h"

#define WB_NAME "wishbone"

#define DBG_PRINT_LVL -2


#define EB_RX_FIFO_DATA         0x20 //
#define EB_TX_FIFO_DATA         0x30 //


/* Module parameters */
static unsigned int max_devices = WISHBONE_MAX_DEVICES;

static unsigned int debug          = 1; /* module parameter, enable debug prints */
static unsigned int debug_cyc_time = 0; /* module parameter, enable debug prints for max cycle time */
static unsigned int debug_ucpy     = 1; /* module parameter, enable debug prints */
static unsigned int debug_skip_cyc_drop = 0; /* module parameter, disable dropping cycle line */

static unsigned int debug_abs_max_cyc_op_time     = 0; /* module parameter, max cycle time after each cycle */
static unsigned int debug_clear_cycle_time = 0; /* module parameter, clear max cycle time after each cycle */

static unsigned int selebslv = 0;

static unsigned long polldly = 0;

//#define FNAME if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s\n", __FUNCTION__);
#define FNAME

/* Module globals */
static LIST_HEAD(wishbone_list); /* Sorted by ascending minor number */
static DEFINE_MUTEX(wishbone_mutex);
static struct class *wishbone_master_class;
static dev_t wishbone_master_dev_first;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30) || LINUX_VERSION_CODE > KERNEL_VERSION(3,1,19)

/* missing 'const' in 2.6.30. present in 2.6.31. */
static int compat_memcpy_fromiovecend(unsigned char *kdata, const struct iovec *iov,
                        int offset, int len)
                        
{
        int i;
        
        FNAME
  
        /* Skip over the finished iovecs */
        while (offset >= iov->iov_len) {
                offset -= iov->iov_len;
                iov++;
        }

#if DBG_PRINT_LVL > -1
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s : Copying %d bytes of data FROM user space\n", __FUNCTION__, len);
#endif
        while (len > 0) {
                u8 __user *base = iov->iov_base + offset;
                int copy = min_t(unsigned int, len, iov->iov_len - offset);

                offset = 0;
                if (copy_from_user(kdata, base, copy))
                        return -EFAULT;

#if DBG_PRINT_LVL > -1                        
                if (unlikely(debug_ucpy)){
                  for(i=0; i<copy;i++){
                    printk(KERN_DEBUG WB_NAME ": %s : 0x%02x\n", __FUNCTION__, kdata[i]);           
                  }
                }
#endif                
                len -= copy;
                  kdata += copy;
                iov++;
        }

        return 0;
}



/* does not exist in 2.6.30. does in 2.6.31. */
static int compat_memcpy_toiovecend(const struct iovec *iov, unsigned char *kdata,
                       int offset, int len)
 {
         int copy, i;
         
         FNAME

#if DBG_PRINT_LVL > -1        
         if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s : Copying %d bytes of data TO user space\n", __FUNCTION__, len);         
#endif
         
         for (; len > 0; ++iov) {
                 /* Skip over the finished iovecs */
                 if (unlikely(offset >= iov->iov_len)) {
                         offset -= iov->iov_len;
                         continue;
                 }
                 copy = min_t(unsigned int, iov->iov_len - offset, len);

                 if (copy_to_user(iov->iov_base + offset, kdata, copy))
                         return -EFAULT;

#if DBG_PRINT_LVL > -1                        
                if (unlikely(debug_ucpy)){
                  for(i=0; i<copy;i++){
                    printk(KERN_DEBUG WB_NAME ": %s : 0x%02x\n", __FUNCTION__, kdata[i]);           
                  }
                }
#endif                         
                 offset = 0;
                 kdata += copy;
                 len -= copy;
         }

         return 0;
}

/* Over-ride with compatible versions */
#define memcpy_toiovecend   compat_memcpy_toiovecend
#define memcpy_fromiovecend compat_memcpy_fromiovecend
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
/* Older linux versions do not have the drvdata 'a' parameter. >= 2.6.37 present. */
#define device_create(c, p, d, a, f, x) device_create(c, p, d, f, x)
#endif

/* Compiler should be able to optimize this to one inlined instruction */
static inline wb_data_t eb_to_cpu(unsigned char* x)
{
    switch (sizeof(wb_data_t)) {
    case 8: return be64_to_cpu(*(wb_data_t*)x);
    case 4: return be32_to_cpu(*(wb_data_t*)x);
    case 2: return be16_to_cpu(*(wb_data_t*)x);
    case 1: return *(wb_data_t*)x;
    }
}

/* Compiler should be able to optimize this to one inlined instruction */
static inline void eb_from_cpu(unsigned char* x, wb_data_t dat)
{
    switch (sizeof(wb_data_t)) {
    case 8: *(wb_data_t*)x = cpu_to_be64(dat); break;
    case 4: *(wb_data_t*)x = cpu_to_be32(dat); break;
    case 2: *(wb_data_t*)x = cpu_to_be16(dat); break;
    case 1: *(wb_data_t*)x = dat;              break;
    }
}

/* Called from the wb->msi_workqueue */
static void wishbone_dispatch_msi(struct work_struct *work)
{
    struct wishbone* wb;
    struct wishbone_request request;
    struct etherbone_master_context *context;
    unsigned long flags;
    uint8_t *wptr;
    int index, i;
  
        FNAME

    wb = container_of(work, struct wishbone, msi_handler);
    context = 0;

    /* Hold this mutex for the whole handler */
    mutex_lock(&wb->msi_mutex);

    /* Hold this mutex while we look for stuff to deliver */
    mutex_lock(&wb->device_mutex);

    /* Don't process a second MSI while a previous is inflight */
    if (!wb->msi_pending) {
        /* Process requests */

#if DBG_PRINT_LVL > 1    
    printk(KERN_DEBUG WB_NAME ": %s: MSI not pending, process requests\n", __FUNCTION__); 
#endif

        while (wb->wops->request(wb, &request)) {
            /* The hardware should already have done this, but be safe */
            request.addr &= wb->mask;

            /* Find the context which receives this MSI */
            index = request.addr / ((wb->mask/WISHBONE_MAX_MSI_OPEN)+1);
            spin_lock_irqsave(&wb->msi_spinlock, flags);
            context = wb->msi_map[index];
            spin_unlock_irqrestore(&wb->msi_spinlock, flags);

            if (context) {
                /* We will dispatch this! */
                wb->msi_pending = 1;
                break;
            } else {

#if DBG_PRINT_LVL > 1        
        printk(KERN_DEBUG WB_NAME ": %s: no MSI handler, handle it immediately\n", __FUNCTION__); 
#endif
                /* If no MSI handler, handle it immediately */
                wb->wops->reply(wb, 1, ~(wb_data_t)0);
            }
        }
    }

    mutex_unlock(&wb->device_mutex);

    /* Deliver the MSI */
    if (context) {
#if DBG_PRINT_LVL > 1
            if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: delivering context=0x%08X\n", __FUNCTION__, context);
#endif            

        mutex_lock(&context->context_mutex);

        /* Fill in the MSI data */
        wptr = &context->msi[0];

        wptr[0] = ETHERBONE_BCA;
        wptr[1] = request.mask;
        if (request.write) {
            wptr[2] = 1;
            wptr[3] = 0;
            wptr += sizeof(wb_data_t);
            eb_from_cpu(wptr, request.addr);
            wptr += sizeof(wb_data_t);
            eb_from_cpu(wptr, request.data);
            wptr += sizeof(wb_data_t);
        } else {
            wptr[2] = 0;
            wptr[3] = 1;
            wptr += sizeof(wb_data_t);
            eb_from_cpu(wptr, WBA_DATA);
            wptr += sizeof(wb_data_t);
            eb_from_cpu(wptr, request.addr);
            wptr += sizeof(wb_data_t);
        }

        wptr[0] = ETHERBONE_CYC | ETHERBONE_BCA | ETHERBONE_RCA;
        wptr[1] = 0xf;
        wptr[2] = 0;
        wptr[3] = 1;
        wptr += sizeof(wb_data_t);

        eb_from_cpu(wptr, WBA_ERR);
        wptr += sizeof(wb_data_t);
        eb_from_cpu(wptr, 4); /* low bits of error status register */
        wptr += sizeof(wb_data_t);

        /* Mark the MSI pending */
        context->msi_unread = wptr - &context->msi[0];
        context->msi_pending = 1;

        mutex_unlock(&context->context_mutex);
    
#if DBG_PRINT_LVL > 4
    if (unlikely(debug)){
      printk(KERN_DEBUG WB_NAME ": %s: MSI data\n", __FUNCTION__);    
      for(i = 0; i<sizeof(context->msi) ;i++){  
        printk(KERN_DEBUG WB_NAME ": 0x%02X\n", context->msi[i]);
      }
    }
#endif

        /* Wake-up any reader of the device */
        wake_up_interruptible(&context->waitq);
        kill_fasync(&context->fasync, SIGIO, POLL_IN);
    }

    mutex_unlock(&wb->msi_mutex);
}

/* Must be called with context_mutex held */
static void claim_msi(struct etherbone_master_context* context)
{
    unsigned long flags;
    unsigned i;
    struct wishbone *wb = context->wishbone;
  
        FNAME

    /* Safe to read msi_index here, because context_mutex held */
    if (context->msi_index != -1) return;

    spin_lock_irqsave(&wb->msi_spinlock, flags);
    for (i = 0; i < WISHBONE_MAX_MSI_OPEN; ++i) {
        if (!wb->msi_map[i]) {
            context->msi_index = i;
            wb->msi_map[i] = context;
#if DBG_PRINT_LVL > 4      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: msi_index=%d context=0x%08X\n", __FUNCTION__, i, context);
#endif      
            break;
        }
    }
    spin_unlock_irqrestore(&wb->msi_spinlock, flags);
}

/* Must be called with both context_mutex and device_mutex held */
static wb_data_t handle_read_cfg(struct etherbone_master_context* context, wb_addr_t addr)
{
  
        wb_data_t rdata;
  
    /* Safe to read msi_index here, because context_mutex held */
    struct wishbone *wb = context->wishbone;
    wb_data_t wide = (wb->mask/WISHBONE_MAX_MSI_OPEN)+1;
  
        FNAME  
  
    switch (addr) {
    case 32: return 0;                             // request high
    case 36: return 0;                             // request low
    case 40: return 0;                             // granted high
    case 44: return context->msi_index != -1;      // granted low
    case 48: return 0;                             // low high
    case 52: return wide*(context->msi_index+0)-0; // low low
    case 56: return 0;                             // high high
    case 60: return wide*(context->msi_index+1)-1; // high low
    default: 
          rdata  = wb->wops->read_cfg(wb, addr);
#if DBG_PRINT_LVL > 4         
          if (unlikely(debug)){
             printk(KERN_DEBUG WB_NAME ": %s: A=0x%08X D=0x%08X\n", __FUNCTION__, addr, rdata);
          }
#endif          
          return rdata;
    }
}

/* Must be called with both context_mutex and device_mutex held */
static void handle_write_cfg(struct etherbone_master_context* context, wb_addr_t addr, wb_data_t data)
{
  
    struct wishbone *wb = context->wishbone;

        FNAME
  
    switch (addr) {
    case 36:
        if (data == 1) {
            claim_msi(context);
        }
        break;

    case WBA_DATA:
        context->msi_data = data;
        break;

    case WBA_ERR:
        if (context->msi_pending) {
            context->msi_pending = 0;
            wb->msi_pending = 0;
#if DBG_PRINT_LVL > 4     
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s:WBA_ERR A=0x%08X D=0x%08X\n", __FUNCTION__, addr, data);
#endif
            wb->wops->reply(wb, data&1, context->msi_data);
            wishbone_slave_ready(wb);
        }
        break;
    }
}

/* Must be called with context_mutex held */
static void etherbone_master_process(struct etherbone_master_context* context)
{
    struct wishbone *wb;
    const struct wishbone_operations *wops;
    unsigned int size, left, i, record_len;
    unsigned char *buf;
  unsigned int rd_addr;
  unsigned int max_cyc_op_time;
  
        FNAME

    if (context->state == header) {
        if (context->received < 8) {
            /* no-op */
            return;
        }

#if DBG_PRINT_LVL > 4
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Processing EB header\n", __FUNCTION__);
#endif    
        context->buf[0] = 0x4E;
        context->buf[1] = 0x6F;
        context->buf[2] = 0x12; /* V.1 Probe-Response */
        context->buf[3] = (sizeof(wb_addr_t)<<4) | sizeof(wb_data_t);
        /* Echo back bytes 4-7, the probe identifier */
        context->processed = 8;
        context->state = idle;
    }

    buf = &context->buf[0];
    wb = context->wishbone;
    wops = wb->wops;

#if DBG_PRINT_LVL > -1
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: START sent=%d processed=%d received=%d\n", 
     __FUNCTION__, context->sent, context->processed, context->received);  
#endif     

    i = RING_INDEX(context->processed);
  
#if DBG_PRINT_LVL > -1
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: ring index i=%d\n", __FUNCTION__, i);  
#endif  
  
    size = RING_PROC_LEN(context);
  
#if DBG_PRINT_LVL > -1  
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: EB packet size=%d\n", __FUNCTION__, size);  
#endif

  // Process EB records
    for (left = size; left >= 4; left -= record_len) {
        unsigned char flags, be, wcount, rcount;

        /* Determine record size */
        flags  = buf[i+0];
        be     = buf[i+1];
        wcount = buf[i+2];
        rcount = buf[i+3];

        record_len = 1 + wcount + rcount + (wcount > 0) + (rcount > 0);
    
#if DBG_PRINT_LVL > 4    
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: EB Record length=%d WC=%d RC=%d\n", 
      __FUNCTION__, record_len, wcount, rcount);
#endif      

        record_len *= sizeof(wb_data_t);

#if DBG_PRINT_LVL > 4    
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: EB Record length in bytes=%d\n", 
      __FUNCTION__, record_len);
#endif
    
        if (left < record_len) break;

        /* Configure byte enable and raise cycle line */
        if (context->state == idle) {
            mutex_lock(&wb->device_mutex);
            context->state = cycle;
      
#if DBG_PRINT_LVL > 4      
            if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Raising cycle\n", __FUNCTION__);
#endif      
      
            wops->cycle(wb, 1);
        }
    
#if DBG_PRINT_LVL > 4    
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Byte enable=%d\n", __FUNCTION__, be);
#endif
    
        wops->byteenable(wb, be);

        /* Process the writes */
        if (wcount > 0) {
            wb_addr_t base_address, increment;
            unsigned char j;
            int wff = flags & ETHERBONE_WFF;
            int wca = flags & ETHERBONE_WCA;

#if DBG_PRINT_LVL > 4
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Processing %d writes\n", __FUNCTION__, wcount);
#endif

            /* increment=0 if wff!=0 */
            increment = sizeof(wb_data_t) * (1 - (wff / ETHERBONE_WFF));

            /* Erase the header */
            eb_from_cpu(buf+i, 0);
            i = RING_INDEX(i + sizeof(wb_data_t));
            base_address = eb_to_cpu(buf+i);

            if (wca) {
                for (j = wcount; j > 0; --j) {
                    eb_from_cpu(buf+i, 0);
                    i = RING_INDEX(i + sizeof(wb_data_t));
          
#if DBG_PRINT_LVL > 4          
                    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: write pci cfg A=0x%08X D=0x%08X\n", 
                                               __FUNCTION__, base_address, eb_to_cpu(buf+i));
#endif

                    handle_write_cfg(context, base_address, eb_to_cpu(buf+i));
                    base_address += increment;
                }
            } else {
                for (j = wcount; j > 0; --j) {
                    eb_from_cpu(buf+i, 0);
                    i = RING_INDEX(i + sizeof(wb_data_t));
          
#if DBG_PRINT_LVL > 4          
                    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: write pci A=0x%08X D=0x%08X\n", 
                                               __FUNCTION__, base_address, eb_to_cpu(buf+i));
#endif

                    wops->write(wb, base_address, eb_to_cpu(buf+i));
                    base_address += increment;
                }
            }
        }

        buf[i+0] = (flags & ETHERBONE_CYC) |
                   (((flags & ETHERBONE_RFF) != 0) ? ETHERBONE_WFF : 0) |
                   (((flags & ETHERBONE_BCA) != 0) ? ETHERBONE_WCA : 0);
        buf[i+1] = be;
        buf[i+2] = rcount; /* rcount -> wcount */
        buf[i+3] = 0;

        if (rcount > 0) {
            unsigned char j;
            int rca = flags & ETHERBONE_RCA;

#if DBG_PRINT_LVL > 4
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Processing %d reads\n", __FUNCTION__, rcount);
#endif
      
            /* Move past header, and leave BaseRetAddr intact */
            i = RING_INDEX(i + sizeof(wb_data_t) + sizeof(wb_data_t));

            if (rca) {
                for (j = rcount; j > 0; --j) {
          rd_addr = eb_to_cpu(buf+i);
                    eb_from_cpu(buf+i, handle_read_cfg(context, eb_to_cpu(buf+i)));
          
#if DBG_PRINT_LVL > 4          
                    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: read cfg A=0x%08X D=0x%08X\n", 
               __FUNCTION__, rd_addr, eb_to_cpu(buf+i));
#endif

                    i = RING_INDEX(i + sizeof(wb_data_t));
                }
            } else {
                for (j = rcount; j > 0; --j) {
          rd_addr = eb_to_cpu(buf+i);
                    eb_from_cpu(buf+i, wops->read(wb, eb_to_cpu(buf+i)));
          
#if DBG_PRINT_LVL > 4          
          if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: read pci A=0x%08X D=0x%08X\n", 
             __FUNCTION__, rd_addr, eb_to_cpu(buf+i));
#endif

                    i = RING_INDEX(i + sizeof(wb_data_t));
                }
            }
        } else {
            i = RING_INDEX(i + sizeof(wb_data_t));
        }

        if ((flags & ETHERBONE_CYC) != 0) {
#if DBG_PRINT_LVL > 4      
      if(debug_skip_cyc_drop){
              if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Skipping cycle drop\n", __FUNCTION__);
      }else{
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Dropping cycle\n", __FUNCTION__);
#endif        
        wops->cycle(wb, 0);
#if DBG_PRINT_LVL > 4        
      }  
#endif

#if DBG_PRINT_LVL > 0      
      // read cycle op time and store it if it is longer that previous
      if (unlikely(debug_cyc_time)) {
          max_cyc_op_time = handle_read_cfg(context, 0xF4);
          if (max_cyc_op_time > debug_abs_max_cyc_op_time) debug_abs_max_cyc_op_time = max_cyc_op_time;
          
          printk(KERN_DEBUG WB_NAME ": %s: Max_Cycle_Op_Time=%6d:\tAbs_Op_Max_Time=%6d:\n", 
            __FUNCTION__, max_cyc_op_time, debug_abs_max_cyc_op_time);        
            
          // clear max cycle op time
          if (debug_clear_cycle_time) handle_read_cfg(context, 0xF5);
      }
#endif
 
            context->state = idle;
            mutex_unlock(&wb->device_mutex);
        }
    }

    context->processed = RING_POS(context->processed + size - left);
  
#if DBG_PRINT_LVL > 4  
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: END sent=%d processed=%d received=%d\n", 
     __FUNCTION__, context->sent, context->processed, context->received);  
#endif

}



// ########################################################################################
// START - START - START - START - START - START - START - START - START - START - START
// ########################################################################################

/* Must be called with context_mutex held */
static void etherbone_master_process_ebs(struct etherbone_master_context* context)
{
  struct wishbone *wb;
  const struct wishbone_operations *wops;
  unsigned int size, left, i, j, record_len;
  unsigned char *buf;
  unsigned int rd_addr;
  unsigned int max_cyc_op_time;
  
  FNAME

  wb = context->wishbone;
  wops = wb->wops;
  buf = &context->buf[0];

  i = RING_INDEX(context->processed);
  
#if DBG_PRINT_LVL > -1  
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: ring index i=%d\n", __FUNCTION__, i);  
#endif  
  
    size = RING_PROC_LEN(context);
  
#if DBG_PRINT_LVL > -1
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: EB packet data size=%d\n", __FUNCTION__, size);  
#endif
  
  if (context->state == header) {
#if DBG_PRINT_LVL > 3
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context->state==header\n", __FUNCTION__);
#endif    

        if (context->received < 8) {
           // full header not yet received, no-op 
#if DBG_PRINT_LVL > 3
          if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Full EB header not yet written\n", __FUNCTION__);
#endif    
            return;
        }

#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Processing EB header\n", __FUNCTION__);
#endif    
//        context->buf[0] = 0x4E;
//        context->buf[1] = 0x6F;
//        context->buf[2] = 0x12; /* V.1 Probe-Response */
//        context->buf[3] = (sizeof(wb_addr_t)<<4) | sizeof(wb_data_t);

    // write EB header to EB slave
        /* Configure byte enable and raise cycle line */
#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Locking device mutex\n", __FUNCTION__);
#endif    
        mutex_lock(&wb->device_mutex);


//#if DBG_PRINT_LVL > 3
//    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Opening EB cycle\n", __FUNCTION__);
//#endif    
//    wops->cycle_ebs(wb, 1);
//    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Opened EB cycle\n", __FUNCTION__);

    #if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: writing EB header [0]: 0x%08X\n", __FUNCTION__, eb_to_cpu(buf+0));
#endif
//    msleep(10);
    
    wops->write_ebs(wb, EB_RX_FIFO_DATA, eb_to_cpu(buf+0));

#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: writing EB header [1]: 0x%08X\n", __FUNCTION__, eb_to_cpu(buf+4));
#endif    
    wops->write_ebs(wb, EB_RX_FIFO_DATA, eb_to_cpu(buf+4));
    
    // ensure that writes are done
    mb();

    // read back responses
    eb_from_cpu(buf+0, wops->read_ebs(wb, EB_TX_FIFO_DATA));
#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Read EB header [0] : 0x%08X\n", __FUNCTION__, buf+0);
#endif    
    
    eb_from_cpu(buf+4, wops->read_ebs(wb, EB_TX_FIFO_DATA));
#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Read EB header [1] : 0x%08X\n", __FUNCTION__, buf+4);
#endif    

#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Unlocking device mutex\n", __FUNCTION__);
#endif    
    mutex_unlock(&wb->device_mutex);

        context->processed = 8;
        context->state = cycle;
        
#if DBG_PRINT_LVL > 3
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Finished EB header processing\n", __FUNCTION__);
#endif    
        
    }else if ((context->state == cycle) && (size > 3)) {

    buf = &context->buf[0];

#if DBG_PRINT_LVL > 3
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context->state==cycle\n", __FUNCTION__);
#endif    


#if DBG_PRINT_LVL > -1
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: START sent=%d processed=%d received=%d\n", 
     __FUNCTION__, context->sent, context->processed, context->received);  
#endif     

    // start processing EB records if there is at least one record header in the buffer
        
#if DBG_PRINT_LVL > 0  
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context->state : idle > cycle\n", __FUNCTION__);  
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Locking device mutex\n", __FUNCTION__);  
#endif
      
            mutex_lock(&wb->device_mutex);
            context->state = cycle;
      j = i;
      // Process EB records: write EB packet to EB slave
      for (left = size; left >= 4; left -= sizeof(wb_data_t)) {
        wops->write_ebs(wb, EB_RX_FIFO_DATA, eb_to_cpu(buf+j));
        j = RING_INDEX(j + sizeof(wb_data_t));
#if DBG_PRINT_LVL > 3  
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: wrote to EB= 0x%08X\n", __FUNCTION__, eb_to_cpu(buf+j));  
#endif
        
      }
      
      mb();
      
      j = i;
      // read EB Slave response
      for (left = size; left >= 4; left -= sizeof(wb_data_t)) {
        eb_from_cpu(buf+j, wops->read_ebs(wb, EB_TX_FIFO_DATA));
        j = RING_INDEX(j + sizeof(wb_data_t));
#if DBG_PRINT_LVL > 3  
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: read from EB= 0x%08X\n", __FUNCTION__, buf+j);  
#endif
      }

            context->state = idle;
#if DBG_PRINT_LVL > 3  
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Unlocking device mutex\n", __FUNCTION__);  
#endif
//      wops->cycle_ebs(wb, 0); // closing cycle in char_master_release
            mutex_unlock(&wb->device_mutex);

#if DBG_PRINT_LVL > -1  
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: pre RING_POS: size=%u left=%u processed=%d\n", 
     __FUNCTION__, size, left, context->processed);  
#endif
    context->processed = RING_POS(context->processed + size - left);
  
#if DBG_PRINT_LVL > -1  
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: END sent=%d processed=%d received=%d\n", 
     __FUNCTION__, context->sent, context->processed, context->received);  
#endif


        } else {
            printk(KERN_DEBUG WB_NAME ": %s: process_ebs dead END. Something went wrong!\n\n", __FUNCTION__); 
        }

}

// ######################################################################
// EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ######################################################################







static int char_master_open(struct inode *inode, struct file *filep)
{
    struct etherbone_master_context *context;
  struct wishbone *wb;

  FNAME
  printk(KERN_DEBUG WB_NAME ": %s: START\n", __FUNCTION__); 
  
    context = kmalloc(sizeof(struct etherbone_master_context), GFP_KERNEL);
    if (!context) return -ENOMEM;

    context->wishbone = container_of(inode->i_cdev, struct wishbone, master_cdev);
  wb = context->wishbone;

    mutex_init(&context->context_mutex);
    context->state = header;
    context->sent = 0;
    context->processed = 0;
    context->received = 0;

    context->msi_unread = 0;
    context->msi_pending = 0;

    context->fasync = 0;
    init_waitqueue_head(&context->waitq);

    context->msi_index = -1;

    filep->private_data = context;


  if (selebslv){
    mutex_lock(&context->context_mutex);
    mutex_lock(&wb->device_mutex);
    wb->wops->cycle_ebs(wb, 1);
    
    
    mutex_unlock(&wb->device_mutex);
    
    // next expect EB header
    context->state = header;
    mutex_unlock(&context->context_mutex);
  }

    return 0;
}

static int char_master_release(struct inode *inode, struct file *filep)
{
    unsigned long flags;
    struct etherbone_master_context *context = filep->private_data;
    struct wishbone *wb;

        FNAME

  wb = context->wishbone;

    /* Did the bad user forget to drop the cycle line? */
    //if (context->state == cycle) {
      mutex_lock(&context->context_mutex);
      mutex_lock(&wb->device_mutex);
      if (selebslv){
        wb->wops->cycle_ebs(wb, 0);
      }else{
        wb->wops->cycle(wb, 0);
      }
      context->state = idle;    
      mutex_unlock(&wb->device_mutex);
      mutex_unlock(&context->context_mutex);
    //}


    /* Do not destroy ourselves while an MSI is inflight to us */
    mutex_lock(&wb->msi_mutex);
    spin_lock_irqsave(&wb->msi_spinlock, flags);
    if (context->msi_index != -1)
        wb->msi_map[context->msi_index] = 0;
    
    context->msi_index = -1;
    spin_unlock_irqrestore(&wb->msi_spinlock, flags);
    mutex_unlock(&wb->msi_mutex);

    /* At this point, we know wishbone_dispatch_msi won't call into us */
    /* Furthermore, we have the last handle as it's being freed, so we
     * implicitly hold context_mutex (don't really hold it during kfree!)
     */

    /* Finish any unhandled MSI */
    if (context->msi_pending) {
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Finish any unhandled MSI\n", __FUNCTION__);
        mutex_lock(&wb->device_mutex);
        context->msi_pending = 0;
        wb->msi_pending = 0;
        wb->wops->reply(wb, 1, ~(wb_data_t)0);
        mutex_unlock(&wb->device_mutex);
        wishbone_slave_ready(wb);
    }

    kfree(context);
    
    printk(KERN_DEBUG WB_NAME ": %s: END\n", __FUNCTION__); 
    return 0;
}

/* Must be called with context_mutex held */
static int deliver_msi(struct etherbone_master_context* context)
{
  FNAME
  
#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context : msi_unread=%d received=%d processed=%d sent=%d\n", 
      __FUNCTION__, context->msi_unread, context->received, context->processed, context->sent);
#endif      
  
    return    context->msi_unread > 0             &&
        context->sent == context->processed &&
        context->sent == context->received;
}

static ssize_t char_master_aio_read(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
    struct file *filep = iocb->ki_filp;
    struct etherbone_master_context *context = filep->private_data;
    unsigned int len, iov_len, ring_len, buf_len;
  
    FNAME

    iov_len = iov_length(iov, nr_segs);
    if (unlikely(iov_len == 0)) return 0;

    if (mutex_lock_interruptible(&context->context_mutex))
        return -EINTR;

    /* If MSI is pending, deliver it */
    if (deliver_msi(context)) {
        /* We don't need a lock here, because no one will write to the msi_unread or
         * msi[] while msi_pending stays high.
         */
#if DBG_PRINT_LVL > -1    
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: If MSI is pending, delivering it\n", __FUNCTION__); 
#endif
        len = min_t(unsigned int, context->msi_unread, iov_len);
        memcpy_toiovecend(iov, context->msi + sizeof(context->msi) - context->msi_unread, 0, len);
        context->msi_unread -= len;
    } else {

#if DBG_PRINT_LVL > -1    
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: If MSI not pending\n", __FUNCTION__); 
#endif
        ring_len = RING_READ_LEN(context);
        len = min_t(unsigned int, ring_len, iov_len);

        /* How far till we must wrap?  */
        buf_len = sizeof(context->buf) - RING_INDEX(context->sent);

#if DBG_PRINT_LVL > -1
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: 1 context : msi_unread=%d received=%d processed=%d sent=%d\n", __FUNCTION__,
      context->msi_unread, context->received, context->processed, context->sent);
#endif      
      
        if (buf_len < len) {
#if DBG_PRINT_LVL > -1
            if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: buf_len (%d) < len (%d)\n", __FUNCTION__, buf_len, len); 
#endif            
            memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, buf_len);
            memcpy_toiovecend(iov, &context->buf[0],            buf_len, len-buf_len);
        } else {
#if DBG_PRINT_LVL > -1
            if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: buf_len (%d) >= len (%d)\n", __FUNCTION__, buf_len, len); 
#endif
            memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, len);
        }
        context->sent = RING_POS(context->sent + len);
#if DBG_PRINT_LVL > -1
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: 2 context : msi_unread=%d received=%d processed=%d sent=%d\n", __FUNCTION__,
            context->msi_unread, context->received, context->processed, context->sent);    
#endif      
    }

    mutex_unlock(&context->context_mutex);

    /* Wake-up polling descriptors */
    wake_up_interruptible(&context->waitq);
    kill_fasync(&context->fasync, SIGIO, POLL_OUT);

#if DBG_PRINT_LVL > 0      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Eaioread msi_unread=%d received=%d processed=%d sent=%d\n", __FUNCTION__,
      context->msi_unread, context->received, context->processed, context->sent);
#endif

    if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
        return -EAGAIN;

    return len;
}



static ssize_t char_master_aio_write(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
    struct file *filep = iocb->ki_filp;
    struct etherbone_master_context *context = filep->private_data;
    unsigned int len, iov_len, ring_len, buf_len;
  
    FNAME
//  msleep(1);
  
    iov_len = iov_length(iov, nr_segs);
    if (unlikely(iov_len == 0)) return 0;

#if DBG_PRINT_LVL > -1    
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Getting context mutex: mutex_lock_interruptible\n", __FUNCTION__); 
#endif
  
    if (mutex_lock_interruptible(&context->context_mutex))
        return -EINTR;

#if DBG_PRINT_LVL > -1    
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Got context mutex\n", __FUNCTION__); 
#endif

    ring_len = RING_WRITE_LEN(context);
    len = min_t(unsigned int, ring_len, iov_len);

    /* How far till we must wrap?  */
    buf_len = sizeof(context->buf) - RING_INDEX(context->received);

    if (buf_len < len) {
#if DBG_PRINT_LVL > -1
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: buf_len < len\n", __FUNCTION__); 
#endif

        memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, buf_len);
        memcpy_fromiovecend(&context->buf[0],                iov, buf_len, len-buf_len);
    } else {
#if DBG_PRINT_LVL > -1        
        if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: buf_len >= len\n", __FUNCTION__); 
#endif
        memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, len);
    }
    context->received = RING_POS(context->received + len);

    /* Process buffers */
  if (selebslv){
#if DBG_PRINT_LVL > -1      
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: calling etherbone_master_process_ebs\n", __FUNCTION__); 
#endif    
    etherbone_master_process_ebs(context);
#if DBG_PRINT_LVL > -1    
    if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: finished etherbone_master_process_ebs\n", __FUNCTION__); 
#endif

   } else {
     etherbone_master_process(context);
   }

    mutex_unlock(&context->context_mutex);

    /* Wake-up polling descriptors */
#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context : msi_unread=%d received=%d processed=%d sent=%d\n", 
      __FUNCTION__, context->msi_unread, context->received, context->processed, context->sent);
#endif      
    
#if DBG_PRINT_LVL > -1        
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Call wake_up_interruptible\n", __FUNCTION__); 
#endif
    wake_up_interruptible(&context->waitq);
#if DBG_PRINT_LVL > -1        
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Call kill_fasync\n", __FUNCTION__); 
#endif
    kill_fasync(&context->fasync, SIGIO, POLL_IN);

#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context : msi_unread=%d received=%d processed=%d sent=%d\n", 
      __FUNCTION__, context->msi_unread, context->received, context->processed, context->sent);
#endif      
    
    if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
        return -EAGAIN;

#if DBG_PRINT_LVL > -1        
  if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: len=%d\n", __FUNCTION__, len); 
#endif
    return len;
}

static unsigned int char_master_poll(struct file *filep, poll_table *wait)
{
    unsigned int mask = 0;
    struct etherbone_master_context *context = filep->private_data;
  
    FNAME

#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Pre poll_wait\n", __FUNCTION__);
#endif 
#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context : msi_unread=%d received=%d processed=%d sent=%d\n", 
      __FUNCTION__, context->msi_unread, context->received, context->processed, context->sent);
#endif      

    poll_wait(filep, &context->waitq, wait);
 
#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: Post poll_wait\n", __FUNCTION__);
#endif      
#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context : msi_unread=%d received=%d processed=%d sent=%d\n", 
      __FUNCTION__, context->msi_unread, context->received, context->processed, context->sent);
#endif      

    mutex_lock(&context->context_mutex);

    if (deliver_msi(context))         mask |= POLLIN  | POLLRDNORM;
    if (RING_READ_LEN (context) != 0) mask |= POLLIN  | POLLRDNORM;
    if (RING_WRITE_LEN(context) != 0) mask |= POLLOUT | POLLWRNORM;

    mutex_unlock(&context->context_mutex);

#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: mask=%08X\n", 
      __FUNCTION__, mask);
#endif      
#if DBG_PRINT_LVL > -1      
      if (unlikely(debug)) printk(KERN_DEBUG WB_NAME ": %s: context : msi_unread=%d received=%d processed=%d sent=%d\n", 
      __FUNCTION__, context->msi_unread, context->received, context->processed, context->sent);
#endif      

    return mask;
}

static int char_master_fasync(int fd, struct file *file, int on)
{
    struct etherbone_master_context* context = file->private_data;
    FNAME

        /* No locking - fasync_helper does its own locking */
        return fasync_helper(fd, file, on, &context->fasync);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,1,0)

static ssize_t char_master_aio_read_iter(struct kiocb *iocb, struct iov_iter *iter)
{
    FNAME
    return char_master_aio_read(iocb, iter->iov, iter->nr_segs, iter->iov_offset);
}

static ssize_t char_master_aio_write_iter(struct kiocb *iocb, struct iov_iter *iter)
{
    FNAME
    return char_master_aio_write(iocb, iter->iov, iter->nr_segs, iter->iov_offset);
}

#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,1,0)

static const struct file_operations etherbone_master_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .read           = do_sync_read,
        .aio_read       = char_master_aio_read,
        .write          = do_sync_write,
        .aio_write      = char_master_aio_write,
        .open           = char_master_open,
        .poll           = char_master_poll,
        .release        = char_master_release,
        .fasync         = char_master_fasync,
};

#else

static const struct file_operations etherbone_master_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        //.read           = new_sync_read,
        .read_iter      = char_master_aio_read_iter,
        //.write          = new_sync_write,
        .write_iter     = char_master_aio_write_iter,
        .open           = char_master_open,
        .poll           = char_master_poll,
        .release        = char_master_release,
        .fasync         = char_master_fasync,
};

#endif

//ssize_t (*read_iter) (struct kiocb *, struct iov_iter *);
//ssize_t (*write_iter) (struct kiocb *, struct iov_iter *);

int wishbone_register(struct wishbone* wb)
{
    struct list_head *list_pos;
    unsigned int devoff, i;
    char workqueue_name[40];
  
    FNAME

    mutex_init(&wb->device_mutex);
    mutex_init(&wb->msi_mutex);
    wb->msi_pending = 0;

    spin_lock_init(&wb->msi_spinlock);
    for (i = 0; i < WISHBONE_MAX_MSI_OPEN; ++i) {
        wb->msi_map[i] = 0;
    }

    /* Grab mutex for insertion of device into global driver list */
    mutex_lock(&wishbone_mutex);

    /* Search the list for gaps, stopping past the gap.
     * If we overflow the list (ie: not gaps), minor already points past end.
     */
    devoff = 0;
    list_for_each(list_pos, &wishbone_list) {
        struct wishbone *entry =
            container_of(list_pos, struct wishbone, list);

        dev_t master_dev_tmp =
          MKDEV(
            MAJOR(wishbone_master_dev_first),
            MINOR(wishbone_master_dev_first) + devoff);

        if (entry->master_dev != master_dev_tmp) {
            /* We found a gap! */
            break;
        } else {
            /* Run out of minors? */
            if (devoff == max_devices-1) goto fail_out;

            /* Try the next minor */
            ++devoff;
        }
    }

    /* Select the free device minor */
    wb->master_dev =
      MKDEV(
        MAJOR(wishbone_master_dev_first),
        MINOR(wishbone_master_dev_first) + devoff);

    /* Connect the file operations with the cdev */
    cdev_init(&wb->master_cdev, &etherbone_master_fops);
    wb->master_cdev.owner = wb->wops->owner;
    if (cdev_add(&wb->master_cdev, wb->master_dev, 1)) goto fail_out;

    /* Create the sysfs entry */
    wb->master_device = device_create(wishbone_master_class, wb->parent, wb->master_dev, NULL, "wbm%d", devoff);
    if (IS_ERR(wb->master_device)) goto fail_master_cdev;

    /* Prepare the MSI dispatcher for being queued */
    INIT_WORK(&wb->msi_handler, &wishbone_dispatch_msi);
    /* Maybe for older kernels?: */
    /* INIT_WORK(&wb->msi_handler, &wishbone_dispatch_msi, &wb->msi_handler); */

    /* Create a workqueue for processing MSIs (in-order) */
    snprintf(workqueue_name, sizeof(workqueue_name), "wishbone/msi_wbm%d", devoff);
    wb->msi_workqueue = create_singlethread_workqueue(workqueue_name);
    if (!wb->msi_workqueue) goto fail_master_dev;

    /* Insert the device into the sorted */
    INIT_LIST_HEAD(&wb->list);
    list_add_tail(&wb->list, list_pos);

    mutex_unlock(&wishbone_mutex);

    /* Startup the MSI queue */
    wishbone_slave_ready(wb);

    return 0;

fail_master_dev:
    device_destroy(wishbone_master_class, wb->master_dev);
fail_master_cdev:
    cdev_del(&wb->master_cdev);
fail_out:
    mutex_unlock(&wishbone_mutex);
    return -ENOMEM;
}

int wishbone_unregister(struct wishbone* wb)
{
    FNAME
    if (WARN_ON(list_empty(&wb->list)))
        return -EINVAL;

    mutex_lock(&wishbone_mutex);

    list_del(&wb->list);
    flush_workqueue(wb->msi_workqueue);
    destroy_workqueue(wb->msi_workqueue);
    device_destroy(wishbone_master_class, wb->master_dev);
    cdev_del(&wb->master_cdev);

    mutex_unlock(&wishbone_mutex);

    return 0;
}

void wishbone_slave_ready(struct wishbone* wb)
{
  FNAME
  
    queue_work(wb->msi_workqueue, &wb->msi_handler);
}

static int __init wishbone_init(void)
{
    int err;
    dev_t overflow;
    FNAME

    //printk(KERN_NOTICE "wishbone: version " __stringify(GIT_REVISION) " loaded\n");
    printk(KERN_NOTICE WB_NAME " : %s : version %s loaded\n", __FUNCTION__, __stringify(GIT_REVISION));
  printk(KERN_NOTICE WB_NAME " : %s : debug print level=%d\n", __FUNCTION__, DBG_PRINT_LVL);

    overflow = MKDEV(0, max_devices-1);
    if (MINOR(overflow) != max_devices-1) {
        err = -ENOMEM;
        goto fail_last;
    }

    wishbone_master_class = class_create(THIS_MODULE, "wbm");
    if (IS_ERR(wishbone_master_class)) {
        err = PTR_ERR(wishbone_master_class);
        goto fail_last;
    }

    if (alloc_chrdev_region(&wishbone_master_dev_first, 0, max_devices, "wbm") < 0) {
        err = -EIO;
        goto fail_master_class;
    }

    return 0;

fail_master_class:
    class_destroy(wishbone_master_class);
fail_last:
    return err;
}

static void __exit wishbone_exit(void)
{
    FNAME
    unregister_chrdev_region(wishbone_master_dev_first, max_devices);
    class_destroy(wishbone_master_class);
}

MODULE_AUTHOR("Wesley W. Terpstra <w.terpstra@gsi.de>");
MODULE_DESCRIPTION("Wishbone character device class");
module_param(max_devices, int, 0644);
MODULE_PARM_DESC(max_devices, "Maximum number of attached wishbone devices");

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debugging information");

module_param(debug_cyc_time, int, 0644);
MODULE_PARM_DESC(debug_cyc_time, "Enable debugging information");

module_param(debug_ucpy, int, 0644);
MODULE_PARM_DESC(debug_ucpy, "Enable debug printout of the data movement between user and kernel space");

module_param(debug_skip_cyc_drop, int, 0644);
MODULE_PARM_DESC(debug_skip_cyc_drop, "Enable skipping of the cycle line drop");


module_param(debug_abs_max_cyc_op_time, int, 0644);
MODULE_PARM_DESC(debug_abs_max_cyc_op_time, "Readout of the max cycle time");

module_param(debug_clear_cycle_time, int, 0644);
MODULE_PARM_DESC(debug_clear_cycle_time, "Enable max cycle time clear after each cycle");

module_param(selebslv, int, 0644);
MODULE_PARM_DESC(selebslv, "Enable EB slave process");

module_param(polldly, long, 0644);
MODULE_PARM_DESC(polldly, "Delay in char_master_poll [us]");





MODULE_LICENSE("GPL");
MODULE_VERSION(WISHBONE_VERSION);

EXPORT_SYMBOL(wishbone_register);
EXPORT_SYMBOL(wishbone_unregister);
EXPORT_SYMBOL(wishbone_slave_ready);

module_init(wishbone_init);
module_exit(wishbone_exit);
