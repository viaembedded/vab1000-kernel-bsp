

#include <asm/page.h>
#include <asm/cacheflush.h>

#define CROSS4K_MAGIC 0x43524f53  /* CROS */
#define CROSS4K_DEBUG 1
#if CROSS4K_DEBUG
static int alloc_size = 0;
module_param_named(cross_allco_size, alloc_size, uint, S_IRUGO);
MODULE_PARM_DESC(cross_allco_size, "Elite cross4k allocate size");

static int Tnum = 0;
module_param_named(cross_num, Tnum, uint, S_IRUGO);
MODULE_PARM_DESC(cross_num, "Elite cross4k num");

static int sg_num = 0;
module_param_named(cross_sg_num, sg_num, uint, S_IRUGO);
MODULE_PARM_DESC(cross_sg_num, "Elite cross4k sg num");

static int sg_size = 0;
module_param_named(cross_sg_size, sg_size, uint, S_IRUGO);
MODULE_PARM_DESC(cross_sg_size, "Elite cross4k sg size");

#endif



typedef struct elite_cross4k_dma_s{
    int size;
    
    char * buffer;
    dma_addr_t phy_buffer;

    struct scatterlist old_sg;
    struct scatterlist *old_sg_p;

    char * old_buffer;
    dma_addr_t old_phy_buffer;
}elite_cross4k_dma_t;

typedef struct elite_cross4k_s {
    u32 magic;
    int total_num;
    dma_addr_t old_point;

    elite_cross4k_dma_t  dma_buffer[0];
}elite_cross4k_t;

#define page_rest_size(buffer) \
    (((~((u32)buffer)) + 1) & (PAGE_SIZE - 1)) 


    
#define  elite_cross4k_check(page_rest_size, len, maxp) \
    ((len > page_rest_size) && ((len < maxp)||(page_rest_size%maxp)))



static elite_cross4k_t *get_urb_patch_p(struct urb * urb)
{
    elite_cross4k_t *p;

    if((urb->num_sgs > 0)&&(urb->transfer_buffer_length > 0)) {
        p = (elite_cross4k_t *)urb->transfer_dma;
    }else
        p = (elite_cross4k_t *)urb->sg;
    if(virt_addr_valid(p)&&(p->magic == CROSS4K_MAGIC))
        return p;
    return NULL;
}
    
/*
    flag = 1 :  urb ---> elite_cross4k_t   backup
    flag = 0 :  elite_cross4k_t ---> urb   restore
*/
static void set_urb_patch_p(struct urb * urb, elite_cross4k_t * p, int flag)
{

    if((urb->num_sgs > 0)&&(urb->transfer_buffer_length > 0)) {
        if(flag ){                               
                p->old_point = urb->transfer_dma;    /* backup*/
                urb->transfer_dma = (dma_addr_t)p;
        }else {                               /*restore*/  
                urb->transfer_dma = p->old_point;
        }
    }else{
       if( flag ) {                                 
            p->old_point =  (dma_addr_t)urb->sg;  /* backup*/
            urb->sg = (struct scatterlist *)p;
       }else {                              /*restore*/ 
            urb->sg = (struct scatterlist *)p->old_point;
       }
       
    }
 }


static void elite_free_buffer(struct usb_device *dev, size_t size, void* addr,
			 dma_addr_t dma)
{
      
      if(size <= PAGE_SIZE/2){
             usb_free_coherent(dev, size, addr, dma);
      } else {
            dma_unmap_single(bus_to_hcd(dev->bus)->self.controller, 
                                dma, size, DMA_TO_DEVICE);
            free_pages((unsigned long)addr, get_order(size));
    }
}

static void* elite_alloc_buffer(struct usb_device *dev, size_t size, gfp_t mem_flags,
			 dma_addr_t *dma)
{
      void * buffer;
      
      if(size <= PAGE_SIZE/2){
            buffer =  usb_alloc_coherent(dev, size, mem_flags, dma);
      } else {
            buffer = (void*)__get_free_pages(mem_flags|__GFP_DMA, get_order(size));
            if(buffer){
                *dma = dma_map_single(bus_to_hcd(dev->bus)->self.controller, 
                                buffer, size,DMA_TO_DEVICE);
                if (dma_mapping_error(bus_to_hcd(dev->bus)->self.controller, *dma)) {
				free_pages((unsigned long)buffer, get_order(size));
                            return NULL;
	         }
           }
    }
    return buffer;
}



static  int elite_cross4k_alloc_dma_sg(struct usb_device *dev, elite_cross4k_dma_t * dma_buffer, struct scatterlist *sg, 
                                                                     int max_packet, gfp_t mem_flags)
{
      int len;
      dma_buffer->buffer = elite_alloc_buffer(dev, sg->length, mem_flags, &(dma_buffer->phy_buffer));

      
      if( dma_buffer->buffer == NULL) {
            printk(KERN_WARNING"elite_cross4k: Fail to allocate dma SG buffer (len = %d)\n", sg->length);
            return -ENOMEM;
      }

      
#if CROSS4K_DEBUG
        alloc_size += sg->length;
#endif
      dma_buffer->old_sg = *sg;
      dma_buffer->old_sg_p = sg;
      dma_buffer->size = sg->length;
      len = sg_copy_to_buffer(sg, 1, dma_buffer->buffer, sg->length);
      if(sg->length > PAGE_SIZE/2) {
            dmac_flush_range(dma_buffer->buffer, dma_buffer->buffer + sg->length);
            outer_flush_range(__pa(dma_buffer->buffer), __pa(dma_buffer->buffer) + sg->length);
      }

      sg_set_buf(sg, dma_buffer->buffer, sg->length);
      sg->dma_address = dma_buffer->phy_buffer;

      if(len != sg->length){
            printk(KERN_WARNING"elite_cross4k: Fail to copy SG buffer (len = %d , read %d)\n", 
                        sg->length, len);
            return -EFAULT;
      }
      return 0;

}

static int elite_cross4k_alloc_dma(struct usb_device *dev, elite_cross4k_dma_t * dma_buffer, void* buffer, 
                                                                        dma_addr_t phy, int len, int max_packet, gfp_t mem_flags)
{
      dma_buffer->buffer = elite_alloc_buffer(dev, len, mem_flags, &(dma_buffer->phy_buffer));        
      if( dma_buffer->buffer == NULL) {
            printk(KERN_WARNING"elite_cross4k: Fail to allocate dma buffer (len = %d)\n", len);
            return -ENOMEM;
      }


#if CROSS4K_DEBUG
        alloc_size += len;
#endif
      dma_buffer->old_sg_p = NULL;
      dma_buffer->old_buffer =  buffer;
      dma_buffer->old_phy_buffer = phy;
      dma_buffer->size = len;
      memcpy(dma_buffer->buffer, buffer, len);
      if(len > PAGE_SIZE/2) {
            dmac_flush_range(dma_buffer->buffer, dma_buffer->buffer + len);
            outer_flush_range(__pa(dma_buffer->buffer), __pa(dma_buffer->buffer) + len);
      }

     
      return 0;
      
        
}


void elite_destroy_cross4k(struct urb * urb)
{
        elite_cross4k_t * pcross4k;
        int i;

        pcross4k = get_urb_patch_p(urb);

        if(pcross4k == NULL)return;


        for(i = 0; i < pcross4k->total_num; i++){
               if(pcross4k->dma_buffer[i].buffer) {
                    if(pcross4k->dma_buffer[i].old_sg_p){
                            *pcross4k->dma_buffer[i].old_sg_p = pcross4k->dma_buffer[i].old_sg;
                    }else {
                        urb->transfer_buffer = pcross4k->dma_buffer[0].old_buffer;
                        urb->transfer_dma =  pcross4k->dma_buffer[0].old_phy_buffer;
                    }

#if CROSS4K_DEBUG
                        alloc_size -= pcross4k->dma_buffer[i].size;
#endif
                        elite_free_buffer(urb->dev, pcross4k->dma_buffer[i].size, 
                                pcross4k->dma_buffer[i].buffer, pcross4k->dma_buffer[i].phy_buffer);
  
               }
        }
        set_urb_patch_p(urb, pcross4k, 0);
#if CROSS4K_DEBUG
       alloc_size -= sizeof(elite_cross4k_t)+pcross4k->total_num*sizeof(elite_cross4k_dma_t);
#endif
        kfree(pcross4k);
}




int elite_create_cross4k(struct urb * urb, gfp_t mem_flags)
{
            int max_n = 0;
            elite_cross4k_t * p;
            int size;
            u32 buffer;
            u32 length;
            int maxp;
            int n;
            int ret = 0;
            struct scatterlist *sg;
            
            if(usb_urb_dir_in(urb) || (urb->transfer_buffer_length == 0))
                return 0;

            maxp = usb_maxpacket(urb->dev, urb->pipe, 1) & 0x07ff;

             /*check urb all out data buffer */
            if (urb->num_sgs){
                for_each_sg(urb->sg, sg, urb->num_sgs, n) {
                    if(unlikely(elite_cross4k_check(page_rest_size(sg->offset), maxp,  sg->length)))
                        max_n++;
                }
            }else if(urb->sg){
                 WARN_ON(1);
                 return 0;                 
            }else{
                buffer = (u32)urb->transfer_buffer;
                length = urb->transfer_buffer_length;
                if(unlikely(elite_cross4k_check(page_rest_size(buffer), maxp, length)))
                    max_n = 1;
            }
            if(max_n == 0) return 0;


            size = sizeof(elite_cross4k_t)+max_n*sizeof(elite_cross4k_dma_t);
            p = (elite_cross4k_t *)kmalloc(size, mem_flags);                        
            if( p == NULL) {
                printk(KERN_WARNING"elite_cross4k: Fail to allocate elite_cross4k_t struct (size = %d)\n", size);
                return -ENOMEM;
            }
#if CROSS4K_DEBUG
        Tnum ++; 
        alloc_size += size;
#endif
            memset(p, 0, size);
            p->magic = CROSS4K_MAGIC;
            p->total_num = max_n;
            /*replace cross 4k dma buffer*/
            if (urb->num_sgs){
                max_n = 0;
                for_each_sg(urb->sg, sg, urb->num_sgs, n) {
                    if(unlikely(elite_cross4k_check(((~sg->offset) & (PAGE_SIZE - 1)) + 1, maxp,  sg->length))){
                        ret = elite_cross4k_alloc_dma_sg(urb->dev, &(p->dma_buffer[max_n]),  sg, maxp, mem_flags);
                        if(ret !=0 )break;
                        max_n++;

#if CROSS4K_DEBUG
                        sg_num ++;
                        sg_size += sg->length; 
#endif
                     }
                }
            }else if(urb->sg){
                BUG_ON(1);
            }else{
                buffer = (u32)urb->transfer_buffer;
                length = urb->transfer_buffer_length;
                ret = elite_cross4k_alloc_dma(urb->dev, &(p->dma_buffer[0]),  urb->transfer_buffer, urb->transfer_dma, length, maxp, mem_flags);
                if(ret == 0) {
                    urb->transfer_buffer = p->dma_buffer[0].buffer;
                    urb->transfer_dma =  p->dma_buffer[0].phy_buffer;
                }
            }
            set_urb_patch_p(urb, p, 1);
            
            if(ret != 0) 
                elite_destroy_cross4k(urb);
            
           return ret;
            
}   


