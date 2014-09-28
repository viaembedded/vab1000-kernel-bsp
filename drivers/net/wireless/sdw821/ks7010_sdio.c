/*
 *   Driver for KeyStream, KS7010 based SDIO cards. 
 *   
 *   ks7010_sdio.c
 *   $Id: ks7010_sdio.c 996 2009-09-14 02:54:21Z sekine $
 *
 *   Copyright (C) 2006-2008 KeyStream Corp.
 *   Copyright (C) 2009 Renesas Technology Corp.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it undr the terms of the GNU General Public License version 2 as
 *   published by the Free Sotware Foundation.
 */

#include <linux/workqueue.h>
#include <asm/atomic.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>

#include "ks_wlan.h"
#include "ks_wlan_ioctl.h"
#include "ks_debug.h"
#include "ks_hostif.h"

#define KS7010_FUNC_NUM 1
#define KS7010_IO_BLOCK_SIZE 512
#define KS7010_MAX_CLOCK 25000000

static int reg_net = 0;

static const struct sdio_device_id if_sdio_ids[] = {
	{ SDIO_DEVICE(0x005b, 0x7910) },
	{ SDIO_DEVICE(0x0023, 0x7910) },
	{ /* all zero */ }
};

MODULE_DEVICE_TABLE(sdio, if_sdio_ids);
struct ks_sdio_model {
        int model;
        const char *firmware;
};

static struct ks_sdio_model ks_sdio_models[] = {
        {
                /* ks7010 */
                .model = 0x10,
                .firmware = "ks7010sd.rom",
        },
};



static int ks7910_sdio_probe(struct sdio_func *function, const struct sdio_device_id *device);
static void ks7910_sdio_remove(struct sdio_func *function);
static void ks7010_rw_function(struct work_struct *work);
static int ks7010_sdio_read( ks_wlan_private *priv, unsigned long address, 
			     unsigned char *buffer, unsigned long length );
static int ks7010_sdio_write( ks_wlan_private *priv, unsigned long address, 
			      unsigned char *buffer, unsigned long length );
#ifdef NO_FIRMWARE_CLASS
static char *romfile = ROM_FILE;
module_param(romfile, charp, S_IRUGO);
#endif
/* macro */

#define inc_txqhead(priv) \
        ( priv->tx_dev.qhead = (priv->tx_dev.qhead + 1) % TX_DEVICE_BUFF_SIZE )
#define inc_txqtail(priv) \
        ( priv->tx_dev.qtail = (priv->tx_dev.qtail + 1) % TX_DEVICE_BUFF_SIZE )
#define cnt_txqbody(priv) \
        (((priv->tx_dev.qtail + TX_DEVICE_BUFF_SIZE) - (priv->tx_dev.qhead)) % TX_DEVICE_BUFF_SIZE )

#define inc_rxqhead(priv) \
        ( priv->rx_dev.qhead = (priv->rx_dev.qhead + 1) % RX_DEVICE_BUFF_SIZE )
#define inc_rxqtail(priv) \
        ( priv->rx_dev.qtail = (priv->rx_dev.qtail + 1) % RX_DEVICE_BUFF_SIZE )
#define cnt_rxqbody(priv) \
        (((priv->rx_dev.qtail + RX_DEVICE_BUFF_SIZE) - (priv->rx_dev.qhead)) % RX_DEVICE_BUFF_SIZE )

void ks_wlan_hw_sleep_doze_request(ks_wlan_private *priv)
{
	unsigned char rw_data;
	int retval;

	//printk(KERN_INFO  "ks_wlan_hw_sleep_doze_request \n");

	/* clear request */
	atomic_set(&priv->sleepstatus.doze_request,0);

	if( atomic_read(&priv->sleepstatus.status) == 0){
		rw_data = GCR_B_DOZE;
		retval = ks7010_sdio_write(priv, GCR_B, &rw_data, sizeof(rw_data));
		if(retval){
			printk(KERN_INFO  " error : GCR_B=%02X\n", rw_data);
			goto out;
		}
		//printk(KERN_INFO  "PMG SET!! : GCR_B=%02X\n", rw_data);
		//printk(KERN_INFO "sleep_mode=SLP_SLEEP\n");
		atomic_set(&priv->sleepstatus.status, 1);
 		priv->last_doze = jiffies;
	}
	else{
		//printk(KERN_INFO "sleep_mode=%d\n",priv->sleep_mode);
	}

out:
	priv->sleep_mode = atomic_read(&priv->sleepstatus.status);
	return;
}

void ks_wlan_hw_sleep_wakeup_request(ks_wlan_private *priv)
{
	unsigned char rw_data;
	int retval;

	//printk(KERN_INFO  "ks_wlan_hw_sleep_wakeup_request \n");

	/* clear request */
	atomic_set(&priv->sleepstatus.wakeup_request,0);

	if( atomic_read(&priv->sleepstatus.status) == 1){
		rw_data = WAKEUP_REQ;
		retval = ks7010_sdio_write(priv, WAKEUP, &rw_data, sizeof(rw_data));
		if(retval){
			printk(KERN_INFO  " error : WAKEUP=%02X\n", rw_data);
			goto out;
		}
		//printk(KERN_INFO  "wake up : WAKEUP=%02X\n", rw_data);
		atomic_set(&priv->sleepstatus.status, 0);
 		priv->last_wakeup = jiffies;
		++priv->wakeup_count;
	}
	else{
		//printk(KERN_INFO "sleep_mode=%d\n",priv->sleep_mode);
	}

out:
	priv->sleep_mode = atomic_read(&priv->sleepstatus.status);
	return;
}


void ks_wlan_hw_wakeup_request(ks_wlan_private *priv)
{
	unsigned char rw_data;
	int retval;

	//printk(KERN_INFO  "ks_wlan_hw_wakeup_request \n");
	if(atomic_read(&priv->psstatus.status)==PS_SNOOZE){
		rw_data = WAKEUP_REQ;
		retval = ks7010_sdio_write(priv, WAKEUP, &rw_data, sizeof(rw_data));
		if(retval){
			printk(KERN_INFO  " error : WAKEUP=%02X\n", rw_data);
		}
		//printk(KERN_INFO  "wake up : WAKEUP=%02X\n", rw_data);
 		priv->last_wakeup = jiffies;
		++priv->wakeup_count;
	}
	else{
		//printk(KERN_INFO "psstatus=%d\n",atomic_read(&priv->psstatus.status));
	}
}

int _ks_wlan_hw_power_save(ks_wlan_private *priv)
{
	int rc=0;
	unsigned char rw_data;
	int retval;

	if(priv->reg.powermgt == POWMGT_ACTIVE_MODE)
		return rc;

	if(priv->reg.operation_mode == MODE_INFRASTRUCTURE &&
	   (priv->connect_status & CONNECT_STATUS_MASK)== CONNECT_STATUS){

		//printk(KERN_INFO 1,"psstatus.status=%d\n",atomic_read(&priv->psstatus.status));
	if (priv->dev_state == DEVICE_STATE_SLEEP) {
		switch(atomic_read(&priv->psstatus.status)){
		case PS_SNOOZE: /* 4 */
			break;
		default:
#if 0
			printk(KERN_INFO "\n\
				psstatus.status=%d\n\
				psstatus.confirm_wait=%d\n\
				psstatus.snooze_guard=%d\n\
				cnt_txqbody=%d\n",
				atomic_read(&priv->psstatus.status),
				atomic_read(&priv->psstatus.confirm_wait),
				atomic_read(&priv->psstatus.snooze_guard),
				cnt_txqbody(priv));
#endif

			if(!atomic_read(&priv->psstatus.confirm_wait)&&
			   !atomic_read(&priv->psstatus.snooze_guard)&&
			   !cnt_txqbody(priv)){
				retval = ks7010_sdio_read(priv, INT_PENDING, &rw_data, sizeof(rw_data));
				if(retval){
					printk(KERN_INFO  " error : INT_PENDING=%02X\n", rw_data);
					queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
					break;
				}
				if(!rw_data){
					rw_data = GCR_B_DOZE;
					retval = ks7010_sdio_write(priv, GCR_B, &rw_data, sizeof(rw_data));
					if(retval){
						printk(KERN_INFO  " error : GCR_B=%02X\n", rw_data);
						queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
						break;
					}
					//printk(KERN_INFO  "PMG SET!! : GCR_B=%02X\n", rw_data);
					atomic_set(&priv->psstatus.status, PS_SNOOZE);
					//printk(KERN_INFO "psstatus.status=PS_SNOOZE\n");
				}
				else{
					queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
				}
			}
			else{
				queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 0);
			}
			break;
		}
	}

	}

	return rc;
}

int ks_wlan_hw_power_save(ks_wlan_private *priv)
{
	queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
	return 0;
}

static int ks7010_sdio_read(ks_wlan_private *priv, unsigned long address, 
			    unsigned char *buffer, unsigned long length)
{
	int rc = -1;
	struct ks_sdio_card *card;

	card = priv->ks_wlan_hw.sdio_card;

	if(length == 1){ /* CMD52 */
		*buffer = sdio_readb(card->func, (unsigned int) address, &rc);
	}		 /* CMD53 */
	else{ /* CMD53 multi-block transfer */
		rc = sdio_memcpy_fromio(card->func, buffer, (unsigned int) address,  (int) length);
	}
	if(rc != 0){
		printk("sdio error erorr=%d size=%ld\n",rc, length);
		++priv->sdio_error_count;
	}else{
		priv->sdio_error_count=0;
	}

	return rc;
}

static int ks7010_sdio_write(ks_wlan_private *priv, unsigned long address,
			     unsigned char *buffer, unsigned long length)
{
	int rc = -1;
	struct ks_sdio_card *card;

	card = priv->ks_wlan_hw.sdio_card;

	if(length == 1){ /* CMD52 */
		sdio_writeb(card->func, *buffer, (unsigned int) address,  &rc);
	}
	else{ 		 /* CMD53 */
		rc = sdio_memcpy_toio(card->func, (unsigned int) address, buffer, length);
	}
	if(rc != 0){
		printk("sdio error erorr=%d size=%ld\n",rc, length);
		++priv->sdio_error_count;
	}else{
		priv->sdio_error_count=0;
	}

	return rc;
}

static int enqueue_txdev(ks_wlan_private *priv, unsigned char *p, unsigned long size,
		  void (*complete_handler)(void *arg1, void *arg2),
		  void *arg1, void *arg2 )
{
	struct tx_device_buffer *sp;

	if (priv->dev_state < DEVICE_STATE_BOOT) {
		kfree(p);
		if (complete_handler != NULL)
			(*complete_handler)(arg1, arg2);
		return 1;
	}

	if ((TX_DEVICE_BUFF_SIZE - 1) <=  cnt_txqbody(priv)) {
		/* in case of buffer overflow */
		printk(KERN_INFO "tx buffer overflow\n");
		kfree(p);
		if (complete_handler != NULL)
			(*complete_handler)(arg1, arg2);
		return 1;
	}

	sp = &priv->tx_dev.tx_dev_buff[priv->tx_dev.qtail];
	sp->sendp = p;
	sp->size = size;
	sp->complete_handler = complete_handler;
	sp->arg1 = arg1;
	sp->arg2 = arg2;
	inc_txqtail(priv);

	return 0;
}

/* write data */
static int write_to_device(ks_wlan_private *priv, unsigned char *buffer, unsigned long size )
{
	int rc,retval;
	unsigned char rw_data;
	struct hostif_hdr *hdr;
	hdr = (struct hostif_hdr *)buffer;
	rc=0;
	
	//printk(KERN_INFO " write_to_device size=%d\n", hdr->size);
	if(hdr->event < HIF_DATA_REQ || HIF_REQ_MAX < hdr->event){
		printk(KERN_INFO "unknown event=%04X\n",hdr->event);
		return 0;
	}

	retval = ks7010_sdio_write(priv, DATA_WINDOW, buffer, size);
	if(retval){
		printk(KERN_INFO  " write error : retval=%d\n", retval);
		return -4;
	}

	rw_data = WRITE_STATUS_BUSY;
	retval = ks7010_sdio_write(priv, WRITE_STATUS, &rw_data, sizeof(rw_data));
	if(retval){
		printk(KERN_INFO  " error : WRITE_STATUS=%02X\n", rw_data);
		return -3;
	}

	return 0;
}

static void tx_device_task(void *dev)
{
	ks_wlan_private	*priv = (ks_wlan_private *)dev;
	struct tx_device_buffer	*sp;
	int rc = 0;

	//printk(KERN_INFO  "tx_device_task\n");
	if(cnt_txqbody(priv)>0 && atomic_read(&priv->psstatus.status) != PS_SNOOZE){
		sp = &priv->tx_dev.tx_dev_buff[priv->tx_dev.qhead];
		if(priv->dev_state >= DEVICE_STATE_BOOT){
			rc = write_to_device(priv, sp->sendp, sp->size);
			if(rc){
				printk(KERN_INFO  "write_to_device error !!(%d)\n", rc);
				queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
				return;
			}

		}
		kfree(sp->sendp); /* allocated memory free */
		if(sp->complete_handler != NULL) /* TX Complete */
			(*sp->complete_handler)(sp->arg1, sp->arg2);		
		inc_txqhead(priv);	
		
		if(cnt_txqbody(priv)>0){
			queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 0);
		}
	}
	return;
}

int ks_wlan_hw_tx( ks_wlan_private *priv, void *p, unsigned long size,
		   void (*complete_handler)(void *arg1, void *arg2),
		   void *arg1, void *arg2 )
{
	int result=0;
	struct hostif_hdr *hdr;
	hdr = (struct hostif_hdr *)p;

	if(hdr->event < HIF_DATA_REQ || HIF_REQ_MAX < hdr->event){
		printk(KERN_INFO "unknown event=%04X\n",hdr->event);
		return 0;
	}

	/* add event to hostt buffer */
	priv->hostt.buff[priv->hostt.qtail] = hdr->event;
        priv->hostt.qtail = (priv->hostt.qtail + 1) % SME_EVENT_BUFF_SIZE;

	//printk(KERN_INFO  "event=%04X\n",hdr->event);
	spin_lock(&priv->tx_dev.tx_dev_lock);
	result = enqueue_txdev(priv, p, size, complete_handler, arg1, arg2);
	spin_unlock(&priv->tx_dev.tx_dev_lock);

	if(cnt_txqbody(priv)>0){
		queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 0);
	}
	return result;
}

static void rx_event_task(unsigned long dev)
{
        ks_wlan_private *priv = (ks_wlan_private *)dev;
	struct rx_device_buffer	*rp;

	//printk(KERN_INFO "rx_event_task\n");

	if(cnt_rxqbody(priv) > 0 && priv->dev_state >= DEVICE_STATE_BOOT){
		rp = &priv->rx_dev.rx_dev_buff[priv->rx_dev.qhead];
		hostif_receive(priv, rp->data, rp->size);
		inc_rxqhead(priv);
		
		if(cnt_rxqbody(priv) > 0){
			tasklet_schedule(&priv->ks_wlan_hw.rx_bh_task);
		}
	}
	
	return;
}

static void ks_wlan_hw_rx(void *dev, uint16_t size)
{
	ks_wlan_private *priv = (ks_wlan_private *)dev;
	int retval;
	struct rx_device_buffer *rx_buffer;
	struct hostif_hdr *hdr;
	unsigned char	read_status;
	unsigned short event=0;

	//printk(KERN_INFO"ks_wlan_hw_rx\n");
	
	/* receive data */
	if (cnt_rxqbody(priv) >= (RX_DEVICE_BUFF_SIZE-1)) {
		/* in case of buffer overflow */
		printk(KERN_INFO"rx buffer overflow \n");
		goto error_out;
	}
	rx_buffer = &priv->rx_dev.rx_dev_buff[priv->rx_dev.qtail];

	retval = ks7010_sdio_read(priv, DATA_WINDOW, &rx_buffer->data[0], hif_align_size(size));
	if(retval){
		goto error_out;
	}

	/* length check */
	if(size > 2046 || size == 0){

		printk(KERN_INFO "-INVAILED DATA dump\n");
		print_buffer(&rx_buffer->data[0],32);

		/* rx_status update */
		read_status = READ_STATUS_IDLE;
		retval = ks7010_sdio_write(priv, READ_STATUS, &read_status, sizeof(read_status));
		if(retval){
			printk(KERN_INFO  " error : READ_STATUS=%02X\n", read_status);
		}
		goto error_out;
	}

	hdr = (struct hostif_hdr *)&rx_buffer->data[0];
	rx_buffer->size = le16_to_cpu(hdr->size) + sizeof(hdr->size);
	event = hdr->event;
	inc_rxqtail(priv);

	/* read status update */
	read_status = READ_STATUS_IDLE;
	retval = ks7010_sdio_write(priv, READ_STATUS, &read_status, sizeof(read_status));
	if(retval){
		printk(KERN_INFO  " error : READ_STATUS=%02X\n", read_status);
	}
	//printk(KERN_INFO  "READ_STATUS=%02X\n", read_status);

	if(atomic_read(&priv->psstatus.confirm_wait)){
		if(IS_HIF_CONF(event)){
			//printk(KERN_INFO  "IS_HIF_CONF true !!\n");
			atomic_dec(&priv->psstatus.confirm_wait);
		}
	}

	/* rx_event_task((void *)priv); */
	tasklet_schedule(&priv->ks_wlan_hw.rx_bh_task);

error_out:
	return;
}

static void ks7010_rw_function(struct work_struct *work)
{
	struct hw_info_t *hw;
	struct ks_wlan_private *priv;
	unsigned char rw_data;
	int retval;

	hw = container_of(work, struct hw_info_t, rw_wq.work);
	priv = container_of(hw, struct ks_wlan_private, ks_wlan_hw);

	//printk(KERN_INFO "ks7010_rw_function \n");


 	/* wiat after DOZE */
 	if(time_after(priv->last_doze + ((30*HZ)/1000), jiffies )){
 		//printk(KERN_INFO  "wait after DOZE \n");
		queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
		return;
	}

	/* wiat after WAKEUP */
	if(time_after(priv->last_wakeup + ((30*HZ)/1000), jiffies )){
		//printk(KERN_INFO  "wait after WAKEUP \n");
		queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
		return;
	}
 
	sdio_claim_host(priv->ks_wlan_hw.sdio_card->func);
	
	/* power save wakeup */
	if(atomic_read(&priv->psstatus.status)==PS_SNOOZE){
		if(cnt_txqbody(priv)>0){
			ks_wlan_hw_wakeup_request(priv);
			queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 1);
		}
		goto err_out;
	}

	/* sleep mode doze */
	if(atomic_read(&priv->sleepstatus.doze_request)==1){
		ks_wlan_hw_sleep_doze_request(priv);
		goto err_out;
	}
	/* sleep mode wakeup */
	if(atomic_read(&priv->sleepstatus.wakeup_request)==1){
		ks_wlan_hw_sleep_wakeup_request(priv);
		goto err_out;
	}

	/* read (WriteStatus/ReadDataSize FN1:00_0014) */
	retval = ks7010_sdio_read(priv, WSTATUS_RSIZE, &rw_data, sizeof(rw_data));
	if(retval){
		printk(KERN_INFO  " error : WSTATUS_RSIZE=%02X psstatus=%d\n", rw_data,atomic_read(&priv->psstatus.status));
		goto err_out;
	}
	//printk(KERN_INFO  "WSTATUS_RSIZE=%02X\n", rw_data);

	if(rw_data&RSIZE_MASK){ /* Read schedule */
		ks_wlan_hw_rx((void *)priv, (uint16_t)(((rw_data&RSIZE_MASK)<<4)));
	}
	if((rw_data&WSTATUS_MASK)){
		tx_device_task((void *)priv);
	}
	_ks_wlan_hw_power_save(priv);

err_out:
	sdio_release_host(priv->ks_wlan_hw.sdio_card->func);

	return;
}



static void ks_sdio_interrupt(struct sdio_func *func)
{
	int retval;
	struct ks_sdio_card *card;
	ks_wlan_private *priv;
	unsigned char status, rsize, rw_data;

	card = sdio_get_drvdata(func);
	priv = card->priv;
	//printk(KERN_INFO  "ks_sdio_interrupt \n");

	if(priv->dev_state >= DEVICE_STATE_BOOT){
		retval = ks7010_sdio_read(priv, INT_PENDING, &status, sizeof(status));
		if(retval){
			printk(KERN_INFO  "read INT_PENDING Failed!!(%d)\n",retval);
			goto intr_out;
		}
		//printk(KERN_INFO  "INT_PENDING=%02X\n", rw_data);

		/* schedule task for interrupt status */
		/* bit7 -> Write General Communication B register */
		/* read (General Communication B register) */
		/* bit5 -> Write Status Idle */
		/* bit2 -> Read Status Busy  */
		if(status&INT_GCR_B || atomic_read(&priv->psstatus.status)==PS_SNOOZE){
			retval = ks7010_sdio_read(priv, GCR_B, &rw_data, sizeof(rw_data));
			if(retval){
				printk(KERN_INFO  " error : GCR_B=%02X\n", rw_data);
				goto intr_out;
			}
			/* printk(KERN_INFO  "GCR_B=%02X\n", rw_data); */
			if(rw_data == GCR_B_ACTIVE){
				if(atomic_read(&priv->psstatus.status)==PS_SNOOZE){
					atomic_set(&priv->psstatus.status, PS_WAKEUP);
					priv->wakeup_count=0;
				}
				complete(&priv->psstatus.wakeup_wait);
			}


		}

		do{
			/* read (WriteStatus/ReadDataSize FN1:00_0014) */
			retval = ks7010_sdio_read(priv, WSTATUS_RSIZE, &rw_data, sizeof(rw_data));
			if(retval){
				printk(KERN_INFO  " error : WSTATUS_RSIZE=%02X\n", rw_data);
				goto intr_out;
			}
			//printk(KERN_INFO  "WSTATUS_RSIZE=%02X\n", rw_data);
			rsize=rw_data&RSIZE_MASK;
			if(rsize){ /* Read schedule */
				ks_wlan_hw_rx((void *)priv, (uint16_t)(((rsize)<<4)));
			}
			if(rw_data&WSTATUS_MASK){
#if 0
				if(status&INT_WRITE_STATUS && !cnt_txqbody(priv)){
					/* dummy write for interrupt clear */
					rw_data =0;
					retval = ks7010_sdio_write(priv, DATA_WINDOW, &rw_data, sizeof(rw_data));
					if (retval) {
						printk(KERN_INFO  "write DATA_WINDOW Failed!!(%d)\n",retval);
					}
					status &= ~INT_WRITE_STATUS;
				}
				else{
#endif
					if(atomic_read(&priv->psstatus.status)==PS_SNOOZE){
						if(cnt_txqbody(priv)){
							ks_wlan_hw_wakeup_request(priv);
							queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq, &priv->ks_wlan_hw.rw_wq, 1);
							return;
						}
					}
					else{
						tx_device_task((void *)priv); 
					}
//				}
			}
		}while(rsize);
	}

intr_out:
	queue_delayed_work(priv->ks_wlan_hw.ks7010sdio_wq,&priv->ks_wlan_hw.rw_wq, 0);
	return; 
}

static int trx_device_init( ks_wlan_private *priv )
{
	/* initialize values (tx) */
	priv->tx_dev.qtail = priv->tx_dev.qhead = 0;

	/* initialize values (rx) */
	priv->rx_dev.qtail = priv->rx_dev.qhead = 0;

	/* initialize spinLock (tx,rx) */
	spin_lock_init(&priv->tx_dev.tx_dev_lock);
	spin_lock_init(&priv->rx_dev.rx_dev_lock);

	tasklet_init(&priv->ks_wlan_hw.rx_bh_task, rx_event_task, (unsigned long)priv);
	
	return 0;
}

static void trx_device_exit( ks_wlan_private *priv )
{
	struct tx_device_buffer	*sp;
	
	/* tx buffer clear */
	while (cnt_txqbody(priv)>0) {
		sp = &priv->tx_dev.tx_dev_buff[priv->tx_dev.qhead];
		kfree(sp->sendp); /* allocated memory free */
		if (sp->complete_handler != NULL) /* TX Complete */
			(*sp->complete_handler)(sp->arg1, sp->arg2);
		inc_txqhead(priv);
	}

	tasklet_kill(&priv->ks_wlan_hw.rx_bh_task);

	return;
}
static int ks7010_sdio_update_index(ks_wlan_private *priv, u32 index)
{
	int rc=0;
	int retval;
	unsigned char *data_buf;
	data_buf = NULL;

	data_buf = kmalloc(sizeof(u32), GFP_KERNEL);
	if(!data_buf){ rc = 1; goto error_out; }

	memcpy(data_buf, &index, sizeof(index));
	retval = ks7010_sdio_write(priv, WRITE_INDEX, data_buf, sizeof(index));
	if(retval){ rc = 2; goto error_out; }

	retval = ks7010_sdio_write(priv, READ_INDEX, data_buf, sizeof(index));
	if(retval){ rc = 3; goto error_out; }
error_out:
	if(data_buf) kfree(data_buf);
	return rc;
}

#define ROM_BUFF_SIZE (64*1024)
static int ks7010_sdio_data_compare(ks_wlan_private *priv, u32 address, 
				    unsigned char *data, unsigned int size)
{
	int rc=0;
	int retval;
	unsigned char *read_buf; 
	read_buf = NULL;
	read_buf = kmalloc(ROM_BUFF_SIZE, GFP_KERNEL);
	if(!read_buf){ rc = 1; goto error_out; }
	retval = ks7010_sdio_read(priv, address, read_buf, size);
	if(retval){ rc = 2; goto error_out; }
	retval = memcmp(data, read_buf, size);

	if(retval){
		printk(KERN_INFO  "data compare error (%d) \n",retval); rc = 3; goto error_out; 
	}
error_out:
	if(read_buf) kfree(read_buf);
	return rc;
}
#ifndef NO_FIRMWARE_CLASS
#include <linux/firmware.h>
#endif
static int ks79xx_upload_firmware(ks_wlan_private *priv, struct ks_sdio_card *card)
{
	unsigned int	size, offset,  n = 0;
	unsigned char	*rom_buf;
	unsigned char rw_data =0;
	int retval, rc=0;
#ifndef NO_FIRMWARE_CLASS
	int length;
	const struct firmware *fw_entry = NULL;
#else
	int orgfsuid, orgfsgid;
	struct file     *srcf;
	mm_segment_t orgfs;
#endif

	rom_buf = NULL;

	/* buffer allocate */
	rom_buf = kmalloc(ROM_BUFF_SIZE, GFP_KERNEL);
	if(!rom_buf){ rc = 3; goto error_out0; }


	sdio_claim_host(card->func);

	/* Firmware running ? */
	retval = ks7010_sdio_read(priv, GCR_A, &rw_data, sizeof(rw_data));
	if(rw_data == GCR_A_RUN){ 
		printk(KERN_INFO "MAC firmware running ...\n"); 
		rc = 0; 
		goto error_out0; 
	}

#ifndef NO_FIRMWARE_CLASS
	if(request_firmware(&fw_entry, priv->reg.rom_file, &priv->ks_wlan_hw.sdio_card->func->dev)!=0){
		printk(KERN_INFO "error request_firmware() file=%s\n", priv->reg.rom_file);
		return 1;
	}
	printk(KERN_INFO "success request_firmware() file=%s size=%d\n", priv->reg.rom_file, fw_entry->size);
	length = fw_entry->size;
#else
	orgfsuid=current->fsuid;
	orgfsgid=current->fsgid;
	current->fsuid=current->fsgid=0;
	orgfs=get_fs();
	set_fs(KERNEL_DS);

	srcf = filp_open(romfile, O_RDONLY, 0);
	if (IS_ERR(srcf)) {
		printk(KERN_INFO  "error %ld opening %s\n", -PTR_ERR(srcf),romfile);
		rc = 1;
		goto error_out1;
	}

        if (!(srcf->f_op && srcf->f_op->read)) {
                printk(KERN_INFO  "%s does not have a read method\n", romfile);
                rc = 2;
                goto error_out2;
        }
#endif

	/* Load Program */
	n = 0;
        do {               
#ifndef NO_FIRMWARE_CLASS
		if(length >= ROM_BUFF_SIZE){
			size = ROM_BUFF_SIZE;
			length = length - ROM_BUFF_SIZE;
		}
		else{
			size=length;
			length=0;
		}
		printk(KERN_INFO  "size = %d\n",size);
		if(size == 0) break;
		memcpy(rom_buf,fw_entry->data+n,size); 
#else
		/* The object must have a read method */
		size = srcf->f_op->read(srcf, rom_buf, ROM_BUFF_SIZE, &srcf->f_pos);
		if (size < 0) {
			printk(KERN_INFO  "Read %s error %d\n", priv->reg.rom_file, -retval);
			rc = 5;
			goto error_out2;
		}
		else if (size == 0) break;
#endif
		/* Update write index */
		offset = n;
		retval = ks7010_sdio_update_index(priv, KS7010_IRAM_ADDRESS+offset);
		if(retval){ rc = 6; goto error_out1; }

		/* Write data */
		retval = ks7010_sdio_write(priv, DATA_WINDOW, rom_buf, size);
		if(retval){ rc = 8; goto error_out1; }

		/* compare */
		retval = ks7010_sdio_data_compare(priv, DATA_WINDOW, rom_buf, size);
		if(retval){ rc = 9; goto error_out1; }
		n += size;

       }while(size);

	/* Remap request */
	rw_data = GCR_A_REMAP;
	retval = ks7010_sdio_write(priv, GCR_A, &rw_data, sizeof(rw_data));
	if(retval){
		rc = 11;
		goto error_out1;
	}
	printk(KERN_INFO " REMAP Request : GCR_A=%02X\n", rw_data);

	/* Firmware running check */
	for (n = 0; n < 50; ++n) {
		mdelay(10);/* wait_ms(10); */
		retval = ks7010_sdio_read(priv, GCR_A, &rw_data, sizeof(rw_data));
		if(retval){ rc = 11; goto error_out1; }
		if(rw_data == GCR_A_RUN) break;
	}
	printk(KERN_INFO  "firmware wakeup (%d)!!!!\n",n);
	if ((50) <= n) {
		printk(KERN_INFO  "firmware can't start\n");
                rc = 12;
		goto error_out1;
	}

	rc = 0;

#ifdef NO_FIRMWARE_CLASS
 error_out2:
	retval=filp_close(srcf ,NULL);
	if (retval)
		printk(KERN_INFO  "error %d closing %s\n", -retval,priv->reg.rom_file);

 error_out1:
	set_fs(orgfs);
	current->fsuid=orgfsuid;
	current->fsgid=orgfsgid;
#else
 error_out1:
	release_firmware(fw_entry);
#endif
 error_out0:
	sdio_release_host(card->func);
	if(rom_buf)
		kfree(rom_buf);
	return rc;
}

static void card_init_task(struct work_struct *work)
{
	struct hw_info_t *hw;
	struct ks_wlan_private *priv;

	hw = container_of(work, struct hw_info_t, init_task);
	priv = container_of(hw, struct ks_wlan_private, ks_wlan_hw);

	printk(KERN_INFO "\ncard_init_task()\n");

	/* init_waitqueue_head(&priv->confirm_wait); */
	init_completion(&priv->confirm_wait);

	printk(KERN_INFO "init_completion()\n");

	/* get mac address & firmware version */
	hostif_sme_enqueue(priv, SME_START);

	printk(KERN_INFO "hostif_sme_enqueu()\n");

	if(!wait_for_completion_interruptible_timeout(&priv->confirm_wait,5*HZ)){
		printk(KERN_INFO "wait time out!! SME_START\n");
	}

	if(priv->mac_address_valid && priv->version_size){
		priv->dev_state = DEVICE_STATE_PREINIT;
	}

	hostif_sme_enqueue(priv, SME_GET_EEPROM_CKSUM);

	/* load initial wireless parameter */
	hostif_sme_enqueue(priv, SME_STOP_REQUEST);

	hostif_sme_enqueue(priv, SME_RTS_THRESHOLD_REQUEST);
	hostif_sme_enqueue(priv, SME_FRAGMENTATION_THRESHOLD_REQUEST);

	hostif_sme_enqueue(priv, SME_WEP_INDEX_REQUEST);
	hostif_sme_enqueue(priv, SME_WEP_KEY1_REQUEST);
	hostif_sme_enqueue(priv, SME_WEP_KEY2_REQUEST);
	hostif_sme_enqueue(priv, SME_WEP_KEY3_REQUEST);
	hostif_sme_enqueue(priv, SME_WEP_KEY4_REQUEST);

	hostif_sme_enqueue(priv, SME_WEP_FLAG_REQUEST);
	hostif_sme_enqueue(priv, SME_RSN_ENABLED_REQUEST);
	hostif_sme_enqueue(priv, SME_MODE_SET_REQUEST);
	hostif_sme_enqueue(priv, SME_START_REQUEST);

	if(!wait_for_completion_interruptible_timeout(&priv->confirm_wait,5*HZ)){
		printk(KERN_INFO "wait time out!! wireless parameter set\n");
	}

	if(priv->dev_state >= DEVICE_STATE_PREINIT){
		printk(KERN_INFO  "DEVICE READY!!\n");
		priv->dev_state = DEVICE_STATE_READY;
		reg_net = register_netdev (priv->net_dev);
		printk(KERN_INFO  "register_netdev=%d\n",reg_net);
	}
	else {
		printk(KERN_INFO  "dev_state=%d\n",priv->dev_state);
	}
}

static struct sdio_driver ks7010_sdio_driver = {
	.name		= "ks7910_sdio",
	.id_table	= if_sdio_ids,
	.probe		= ks7910_sdio_probe,
	.remove		= ks7910_sdio_remove,
};


extern int ks_wlan_net_start(struct net_device *dev);
extern int ks_wlan_net_stop(struct net_device *dev);

static int ks7910_sdio_probe(struct sdio_func *func, const struct sdio_device_id *device)
{
	ks_wlan_private *priv;
	struct ks_sdio_card *card;
	struct net_device *netdev;
	unsigned char rw_data;
	int i=0, ret;

	printk("ks7910_sdio_probe()\n");

	priv = NULL;
	netdev=NULL;


	/* initilize ks_sdio_card */
	card = kzalloc(sizeof(struct ks_sdio_card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->func  = func;
	card->model = 0x10;
	spin_lock_init(&card->lock);

	/* select model */
	for (i = 0;i < ARRAY_SIZE(ks_sdio_models);i++) {
		if (card->model == ks_sdio_models[i].model)
			break;
	}

	if (i == ARRAY_SIZE(ks_sdio_models)) {
		printk(KERN_INFO  "unkown card model 0x%x\n", card->model);
		goto error;
	}

	card->firmware = ks_sdio_models[i].firmware;


	/*** Initialize  SDIO ***/
	sdio_claim_host(func);

	/* bus setting  */
	/* Issue config request to override clock rate */
	
	/* function blocksize set */
	ret = sdio_set_block_size(func, KS7010_IO_BLOCK_SIZE);
	printk(KERN_INFO  "multi_block=%d sdio_set_block_size()=%d %d\n", func->card->cccr.multi_block, func->cur_blksize,  ret);

	/* Allocate the slot current */

	/* function enable */
	ret = sdio_enable_func(func);
	printk(KERN_INFO  "sdio_enable_func() %d\n", ret);
	if (ret)
		goto error_free_card;

	/* interrupt disable */
	sdio_writeb(func, 0, INT_ENABLE,  &ret);
	if (ret)
		goto error_free_card;
	sdio_writeb(func, 0xff, INT_PENDING,  &ret);
	if (ret)
		goto error_disable_func;

	/* setup interrupt handler */
	ret = sdio_claim_irq(func, ks_sdio_interrupt);
	if (ret)
		goto error_disable_func;

	sdio_release_host(func);

	sdio_set_drvdata(func, card);

	printk(KERN_INFO  "class = 0x%X, vendor = 0x%X, "
		"device = 0x%X\n",
		func->class, func->vendor, func->device);
	

	/* private memory allocate */
	netdev = alloc_etherdev(sizeof(*priv));
	if (netdev == NULL) {
		printk (KERN_ERR "ks79xx : Unable to alloc new net device\n");
		goto error_release_irq;
	}
	if (dev_alloc_name(netdev, netdev->name) < 0) {
		printk (KERN_ERR "ks79xx :  Couldn't get name!\n");
		goto error_free_netdev;
	}
	printk(KERN_INFO "[probe]private memory allocate\n");
	priv = kzalloc(sizeof(struct ks_wlan_private), GFP_KERNEL);
	if (!priv)
	{
		printk(KERN_INFO "allocat priv faile\n");	
		return -ENOMEM;
	}		
	netdev->ml_priv = priv;		/*for linux 2.6.35*/
	card->priv = priv;
	SET_NETDEV_DEV(netdev, &card->func->dev);	/* for create sysfs symlinks */
	/* private memory initialize */	
	priv->ks_wlan_hw.sdio_card = card;
	init_completion(&priv->ks_wlan_hw.ks7010_sdio_wait);
	priv->ks_wlan_hw.read_buf = NULL;
	priv->ks_wlan_hw.read_buf = kmalloc(RX_DATA_SIZE, GFP_KERNEL);
	if(!priv->ks_wlan_hw.read_buf){
		goto error_free_netdev;
	}
	priv->dev_state = DEVICE_STATE_PREBOOT;
	priv->net_dev = netdev;
	priv->firmware_version[0] = '\0';
	priv->version_size = 0;
	priv->last_doze = jiffies;	/* set current jiffies */
	priv->last_wakeup = jiffies;
	memset(&priv->nstats, 0, sizeof(priv->nstats));
	memset(&priv->wstats, 0, sizeof(priv->wstats));	
	/* sleep mode */
	atomic_set(&priv->sleepstatus.doze_request,0);
	atomic_set(&priv->sleepstatus.wakeup_request,0);
	atomic_set(&priv->sleepstatus.wakeup_request,0);
	trx_device_init(priv);
	hostif_init(priv);
        ks_wlan_net_start(netdev);
	/* Read config file */
	ret = ks_wlan_read_config_file(priv);
	if (ret) {
		printk(KERN_ERR "ks79xx: read configuration file failed !! retern code = %d\n", ret);
		goto error_free_read_buf;
	}

	/* Upload firmware */
	ret = ks79xx_upload_firmware(priv, card); /* firmware load */
	if(ret){
		printk(KERN_ERR "ks79xx: firmware load failed !! retern code = %d\n", ret);
		goto error_free_read_buf;
	}
	
	/* interrupt setting */
	/* clear Interrupt status write (ARMtoSD_InterruptPending FN1:00_0024) */
	rw_data = 0xff;
	sdio_claim_host(func);
	ret = ks7010_sdio_write(priv, INT_PENDING, &rw_data, sizeof(rw_data));
	sdio_release_host(func);
	if(ret){
		printk(KERN_INFO  " error : INT_PENDING=%02X\n", rw_data);
	}
	printk(KERN_INFO  " clear Interrupt : INT_PENDING=%02X\n", rw_data);


	/* enable ks7010sdio interrupt (INT_GCR_B|INT_READ_STATUS|INT_WRITE_STATUS) */
	rw_data = (INT_GCR_B|INT_READ_STATUS|INT_WRITE_STATUS);
	sdio_claim_host(func);
	ret = ks7010_sdio_write(priv, INT_ENABLE, &rw_data, sizeof(rw_data));
	sdio_release_host(func);
	if(ret){
		printk(KERN_INFO  " error : INT_ENABLE=%02X\n", rw_data);
	}
	printk(KERN_INFO  " enable Interrupt : INT_ENABLE=%02X\n", rw_data);
	priv->dev_state = DEVICE_STATE_BOOT;

	priv->ks_wlan_hw.ks7010sdio_wq = create_workqueue("ks7010sdio_wq");
	if(!priv->ks_wlan_hw.ks7010sdio_wq){
		printk(KERN_INFO  "create_workqueue failed !!\n");
		goto error_free_read_buf;
	}
		printk(KERN_INFO  "create_workqueue ok1!!\n");
	priv->ks_wlan_hw.ks7010sdio_init = create_singlethread_workqueue("ks7010sdio_init");
	if(!priv->ks_wlan_hw.ks7010sdio_init){
		printk(KERN_INFO  "create_workqueue failed !!\n");
		goto error_free_sdio_wq;
	}
		printk(KERN_INFO  "create_workqueue ok2 !!\n");
	INIT_WORK(&priv->ks_wlan_hw.init_task, card_init_task);
		printk(KERN_INFO  "create_workqueue init !!\n");
	INIT_DELAYED_WORK(&priv->ks_wlan_hw.rw_wq, ks7010_rw_function);
		printk(KERN_INFO  "create_workqueue delay!!\n");
	queue_work(priv->ks_wlan_hw.ks7010sdio_init, &priv->ks_wlan_hw.init_task);
		printk(KERN_INFO  "create_workqueue work!!\n");
	return 0;

error_free_sdio_wq:
	flush_workqueue(priv->ks_wlan_hw.ks7010sdio_wq);
	destroy_workqueue(priv->ks_wlan_hw.ks7010sdio_wq);
	priv->ks_wlan_hw.ks7010sdio_wq = NULL;
error_free_read_buf:
	kfree(priv->ks_wlan_hw.read_buf);
	priv->ks_wlan_hw.read_buf = NULL;
error_free_netdev:
	free_netdev(priv->net_dev);
	card->priv = NULL;
error_release_irq:
	sdio_claim_host(func);
	sdio_release_irq(func);
error_disable_func:
	sdio_disable_func(func);
error_free_card:
	sdio_release_host(func);
	sdio_set_drvdata(func, NULL);
	kfree(card);
error:
	return -ENODEV;
}

static void ks7910_sdio_remove(struct sdio_func *func)
{
	int ret;
	struct ks_sdio_card *card;
	struct ks_wlan_private *priv;
	struct net_device *netdev;
	printk(KERN_INFO  "ks7910_sdio_remove()\n");

	card = sdio_get_drvdata(func);

	if(card == NULL)
		return;

	printk(KERN_INFO  "priv = card->priv\n");
	priv = card->priv;
	netdev = priv->net_dev;
	if(priv){
		ks_wlan_net_stop(netdev);
		printk(KERN_INFO  "ks_wlan_net_stop\n");

		/* interrupt disable */
		sdio_claim_host(func);
		sdio_writeb(func, 0, INT_ENABLE,  &ret);
		sdio_writeb(func, 0xff, INT_PENDING,  &ret);
		sdio_release_host(func);
		printk(KERN_INFO  "interrupt disable\n");

		/* send stop request to MAC */
		{
			struct hostif_stop_request_t *pp;
			pp = (struct hostif_stop_request_t *)kzalloc(hif_align_size(sizeof(*pp)), GFP_KERNEL );
			if (pp==NULL) {
				printk(KERN_INFO "allocate memory failed..\n");
				return; /* to do goto ni suru*/
			}
			pp->header.size	= cpu_to_le16((uint16_t)(sizeof(*pp)-sizeof(pp->header.size)));
			pp->header.event = cpu_to_le16((uint16_t)HIF_STOP_REQ);

			sdio_claim_host(func);
			write_to_device(priv, (unsigned char *) pp, hif_align_size(sizeof(*pp)));
			sdio_release_host(func);
			kfree(pp);
		}
		printk(KERN_INFO  "STOP Req\n");

		if(priv->ks_wlan_hw.ks7010sdio_wq){
			flush_workqueue(priv->ks_wlan_hw.ks7010sdio_wq);
			destroy_workqueue(priv->ks_wlan_hw.ks7010sdio_wq);
		}
		printk(KERN_INFO  "destroy_workqueue(priv->ks_wlan_hw.ks7010sdio_wq);\n");

		if(priv->ks_wlan_hw.ks7010sdio_init){
			flush_workqueue(priv->ks_wlan_hw.ks7010sdio_init);
			destroy_workqueue(priv->ks_wlan_hw.ks7010sdio_init);
		}
		printk(KERN_INFO  "destroy_workqueue(priv->ks_wlan_hw.ks7010sdio_init);\n");

		hostif_exit(priv);
		printk(KERN_INFO  "hostif_exit\n");

		if(!reg_net)
			unregister_netdev(netdev);
		printk(KERN_INFO  "unregister_netdev\n");

		trx_device_exit(priv);		
		if(priv->ks_wlan_hw.read_buf){
			kfree(priv->ks_wlan_hw.read_buf);
		}
		free_netdev(priv->net_dev);
		card->priv = NULL;
	}

	sdio_claim_host(func);
	sdio_release_irq(func);
	printk(KERN_INFO  "sdio_release_irq()\n");
	sdio_disable_func(func);
	printk(KERN_INFO  "sdio_disable_func()\n");
	sdio_release_host(func);

	sdio_set_drvdata(func, NULL);

	kfree(card);
	printk(KERN_INFO  "kfree()\n");


	printk(KERN_INFO " Bye !!\n");
	return;
}

static int __init ks7010_sdio_init( void )
{
	int status;
	printk(KERN_INFO "ks7010_sdio : %s %s\n" ,__DATE__,__TIME__);

	/* register with bus driver core */
	status = sdio_register_driver(&ks7010_sdio_driver);
	if(status != 0){
		printk(KERN_INFO "ks79xx_sdio : failed to register with bus driver, %d\n", status );
	}
	else
	{
		printk(KERN_INFO"ks79xx_sdio : success to register with bus driver, %d\n", status );
	}
	return status;
}

static void __exit ks7010_sdio_exit( void )
{
	printk(KERN_INFO "[sdio exit] \n");
	sdio_unregister_driver(&ks7010_sdio_driver);
	return;
}

module_init(ks7010_sdio_init);
module_exit(ks7010_sdio_exit);

MODULE_AUTHOR("KeyStream");
MODULE_DESCRIPTION("Driver for KeyStream, KS7010 based SDIO cards. ");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
MODULE_SUPPORTED_DEVICE("KS7910");
