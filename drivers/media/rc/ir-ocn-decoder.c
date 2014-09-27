/* ir-pg-ocn-decoder.c - handle PZ-OCN IR Pulse/Space protocol
 *
 * Copyright (C) 2013 by S3 Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/bitrev.h>
#include <linux/module.h>
#include "rc-core-priv.h"

#define OCN_USER_NBITS		        24
#define OCN_KEY_NBITS		        16
#define OCN_UNIT		                600000  // ns 
#define OCN_HEADER1_PULSE	        (6 * OCN_UNIT)
#define OCN_HEADER1_SPACE	        (3 * OCN_UNIT)
#define OCN_HEADER2_PULSE	        (6 * OCN_UNIT)
#define OCN_HEADER2_SPACE	        (3 * OCN_UNIT)
#define OCN_REPEAT_SPACE	        (6 * OCN_UNIT)
#define OCN_REPEAT_END_PULSE   (1 * OCN_UNIT)
#define OCN_BIT_PULSE		        (1 * OCN_UNIT)
#define OCN_BIT_0_SPACE		(1 * OCN_UNIT)             
#define OCN_BIT_1_SPACE		(2 * OCN_UNIT)


enum ocn_state {
	STATE_INACTIVE,
	STATE_HEADER1_SPACE,
	STATE_USER_BIT_PULSE,
	STATE_USER_BIT_SPACE,
	STATE_HEADER2_PULSE,
	STATE_HEADER2_SPACE,
	STATE_KEY_BIT_PULSE,
	STATE_KEY_BIT_SPACE,	
	STATE_REPEAT_END_PULSE,
};

/**
 * ir_ocn_decode() - Decode one OCN pulse or space
 * @dev:	the struct rc_dev descriptor of the device
 * @duration:	the struct ir_raw_event descriptor of the pulse/space
 *
 * This function returns -EINVAL if the pulse violates the state machine
 */
static int ir_ocn_decode(struct rc_dev *dev, struct ir_raw_event ev)
{
	struct ocn_dec *data = &dev->raw->ocn;
	u8 usercode0,usercode1,usercode2;
        u32 scancode = 0;

	if (!(dev->raw->enabled_protocols & RC_TYPE_OCN))
		return 0;

	if (!is_timing_event(ev)) {
		if (ev.reset)
			data->state = STATE_INACTIVE;
		return 0;
	}

	IR_dprintk(2, "OCN decode started at state %d (%uus %s)\n",
		   data->state, TO_US(ev.duration), TO_STR(ev.pulse));

	switch (data->state) {

	case STATE_INACTIVE:
		if (!ev.pulse)
			break;
		if (!eq_margin(ev.duration, OCN_HEADER1_PULSE, OCN_UNIT / 2)) 
			break;
     
		data->count = 0;
		data->state = STATE_HEADER1_SPACE;
		return 0;

	case STATE_HEADER1_SPACE:
		if (ev.pulse)
			break;

		if (eq_margin(ev.duration, OCN_HEADER1_SPACE, OCN_UNIT / 2)) {
			data->state = STATE_USER_BIT_PULSE;
			return 0;
		} else if (eq_margin(ev.duration, OCN_REPEAT_SPACE, OCN_UNIT / 2)) {
			if (!dev->keypressed) {
				IR_dprintk(1, "Discarding last key repeat: event after key up\n");
			} else {
				data->state = STATE_REPEAT_END_PULSE;
			}
			return 0;
		}

		break;

	case STATE_USER_BIT_PULSE:
		if (!ev.pulse)
			break;

		if (!eq_margin(ev.duration, OCN_BIT_PULSE, OCN_UNIT / 2))
			break;

		data->state = STATE_USER_BIT_SPACE;
		return 0;

	case STATE_USER_BIT_SPACE:
             
		if (ev.pulse)
			break;

		data->bits <<= 1;
		if (eq_margin(ev.duration, OCN_BIT_1_SPACE, OCN_UNIT / 2))
			data->bits |= 1;
		else if (!eq_margin(ev.duration, OCN_BIT_0_SPACE, OCN_UNIT / 2))
			break;

               data->count++;
		if (data->count == OCN_USER_NBITS)
			data->state = STATE_HEADER2_PULSE;
		else
			data->state = STATE_USER_BIT_PULSE;

		return 0;
        case STATE_HEADER2_PULSE:
            if (!ev.pulse)
                break;
		
            if (eq_margin(ev.duration, OCN_HEADER2_PULSE, OCN_UNIT / 2)) {
                data->state = STATE_HEADER2_SPACE;
                return 0;
                }
            
            break;        
        case STATE_HEADER2_SPACE:
            if (ev.pulse)
                break;
            if (eq_margin(ev.duration, OCN_HEADER2_SPACE, OCN_UNIT / 2)) {
                data->state = STATE_KEY_BIT_PULSE;
                return 0;
                } 
            
            break;
 	case STATE_KEY_BIT_PULSE:
		if (!ev.pulse)
			break;

		if (!eq_margin(ev.duration, OCN_BIT_PULSE, OCN_UNIT / 2))
			break;

		data->state = STATE_KEY_BIT_SPACE;
		return 0;

	case STATE_KEY_BIT_SPACE:
		if (ev.pulse)
			break;

		data->bits <<= 1;
		if (eq_margin(ev.duration, OCN_BIT_1_SPACE, OCN_UNIT / 2))
			data->bits |= 1;
		else if (!eq_margin(ev.duration, OCN_BIT_0_SPACE, OCN_UNIT / 2))
			break;
		data->count++;

		if (data->count == (OCN_USER_NBITS+OCN_KEY_NBITS))
			goto decode_finish;
		else
			data->state = STATE_KEY_BIT_PULSE;

		return 0;
        case STATE_REPEAT_END_PULSE:
            if (!ev.pulse)
                break;

            if (eq_margin(ev.duration, OCN_REPEAT_END_PULSE, OCN_UNIT / 2))
            {
                rc_repeat(dev);
                IR_dprintk(1, "Repeat last key\n");
                data->state = STATE_INACTIVE;
                return 0;    
            }       
            break;
	}
      
	IR_dprintk(1, "OCN decode failed at count %d state %d (%uus %s)\n",
		   data->count, data->state, TO_US(ev.duration), TO_STR(ev.pulse));
	data->state = STATE_INACTIVE;
	return -EINVAL;

decode_finish:
    
    usercode0     = bitrev8((data->bits >> 32) & 0xff);
    usercode1     = bitrev8((data->bits >> 24) & 0xff);
    usercode2     = bitrev8((data->bits >> 16) & 0xff);

    if ((usercode0 !=0x4F)||(usercode1 !=0x43)||(usercode2 !=0x4E)) 
    {
        IR_dprintk(1, "OCN user code error: received 0x%02x,0x%02x,0x%02x\n",
            usercode0,usercode1,usercode2);
    }
    else
    {   
        scancode  = (bitrev8(data->bits >> 8 )<<8) | bitrev8((data->bits)) ;
        scancode = scancode | 0x10000;//Add to map RC_MAP_ELITE_TV
        IR_dprintk(1, "OCN scancode 0x%04x\n", scancode);
		printk("OCN keycode %d\n", rc_g_keycode_from_table(dev, scancode));
        rc_keydown(dev, scancode, 0);
        data->state = STATE_INACTIVE;
        return 0;
    }
   
    data->state = STATE_INACTIVE;
    return -EINVAL;  
}

static struct ir_raw_handler ocn_handler = {
	.protocols	  = RC_TYPE_OCN,
	.decode      = ir_ocn_decode,
};

static int __init ir_ocn_decode_init(void)
{
	ir_raw_handler_register(&ocn_handler);

	printk(KERN_INFO "IR OCN protocol handler initialized\n");
	return 0;
}

static void __exit ir_ocn_decode_exit(void)
{
	ir_raw_handler_unregister(&ocn_handler);
}

module_init(ir_ocn_decode_init);
module_exit(ir_ocn_decode_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("S3 Graphics Inc.");
MODULE_DESCRIPTION("OCN IR protocol decoder");

