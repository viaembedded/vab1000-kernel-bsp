/* Elite IR Controler keymap 
 *
 * Copyright (C) 2013 S3 Graphics 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>
#include <linux/module.h>

static struct rc_map_table rc_elite_tv[] = {
	/* Type 1: OCN */
	{ 0x10FF0, 538,},//KEY_POWER        
	{ 0x10CF3, 542,},//KEY_MUTE
	{ 0x10DF2, 518,},//KEY_VOLUMEUP           
	{ 0x12DD2, 514,},//KEY_VOLUMEDOWN,
	{ 0x12FD0, 530,},//KEY_CHANNELDOWN
	{ 0x12ED1, 534,},//KEY_CHANNELUP
	{ 0x10EF1, 522,},//KEY_EXIT
	{ 0x10BF4, 526,},//KEY_BACK
	{ 0x11CE3, 576,},//KEY_PAGEUP     
	{ 0x126D9, 584,},//KEY_PAGEDOWN
	{ 0x130CF, 539,}, //KEY_PVR         
	{ 0x12AD5, 535,},//KEY_PROGRAM
	{ 0x12BD4, 596,},//KEY_UP
	{ 0x129D6, 580,}, //KEY_LEFT
	{ 0x128D7, 604,}, //KEY_RIGHT
	{ 0x125DA, 592,}, //KEY_DOWN   
	{ 0x12CD3, 588,},//KEY_OK
	{ 0x11BE4, 523,},//KEY_INFO         
	{ 0x1423E, 581,},//KEY_REWIND  
	{ 0x15A58, 589,},//KEY_PLAY     
	{ 0x15C60, 597,},//KEY_STOP         
	{ 0x1324F, 605,},//KEY_FASTFORWARD        
	{ 0x118E7, 577,},//KEY_REFRESH    
	{ 0x12860, 515,},//KEY_CONNECT   
	{ 0x124DB, 543,},//KEY_FAVORITES
	{ 0x18612, 520,},//KEY_RECORD
	{ 0x1351A, 519,},//KEY_CONFIG,   
	{ 0x13378, 512,},//KEY_FIND
	{ 0x16512, 600,},//KEY_HELP
	{ 0x1492A, 517,},//KEY_SOUND
	{ 0x116E9, 527,},//KEY_TV
	{ 0x117E8, 531,},//KEY_AUDIO        
	{ 0x119E6, 545,}, /*KEY_SD*/
	{ 0x11AE5, 546,}, /*KEY_HD*/
    { 0x121DE, 585,},//KEY_1
    { 0x123DC, 593,},//KEY_2   
    { 0x122DD, 601,},//KEY_3
    { 0x11DE2, 525,},//KEY_4 
    { 0x11EE1, 533,},//KEY_5
    { 0x11FE0, 541,},//KEY_6
    { 0x120DF, 521,},//KEY_7
    { 0x111EE, 529,},//KEY_8
    { 0x112ED, 537,},//KEY_9
    { 0x114EB, 524,},//"*"
    { 0x113EC, 532,},//KEY_0
    { 0x115EA, 540,}, //"#"  
    
    /* Type 2: RC-5 */
	{ 0x21650, 538,},//KEY_POWER           
	{ 0x21651, 542,},//KEY_MUTE
	{ 0x21657, 518,},//KEY_VOLUMEUP           
	{ 0x21656, 514,},//KEY_VOLUMEDOWN,
	{ 0x21659, 530,},//KEY_CHANNELDOWN
	{ 0x2165A, 534,},//KEY_CHANNELUP
	{ 0x21654, 522,},//KEY_EXIT
	{ 0x21655, 526,},//KEY_BACK
	{ 0x21668, 576,},//KEY_PAGEUP    
	{ 0x21669, 584,},//KEY_PAGEDOWN
	{ 0x21662, 539,}, //KEY_PVR        
	{ 0x21644, 535,},//KEY_PROGRAM
	{ 0x21663, 596,},//KEY_UP
	{ 0x21665, 580,}, //KEY_LEFT   
	{ 0x21666, 604,}, //KEY_RIGHT
	{ 0x21664, 592,}, //KEY_DOWN        
	{ 0x21667, 588,},//KEY_OK
	{ 0x21660, 523,},//KEY_INFO          
	{ 0x2166C, 581,},//KEY_REWIND      
	{ 0x2166D, 589,},//KEY_PLAY      
	{ 0x2166E, 597,},//KEY_STOP           
	{ 0x2166F, 605,},//KEY_FASTFORWARD            
	{ 0x2166B, 577,},//KEY_REFRESH          
	{ 0x21661, 515,},//KEY_CONNECT     
	{ 0x21642, 543,},//KEY_FAVORITES
	{ 0x21643, 520,},//KEY_RECORD
	{ 0x21658, 519,},//KEY_CONFIG, 
	{ 0x21652, 512,},//KEY_FIND
	{ 0x2166A, 600,},//KEY_HELP
	{ 0x2165C, 517,},//KEY_SOUND
	{ 0x21645, 527,},//KEY_TV
	{ 0x21646, 531,},//KEY_AUDIO        
	{ 0x21647, 545,}, /*KEY_SD*/
	{ 0x21648, 546,}, /*KEY_HD*/
	{ 0x21671, 585,},//KEY_1
	{ 0x21672, 593,},//KEY_2   
	{ 0x21673, 601,},//KEY_3
	{ 0x21674, 525,},//KEY_4 
	{ 0x21675, 533,},//KEY_5
	{ 0x21676, 541,},//KEY_6
	{ 0x21677, 521,},//KEY_7
	{ 0x21678, 529,},//KEY_8
	{ 0x21679, 537,},//KEY_9
	{ 0x21649, 524,},//"*"
 	{ 0x21670, 532,},//KEY_0
 	{ 0x2164A, 540,}, //"#"   

	/* Type 3: NEC */
	{ 0x7F1A, 538,},//KEY_POWER       
	{ 0x7F1E, 542,},//KEY_MUTE
	{ 0x7F06, 518,},//KEY_VOLUMEUP          
	{ 0x7F02, 514,},//KEY_VOLUMEDOWN,
	{ 0x7F12, 530,},//KEY_CHANNELDOWN
	{ 0x7F16, 534,},//KEY_CHANNELUP
	{ 0x7F0A, 522,},//KEY_EXIT    
	{ 0x7F0E, 526,},//KEY_BACK
	{ 0x7F40, 576,},//KEY_PAGEUP      
	{ 0x7F48, 584,},//KEY_PAGEDOWN
	{ 0x7F1B, 539,}, //KEY_PVR          
	{ 0x7F17, 535,},//KEY_PROGRAM
	{ 0x7F54, 596,},//KEY_UP
	{ 0x7F44, 580,}, //KEY_LEFT 
	{ 0x7F5C, 604,}, //KEY_RIGHT
	{ 0x7F50, 592,}, //KEY_DOWN           
	{ 0x7F4C, 588,},//KEY_OK
	{ 0x7F0B, 523,},//KEY_INFO            
	{ 0x7F45, 581,},//KEY_REWIND        
	{ 0x7F4D, 589,},//KEY_PLAY         
	{ 0x7F55, 597,},//KEY_STOP               
	{ 0x7F5D, 605,},//KEY_FASTFORWARD          
	{ 0x7F41, 577,},//KEY_REFRESH         
	{ 0x7F03, 515,},//KEY_CONNECT  	
	{ 0x7F1F, 543,},//KEY_FAVORITES
	{ 0x7F08, 520,},//KEY_RECORD
	{ 0x7F07, 519,},//KEY_CONFIG,  
	{ 0x7F00, 512,},//KEY_FIND
	{ 0x7F58, 600,},//KEY_HELP
	{ 0x7F05, 517,},//KEY_SOUND
	{ 0x7F0F, 527,},//KEY_TV
	{ 0x7F13, 531,},//KEY_AUDIO        
	{ 0x7F21, 545,}, /*KEY_SD*/
	{ 0x7F22, 546,}, /*KEY_HD*/
	{ 0x7F49, 585,},//KEY_1
	{ 0x7F51, 593,},//KEY_2   
	{ 0x7F59, 601,},//KEY_3
	{ 0x7F0D, 525,},//KEY_4 
	{ 0x7F15, 533,},//KEY_5
	{ 0x7F1D, 541,},//KEY_6
	{ 0x7F09, 521,},//KEY_7
	{ 0x7F11, 529,},//KEY_8
	{ 0x7F19, 537,},//KEY_9
	{ 0x7F0C, 524,},//"*"
	{ 0x7F14, 532,},//KEY_0
	{ 0x7F1C, 540,}, //"#"  
        
};

static struct rc_map_list rc_elite_tv_map = {
	.map = {
		.scan    = rc_elite_tv,
		.size    = ARRAY_SIZE(rc_elite_tv),
		.rc_type = RC_TYPE_OCN,
		.name    = RC_MAP_ELITE_TV,
	}
};

static int __init init_rc_elite_tv_map(void)
{
	return rc_map_register(&rc_elite_tv_map);
}

static void __exit exit_rc_elite_tv_map(void)
{
	rc_map_unregister(&rc_elite_tv_map);
}

module_init(init_rc_elite_tv_map)
module_exit(exit_rc_elite_tv_map)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("S3 Graphics Inc.");

