README.txt
KS7010 Linux driver

Copyright (C) 2009 Renesas Technology Corp.

== Version ==

   KS7010 SDIO device driver for Linux 2.6.24   version 007 (for evaluate, beta version)
   KS7010 MAC Firmware                          version 0.2F 

== Features ==
   IEEE801.11b/g
   Infrastructure mode and ad-hoc mode 
   WEP40/WEP104/WPA/WPA2  
   Linux Wireless extension 29

== Throughput ==
   TCP Tx       15.3Mbps
   TCP Rx       19.2Mbps
   UDP Tx       12.6Mbps
   UDP Rx       14.9Mbps

 Equipment
   WLAN KS7930EVB(KS7010) SDIO I/F
   PC           TOSHIBA Qosmio E10 375LS
   AP           Corega CG-WLBARG2
   Tool         chariot

== Supported Hardware ==
   Lenovo IBM ThinkPad x60 (SDIO controller: Ricoh)
   Lenovo IBM ThinkPad x40 (SDIO controller: Ricoh)
   TOSHIBA dynabook SS M10 (SDIO controller: Texas Instruments)
   TOSHIBA Qosmio E10 375LS (SDIO controller: Texas Instruments)
   Panasonic LetÅfs note CF-T4 (SDIO controller Ricoh)
 

== Required Software ==
   Linux Kernel 2.6.24
     Required ksc_mmc.patch

   Wireless tools ver29
     http://www.hpl.hp.com/personal/Jean_Tourrilhes/Linux/Tools.html

   wpa_supplicant ver0.5.8
     http://hostap.epitest.fi/wpa_supplicant/

   Gnu c compiler

== Files ==
GPL/
  Kernel/
    ksc_mmc.patch       patch file for mmc driver.
    Sample.config       kernel.config
  Src/
    Makefile
    Makefile-obj
    eap_packet.h        header file for WPA EAP packet.
    ks_debug.c                  debug function
    ks_debug.h
    ks_hostif.c         Driver ? MAC Firmware interface.
    ks_hostif.h
    ks_wlan.h                   header file for KS7010
    ks_wan_ioctl.h              header file for wireless private ioctl.
    ks_wlan_net.c       netdev API, wireless extension API.
    michael_mic.c       calc TKIP MIC.    
    michael_mic.h
    sdio/
      Makefile
      ks7010_sdio.c             SDIO interface driver.
      ks7010_sdio.h
      ks79xx.cfg                config file.
Firmware/
      ks7010sd.rom              ks7010 firmware 0.2b
 

== Install driver ==
 step1: Compile kernel
   extract kernel-2.6.24.tar.bz2
        #tar xjf linux-2.6.24.tar.bz2
   Apply ksc_mmc.path to kernel source code.
        #cd linux-2.6.24
        #patch ?p1 < ksc_mmc.patch
   Kernel configuration
        # make menuconfig
        Select ÅgDevice DriversÅh
        Select ÅgMMC/SD card supportÅh to ÅgMÅh
 
 

        Select Secure Digital Host Controller Interface support (EXPERIMENTAL)
 
        Select Networking
        Select Wireless
        Select Wireless extensions


   Check kernel Configuration
        #more .config

        CONFIG_MMC=m
        # CONFIG_MMC_DEBUG is not set
        # CONFIG_MMC_UNSAFE_RESUME is not set

        #
        # MMC/SD Card Drivers
        #
        CONFIG_MMC_BLOCK=m
        CONFIG_MMC_BLOCK_BOUNCE=y
        CONFIG_SDIO_UART=m

        #
        # MMC/SD Host Controller Drivers
        #
        CONFIG_MMC_SDHCI=m
        CONFIG_MMC_RICOH_MMC=m
        CONFIG_MMC_WBSD=m
        CONFIG_MMC_TIFM_SD=m
        CONFIG_NEW_LEDS=y
        CONFIG_LEDS_CLASS=m
        Compile kernel
   On Debian
       #make-kpkg --revision=01  --append_to_version -sdio --config=menuconfig ?initrd

   The other distribution
       #make 

   Install kernel
        On Debian
        #dpkg ?i linux-image-2.6.24-sdio_01_i386.deb

        The other distribution
        #make install
        #make modules_install

   Reboot PC, and Select New Kernel.

 step2:  Install KS7010 WLAN Driver
  Compile driver source code
     - check Makefile.
         set kernel source dir to KDIR
         example  KDIR=/usr/src/linux-2.6.24
       # make
     - driver file is Ågks79xx_sdio.koÅh. Please see Install section in this file.

   Copy Config file and ROM file.
        # cp frimware/ks7010sd.rom  /lib/firmware
        # cp GPL/Src/sdio/ks79xx.cfg /lib/firmware

        # cd GPL/Src
        # make
        # insmod ./ks79xx_sdio.ko

 step3:  Insert wlan module
      - Insert ks7010 SDIO module to SDIO slot of PC
      - Check dmesg
        # dmesg |grep ks

        ks_wlan: MAC ADDRESS = 12:13:ff:ff:ff:ff
       ks_wlan: firmware ver. = 7010-SDIO-0.25 [Mar 03 2008  20:07:09]
 
 step4: setting WLAN configuration
        # ifconfig eth1 192.168.10.5
        # iwconfig eth1 essid VIAS3-SWOpenGLSoC-Linksys

 step5: setting WPA configuration  ( hostap wpa_supplicant ver 0.5.8)
        # wpa_supplicant ?Dwext -ieth0 -d  -c wpa_tkip.conf


command

iwlist (eth1) scan
dhclient (eth1)
ping xx.xx.xx


modify  linux kernel 
sdio_cis.c    cistpl_funce_func
L178: size =28 < SDIO_SDIO_REV_1_00 = 42

