LE-Board-Firmware
=================

Install (For Ubuntu 14)
-------

* Go to http://www.broadcom.com/products/wiced/smart/, register and download WICED-Smart-SDK-1.1.0(.7z Archive)

* Extract the SDK somewhere in your workspace

* Enter WICED-Smart-SDK-1.1.0/Apps/RAM and clone this repository

    cd WICED-Smart-SDK-1.1.0/Apps/RAM
    
    git clone git@github.com:NilsDarphin/LEBoard_firmware.git
    
* In the file WICED-Smart-SDK-1.1.0/wiced_toolchain_common.mk add the following lines 
    
*Line 71 (Linux 32 bits)*
    
    CGS_FULL_NAME         := "$(CGS_PATH)Linux64/cgs"
    CHIPLOAD_FULL_NAME    := "$(CHIPLOAD_PATH)Linux64/ChipLoad"
    DETECTANDID_FULL_NAME := "$(DETECTANDID_PATH)Linux64/detandid"
    HEX_TO_BIN_FULL_NAME  := "$(HEX_TO_BIN_PATH)Linux64/ihex2bin"
        
*Line 104 (Linux 64 bits)*
    
    CGS_FULL_NAME         := "$(CGS_PATH)Linux32/cgs"
    CHIPLOAD_FULL_NAME    := "$(CHIPLOAD_PATH)Linux32/ChipLoad"
    DETECTANDID_FULL_NAME := "$(DETECTANDID_PATH)Linux32/detandid"
    HEX_TO_BIN_FULL_NAME  := "$(HEX_TO_BIN_PATH)Linux32/ihex2bin"

* Add execute permissions to tools included in the SDK

    sudo chmod u+x Tools/CGS/Linux64/cgs
    
    sudo chmod u+x Tools/ChipLoad/Linux64/ChipLoad
    
    sudo chmod u+x Tools/DetectAndId/Linux64/detandid
    
    sudo chmod u+x Tools/IntelHexToBin/Linux64/ihex2bin
    
    
    sudo chmod u+x Tools/CGS/Linux32/cgs
    
    sudo chmod u+x Tools/ChipLoad/Linux32/ChipLoad
    
    sudo chmod u+x Tools/DetectAndId/Linux32/detandid
    
    sudo chmod u+x Tools/IntelHexToBin/Linux32/ihex2bin
    
* Make the SDK use your own version of perl (fix problems related to perl version)

    mv Tools/common/Linux64/perl Tools/common/Linux64/perl.old
    ln -s /usr/bin/perl Tools/common/Linux64/perl
    
    mv Tools/common/Linux32/perl Tools/common/Linux32/perl.old
    ln -s /usr/bin/perl Tools/common/Linux32/perl
    
* Create a new configuration folder for the project by using an existing one

    cp -R Wiced-Smart/tier2/brcm/automation_io  Wiced-Smart/tier2/brcm/LEBoard_firmware
    mv Wiced-Smart/tier2/brcm/LEBoard_firmware/bld/automation_io.cgs Wiced-Smart/tier2/brcm/LEBoard_firmware/bld/LEBoard_firmware.cgs
    