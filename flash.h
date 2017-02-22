//****************************************************************************
// FILE NAME: flash.h                                
//                                           
// PURPOSE: Header File for S12 FLASH (32K) 
//                               
//****************************************************************************

#define FDIV8	0x3F	/*bit masks	*/
#define PRDIV8	0x40
#define FDIVLD	0x80

#define BLANK	0x04	/*bit masks	*/
#define ACCERR	0x10
#define PVIOL 	0x20
#define CCIF	0x40
#define CBEIF	0x80

#define PROG	0x20
#define ERASE	0x40
#define ERASE_VERIFY	0x05 
#define BYTE_PROGRAM	0x20
#define SECTOR_ERASE	0x40
#define MASS_ERASE 		0x41
