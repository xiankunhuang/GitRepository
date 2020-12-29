#ifndef _IAP_H_
#define _IAP_H_

/* IAP function enter address */
#define IAP_ENTER_ADR   0x1FFF1FF1

/* 
 *  Definition of IAP function CMD code 
 */                                     
#define IAP_Prepare             50                   
#define IAP_RAMTOFLASH          51                                                                      
#define IAP_ERASESECTOR         52 
#define IAP_BLANKCHK            53
#define IAP_READPARTID          54
#define IAP_BOOTCODEID          55
#define IAP_COMPARE             56
#define IAP_GET_UID             58

/*
 *  Definition of IAP function RETURN code 
 */
#define CMD_SUCCESS                                0
#define INVALID_COMMAND                            1
#define SRC_ADDR_ERROR                             2 
#define DST_ADDR_ERROR                             3
#define SRC_ADDR_NOT_MAPPED                        4
#define DST_ADDR_NOT_MAPPED                        5
#define COUNT_ERROR                                6
#define INVALID_SECTOR                             7
#define SECTOR_NOT_BLANK                           8
#define SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION    9
#define COMPARE_ERROR                              10
#define BUSY                                       11


/* 
 *  very important parameter, pay close attention
 */
// TODO: check this when debugging 
#define IAP_FCCLK            (_F_CCLK/1000)

/* these functions are currently used */
unsigned int  sectorPrepare (unsigned char ucSec1, unsigned char ucSec2);
unsigned int  ramCopy (unsigned int ulDst, unsigned int ulSrc, unsigned int ulNo);
unsigned int  sectorErase (unsigned char ucSec1, unsigned char ucSec2);
unsigned int  blankChk (unsigned char ucSec1, unsigned char ucSec2);
unsigned int  dataCompare (unsigned int ulDst, unsigned int ulSrc, unsigned int ulNo);

/* these functions are currently not used */
unsigned int  parIdRead (void);
unsigned int  codeIdBoot (void);
unsigned int  reinvoke_isp ();

#endif // _IAP_H_

