#include "config.h"

typedef void (*IAP)(unsigned int[],unsigned int[]);
IAP IAP_Entry =(IAP) IAP_ENTER_ADR; /* 初始化函数指针IAP_Entry      */

unsigned int  GulParamin[8];
unsigned int  GulParamout[8];

unsigned int sectorPrepare (unsigned char ucSec1, unsigned char ucSec2)
{  
    GulParamin[0] = IAP_Prepare;
    GulParamin[1] = ucSec1; 
    GulParamin[2] = ucSec2;                            
    (*IAP_Entry)(GulParamin, GulParamout);

    return (GulParamout[0]);
}

unsigned int ramCopy (unsigned int ulDst, unsigned int ulSrc, unsigned int ulNo)
{  
    GulParamin[0] = IAP_RAMTOFLASH;
    GulParamin[1] = ulDst; 
    GulParamin[2] = ulSrc;
    GulParamin[3] = ulNo;
    GulParamin[4] = IAP_FCCLK;
    (*IAP_Entry)(GulParamin, GulParamout); 
    
    return (GulParamout[0]);  
}

unsigned int sectorErase (unsigned char ucSec1, unsigned char ucSec2)
{  
    GulParamin[0] = IAP_ERASESECTOR;
    GulParamin[1] = ucSec1;
    GulParamin[2] = ucSec2;
    GulParamin[3] = IAP_FCCLK;
	
    (*IAP_Entry)(GulParamin, GulParamout); 
	
    return (GulParamout[0]); 
}

unsigned int blankChk (unsigned char ucSec1, unsigned char ucSec2)
{  
    GulParamin[0] = IAP_BLANKCHK;
    GulParamin[1] = ucSec1;
    GulParamin[2] = ucSec2;
    (*IAP_Entry)(GulParamin, GulParamout);

    return (GulParamout[0]);
}

unsigned int parIdRead (void)
{  
    GulParamin[0] = IAP_READPARTID;
    (*IAP_Entry)(GulParamin, GulParamout);

    return (GulParamout[0]);
}

unsigned int codeIdBoot (void)
{  
    GulParamin[0] = IAP_BOOTCODEID;
    (*IAP_Entry)(GulParamin, GulParamout);

    return (GulParamout[0]);
}

unsigned int dataCompare (unsigned int ulDst, unsigned int ulSrc, unsigned int ulNo)
{  
    GulParamin[0] = IAP_COMPARE;
    GulParamin[1] = ulDst;
    GulParamin[2] = ulSrc;
    GulParamin[3] = ulNo;
    (*IAP_Entry)(GulParamin, GulParamout);

    return (GulParamout[0]); 
}

unsigned int reinvoke_isp ()
{  
    GulParamin[0] = 57;      
    (*IAP_Entry)(GulParamin, GulParamout);                             

    return (GulParamout[0]);                                         
}

