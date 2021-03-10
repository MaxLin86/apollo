// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details



#ifndef CCP_MEM_H
#define CCP_MEM_H 1

//Used by HSM to sync start of CCP
//Usefull when B1 starts CCP as a separate image
#define CCP_START_FUSA (0x0001B000)
#define CCP_START_SYNC ((128*1024) - 4)
#define CCP_START_SYNC_MAGIC 0x43435053 //CCPS
#define HSM_CCP_START_SYNC (CCP_START_SYNC + 0x18000000)

#endif
