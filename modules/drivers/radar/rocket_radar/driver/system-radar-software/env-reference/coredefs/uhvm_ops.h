// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHVM_OPS_H
#define SRS_HDR_UHVM_OPS_H 1


enum Opcode
{
//  Opcode         Name                                             : Format
//  =============  ================================================ : =======================
    OP_NOP,        // NoOPeration                                   :
    OP_EXIT,       // EXIT immediately                              :
    OP_SLEEP,      // SLEEP microseconds                            : immed24
    OP_REGS,       // print REGisterS                               :
    OP_UHP,        // UHPrintf                                      : ???

    OP_LDI,        // LoaD Immediate                                : reg1i
    OP_LDIX,       // LoaD Immediate eXtended                       : reg1i + u32

    OP_LD,         // LoaD from VM memory                           : reg1i + addr32
    OP_LDR,        // LoaD from real memory address or HW register  : reg1i + addr32
    OP_LDF,        // LoaD Field from memory address                : reg1i + addr32 + mask32

    OP_ST,         // STore to VM memory                            : reg1i + addr32
    OP_STR,        // STore to real memory address or HW register   : reg1i + addr32
    OP_STRI,       // STore Immediate to real memory or HW register : u32   + addr32
    OP_STF,        // STore Field to memory address                 : reg1i + addr32 + mask32
    OP_STFI,       // STore Immedite Field to memory address        : u32   + addr32 + mask32

    OP_CALL,       // CALL subroutine                               : immed24
    OP_RET,        // RETurn from subroutine or from main           :

    OP_BR,         // BRanch always                                 : immed24
    OP_BEQ,        // BRanch if EQual                 (       Z=1)  : immed24
    OP_BNE,        // BRanch if Not Equal             (       Z=0)  : immed24
    OP_BGT,        // BRanch if Greater Than          (N=0 && Z=0)  : immed24
    OP_BLT,        // BRanch if Less Than             (N=1 && Z=0)  : immed24
    OP_BGE,        // BRanch if Greater Than or Equal (N=0 || Z=1)  : immed24
    OP_BLE,        // BRanch if Less Than or Equal    (N=1 || Z=1)  : immed24

    OP_TST,        // TeST one register and set flags               : reg1i
    OP_CMP,        // CoMPare two registers                         : reg2i
    OP_CMPI,       // CoMPare Immediate to register                 : reg1i
    OP_CMPIX,      // CoMPare Immediate eXtended to register        : reg1i + u32
    OP_ADD,        // ADD two registers                             : reg3
    OP_ADDI,       // ADD Immediate to register                     : reg2i
    OP_ADDIX,      // ADD Immediate eXtended to register            : reg2i + u32
    OP_SUB,        // SUB two registers                             : reg3
    OP_SUBI,       // SUB Immediate from register                   : reg2i
    OP_SUBIX,      // SUB Immediate eXtended from register          : reg2i + u32
    OP_MUL,        // MULtiply two registers                        : reg3
    OP_DIV,        // DIVde two registers                           : reg3

};

struct Instr_immed24
{
    uint32_t                opcode      :  8;
    uint32_t                immed       : 24;
};

struct Instr_immed8x3
{
    uint32_t                opcode      :  8;
    uint32_t                immediate1  :  8;
    uint32_t                immediate2  :  8;
    uint32_t                immediate3  :  8;
};

struct Instr_reg3i
{
    uint32_t                opcode      :  8;
    uint32_t                reg_dst     :  5;
    uint32_t                reg_src1    :  5;
    uint32_t                reg_src2    :  5;
    uint32_t                misc        :  1;
    uint32_t                immed       :  8;
};

struct Instr_reg2i
{
    uint32_t                opcode      :  8;
    uint32_t                reg_dst     :  5;
    uint32_t                reg_src     :  5;
    uint32_t                immed       : 14;
};

struct Instr_reg1i
{
    uint32_t                opcode      :  8;
    uint32_t                reg         :  5;
    uint32_t                misc        :  3;
    uint32_t                immed       : 16;
};

struct Instruction
{
    union
    {
        uint32_t                    raw;
        struct Instr_immed24        immed24;
        struct Instr_immed8x3       immed8x3;
        struct Instr_reg3i          reg3i;
        struct Instr_reg2i          reg2i;
        struct Instr_reg1i          reg1i;
    };
};

#endif // SRS_HDR_UHVM_OPS_H
