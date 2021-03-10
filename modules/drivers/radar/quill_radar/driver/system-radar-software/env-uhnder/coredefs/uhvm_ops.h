#ifndef SRS_HDR_UHVM_OPS_H
#define SRS_HDR_UHVM_OPS_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE


enum Opcode
{
//  Opcode         Name                                             : Format
//  =============  ================================================ : =======================
    OP_NOP,        // NoOPeration                                   : ---

    OP_REGS,       // print REGisterS                               : ---
    OP_UHP,        // UHPrintf                                      : ???

    OP_LDI,        // LoaD Immediate                                : reg1i
    OP_LDIX,       // LoaD Immediate eXtended                       : reg1i + u32
    OP_LD,         // LoaD from memory address                      : reg1i + addr32
    OP_LDF,        // LoaD Field from memory address                : reg1i + addr32 + mask32

    OP_ST,         // STore to memory address                       : reg1i + addr32
    OP_STF,        // STore Field to memory address                 : reg1i + addr32 + mask32

    OP_CALL,       // CALL subroutine                               : immed24
    OP_RET,        // RETurn from subroutine                        : ---

    OP_BR,         // BRanch always                                 : immed24
    OP_BEQ,        // BRanch if EQual                    (    Z=1)  : immed24
    OP_BNE,        // BRanch if Not Equal                (    Z=0)  : immed24
    OP_BGT,        // BRanch if Greater Than             (N=0,Z=0)  : immed24
    OP_BLT,        // BRanch if Less Than                (N=1,Z=0)  : immed24
    OP_BGE,        // BRanch if Greater Than or Equal    (N=0,Z=1)  : immed24
    OP_BLE,        // BRanch if Less Than or Equal       (N=1,Z=1)  : immed24

    OP_CMP,        // CoMPare two registers                         : reg2i
    OP_CMPI,       // CoMPare Immediate to register                 : reg1i
    OP_CMPIX,      // CoMPare Immediate eXtended to register        : reg1i + u32
    OP_ADD,        // ADD two registers                             : reg3
    OP_ADDI,       // ADD Immediate to register                     : reg2i
    OP_ADDIX,      // ADD Immediate eXtended to register            : reg2i + u32
    OP_SUB,        // SUB two registers                             : reg3
    OP_SUBI,       // SUB Immediate from register                   : reg2i
    OP_SUBIX,      // SUB Immediate eXtended from register          : reg2i + u32

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
