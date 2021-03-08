% START_SOFTWARE_LICENSE_NOTICE
% -------------------------------------------------------------------------------------------------------------------
% Copyright (C) 2016-2017 Uhnder, Inc. All rights reserved.
% This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
% under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
% develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
% strictly prohibited.
% Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
% Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
% THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
% BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
% REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
% TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
% PROGRAM IS STRICTLY PROHIBITED.
% THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
% WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
% THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
% IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
% PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
% SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
% WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
% -------------------------------------------------------------------------------------------------------------------
% END_SOFTWARE_LICENSE_NOTICE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2016 Uhnder Inc
% Author: 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: complexDouble2int32()
%       
% Option:
%
% Input:     complex double input
%        
% Output:    int16 and exponent
%
% Calls: 
%       
%
% Description: This function scales up and outputs int16 data and an exponent 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [int32Out, shift] = complexDouble2int32(cpxIn,bits)
% bits of precision
if ~exist('bits','var')
    bits = 32;
end
% tempI=real(cpxIn);
% tempQ=imag(cpxIn);
% 
% mval = 2^(16 - 2) - 1;
% mI = max(abs(tempI(:)));
% mQ = max(abs(tempQ(:)));
% m = max(mI, mQ);
% shift = 0;
% if m==0
%     shift = 0;
% else
%     while (m * 2^shift) < mval;
%         shift = shift + 1;
%     end
% end
% outI = tempI .* 2^shift;
% outQ = tempQ .* 2^shift;
% int16Out = complex(int16(outI), int16(outQ));
% % output shift needed to get to original values
% shift = -shift;
tempI=real(cpxIn);
tempQ=imag(cpxIn);
scaleTo=0.5;

m = max( max(abs(tempI(:))),  max(abs(tempQ(:))));
shift = 0;
int32OutI = tempI;
int32OutQ = tempQ;

if m==0
    shift = 0;
elseif m>scaleTo
    while (m * 2^shift) > scaleTo-(2^-(bits-1));
        shift = shift - 1;
        int32OutI = int32OutI/2;
        int32OutQ = int32OutQ/2;
    end
    
else
    while (m * 2^shift) < scaleTo-(2^-(bits-1));
        shift = shift + 1;
        int32OutI = int32OutI*2;
        int32OutQ = int32OutQ*2;
    end
    shift = shift - 1;
    int32OutI = int32OutI/2;
    int32OutQ = int32OutQ/2;
end
int32Out=complex(int32(round(int32OutI*2^(bits-1)/scaleTo)),int32(round(int32OutQ*2^(bits-1)/scaleTo)));

