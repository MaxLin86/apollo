% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2016 Uhnder Inc
% Author: 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: complexDouble2int64()
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

function [int64Out, shift] = complexDouble2int64(cpxIn,bits)
% bits of precision
if ~exist('bits','var')
    bits = 64;
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
int64OutI = tempI;
int64OutQ = tempQ;

if m==0
    shift = 0;
elseif m>scaleTo
    while (m * 2^shift) > scaleTo-(2^-(bits-1));
        shift = shift - 1;
        int64OutI = int64OutI/2;
        int64OutQ = int64OutQ/2;
    end
    
else
    while (m * 2^shift) < scaleTo-(2^-(bits-1));
        shift = shift + 1;
        int64OutI = int64OutI*2;
        int64OutQ = int64OutQ*2;
    end
    shift = shift - 1;
    int64OutI = int64OutI/2;
    int64OutQ = int64OutQ/2;
end
int64Out=complex(int64(round(int64OutI*2^(bits-1)/scaleTo)),int64(round(int64OutQ*2^(bits-1)/scaleTo)));

