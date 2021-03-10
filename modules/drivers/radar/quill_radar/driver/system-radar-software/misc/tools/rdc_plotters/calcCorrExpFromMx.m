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
function [time_rdc,rdc1expMx] = calcCorrExpFromMx(rangeexpfile,time_rdc)

[RangeBins, NVrx, N] = size(time_rdc);


% base exponents, 8 range exponents and reserved
rangeExponentsMxAll = zeros(RangeBins,10);
disp('Reading RDC1 Exponents file');
f = fopen(rangeexpfile, 'r');

for rr = 1:RangeBins
    % load range exponents matrix
    %baseRangeExponent
    rangeExponentsMxAll(rr,1) = fread(f, 1, 'ubit4');
    % eight rangeExponentsMx
    rangeExponentsMxAll(rr,2:9) = fread(f, 8, 'ubit13');
    % reserved field
    rangeExponentsMxAll(rr,10) = fread(f, 1, 'ubit20');
end
fclose(f);

baseRangeExponent = rangeExponentsMxAll(:,1);
rangeExponentsMx = rangeExponentsMxAll(:,2:9);
rangeExpReserved = rangeExponentsMxAll(:,10);
clear rangeExponentsMxAll
rdc1expMx = zeros(RangeBins,N);
for rr = 1:RangeBins
    currBase = single(baseRangeExponent(rr));
    currRangeExponents = ones(1,N)*currBase;
    validRexpCol=sum(rangeExponentsMx(rr,:)>0); % zeros are unused exponents

    if validRexpCol > 0
        for itidx = 1:validRexpCol
            currRangeExponents(rangeExponentsMx(rr,itidx)+1:end) = ...
            currRangeExponents(rangeExponentsMx(rr,itidx)+1:end) + 1;
        end
    end
        
    expo = 2 .^ currRangeExponents;
    rdc1expMx(rr,:) = expo; 
    expo = int32(repmat(expo, [NVrx, 1]));
    
    rI = real(squeeze(int32(time_rdc(rr, :, :))));
    rQ = imag(squeeze(int32(time_rdc(rr, :, :))));
    
    if NVrx == 1  || N==1;
        rI = transpose(rI);
        rQ = transpose(rQ);
    end
    
    time_rdc(rr, :, :) = complex(rI .* expo, rQ .* expo);
end
clear rangeExponentsMx
