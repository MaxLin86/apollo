% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
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
