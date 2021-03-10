% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [time_rdc, rdc1expMx] = calcCorrExpFromMxOctave(rangeexpfile,time_rdc)

[RangeBins, NVrx, N] = size(time_rdc);


% base exponents, 8 range exponents and reserved
rangeExponentsMxAll = zeros(RangeBins,10);
disp('Reading RDC1 Exponents file');
f = fopen(rangeexpfile, 'r');

for rr = 1:RangeBins
    % octave support: clunkier bit parsing from uint32s
    Rexp = fread(f, 4, '*uint32');
    rangeExponentsMxAll(rr,1) = bin2dec(dec2bin(bitget(Rexp(1),4:-1:1))');
    rangeExponentsMxAll(rr,2) = bin2dec(dec2bin(bitget(Rexp(1),17:-1:5))');
    rangeExponentsMxAll(rr,3) = bin2dec(dec2bin(bitget(Rexp(1),30:-1:18))');
    rangeExponentsMxAll(rr,4) = bin2dec(dec2bin([ bitget(Rexp(2),11:-1:1) bitget(Rexp(1),32:-1:31)])');
    rangeExponentsMxAll(rr,5) = bin2dec(dec2bin(bitget(Rexp(2),24:-1:12))');
    rangeExponentsMxAll(rr,6) = bin2dec(dec2bin([ bitget(Rexp(3),5:-1:1) bitget(Rexp(2),32:-1:25)])');
    rangeExponentsMxAll(rr,7) = bin2dec(dec2bin(bitget(Rexp(3),18:-1:6))');
    rangeExponentsMxAll(rr,8) = bin2dec(dec2bin(bitget(Rexp(3),31:-1:19))');
    rangeExponentsMxAll(rr,9) = bin2dec(dec2bin([ bitget(Rexp(4),12:-1:1) bitget(Rexp(3),32)])');
    rangeExponentsMxAll(rr,10) = bin2dec(dec2bin(bitget(Rexp(4),32:-1:13))');
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
    
    rI = squeeze(int32(real(time_rdc(rr, :, :))));
    rQ = squeeze(int32(imag(time_rdc(rr, :, :))));
    
    if NVrx == 1  || N==1;
        rI = transpose(rI);
        rQ = transpose(rQ);
    end
    
    time_rdc(rr, :, :) = complex(rI .* expo, rQ .* expo);
end
clear rangeExponentsMx
