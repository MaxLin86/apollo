% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

function [strout, rngMax, rngMag, interpMagAz, interpBinAz, interpMagEl, interpBinEl] =  printInterpPeaksAzEl(rCutdata, dBpeak, peakCensorRngAzEl, rngMaxes, azbins, elbins)

[maxVal, indmax ]=max(rCutdata(:));
strout =[];
[azB,elB]=ind2sub(size(rCutdata),find(mag2db(rCutdata(:))>=mag2db(maxVal)+dBpeak));
SSP = ones(size(rCutdata)+2)*eps;
SSP(2:end-1, 2:end-1) = rCutdata;
azBpAll=azB+1;
elBpAll=elB+1;

count=0;
for itb = 1:length(azBpAll)
    azBp = azBpAll(itb);
    elBp = elBpAll(itb);
    
    if      SSP(azBp,elBp)> SSP(azBp+1,elBp) & SSP(azBp,elBp) >= SSP(azBp-1,elBp) & ... %  line1
            SSP(azBp,elBp)> SSP(azBp,elBp+1) & SSP(azBp,elBp) >= SSP(azBp,elBp-1) & ... %  line2
            SSP(azBp,elBp)> SSP(azBp+1,elBp+1) & SSP(azBp,elBp) >= SSP(azBp-1,elBp-1) & ... %  line3
            SSP(azBp,elBp)> SSP(azBp+1,elBp-1) & SSP(azBp,elBp) >= SSP(azBp-1,elBp+1)  %  line4
        count = count+1;
        
        rngMax(count) = rngMaxes(azBp-1,elBp-1);
        rngMag(count) = rCutdata(azBp-1,elBp-1);
        if exist('azbins','var')
            azmp = [azbins(1); azbins(1:size(rCutdata,1)); azbins(size(rCutdata,1))];
            elmp = [elbins(1); elbins(1:size(rCutdata,1):end); elbins(end)];
            
            [ interpMagAz(count), interpBinAz(count) ] = getInterpPeak( [ SSP(azBp-1,elBp) SSP(azBp,elBp) SSP(azBp+1,elBp) ], azmp([ azBp-1 azBp azBp+1]) );
            [ interpMagEl(count), interpBinEl(count) ] = getInterpPeak( [ SSP(azBp,elBp-1) SSP(azBp,elBp) SSP(azBp,elBp+1) ], elmp([ elBp-1 elBp elBp+1]) );
            
            
            interpYm = rngMax(count)*cosd(interpBinEl(count))*sind(interpBinAz(count)); % crossrange
            interpHm = rngMax(count)*sind(interpBinEl(count)); % crossrange
            if (rngMax(count)>=peakCensorRngAzEl(1) && rngMax(count)<=peakCensorRngAzEl(2) && interpBinAz(count)>=peakCensorRngAzEl(3) && interpBinAz(count)<=peakCensorRngAzEl(4) && interpBinEl(count)>=peakCensorRngAzEl(5) && interpBinEl(count)<=peakCensorRngAzEl(6));
                strout = [strout ...
                    sprintf('R/mag (m)(dB) %0.2f   %0.2f\n',   rngMax(count), mag2db(rngMag(count))) ...
                    sprintf('Az/El (deg)  %0.2f   %0.2f\n',   interpBinAz(count), interpBinEl(count)) ...
                    sprintf('Y/Z   (m)     %0.3f   %0.3f\n\n', interpYm, interpHm)];
            end
        else
            [ interpMagAz(count), interpBinAz(count) ] = getInterpPeak( [ SSP(azBp-1,elBp) SSP(azBp,elBp) SSP(azBp+1,elBp) ], [ azBp-1 azBp azBp+1]);
            [ interpMagEl(count), interpBinEl(count) ] = getInterpPeak( [ SSP(azBp,elBp-1) SSP(azBp,elBp) SSP(azBp,elBp+1) ], [ elBp-1 elBp elBp+1]);
            if (rngMax(count)>=peakCensorRngAzEl(1) && rngMax(count)<=peakCensorRngAzEl(2) && interpBinAz(count)>=peakCensorRngAzEl(3) && interpBinAz(count)<=peakCensorRngAzEl(4) && interpBinEl(count)>=peakCensorRngAzEl(5) && interpBinEl(count)<=peakCensorRngAzEl(6));
                strout = [strout sprintf('Interp. Az Bin %0.2f\n',interpBinAz(count)-1) sprintf('Interp. El Bin %0.2f\n\n',interpBinEl(count)-1)];
            end
        end
    end
end

end

