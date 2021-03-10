% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

function [strout] =  printInterpPeaksCI(CIdata, dBpeak, peakCensorRngAzEl, xm, ym)

[maxVal, indmax ]=max(CIdata(:));
strout =[];
[xB,yB]=ind2sub(size(CIdata),find(mag2db(CIdata(:))>=mag2db(maxVal)+dBpeak));
SSP = ones(size(CIdata)+2);
SSP(2:end-1, 2:end-1) = CIdata;
xBpAll=xB+1;
yBpAll=yB+1;

count=0;
for itb = 1:length(xBpAll)
    xBp = xBpAll(itb);
    yBp = yBpAll(itb);
    
    if      SSP(xBp,yBp)> SSP(xBp+1,yBp) & SSP(xBp,yBp) >= SSP(xBp-1,yBp) & ... %  line1
            SSP(xBp,yBp)> SSP(xBp,yBp+1) & SSP(xBp,yBp) >= SSP(xBp,yBp-1) & ... %  line2
            SSP(xBp,yBp)> SSP(xBp+1,yBp+1) & SSP(xBp,yBp) >= SSP(xBp-1,yBp-1) & ... %  line3
            SSP(xBp,yBp)> SSP(xBp+1,yBp-1) & SSP(xBp,yBp) >= SSP(xBp-1,yBp+1) %  line4
        count = count+1;
        xmp = [0; xm; 0];
        ymp = [0; ym; 0];
        
        [ interpMagAng, interpBinAng ] = getInterpPeak( [ SSP(xBp-1,yBp) SSP(xBp,yBp) SSP(xBp+1,yBp) ], xmp([ xBp-1 xBp xBp+1]) );
        [ interpMagRng, interpBinRng ] = getInterpPeak( [ SSP(xBp,yBp-1) SSP(xBp,yBp) SSP(xBp,yBp+1) ], ymp([ yBp-1 yBp yBp+1]) );
        interpYm=interpBinRng*sind(interpBinAng); % crossrange
        interpXm =interpBinRng*cosd(interpBinAng); %downrange
        if (interpBinRng>=peakCensorRngAzEl(1) && interpBinRng<=peakCensorRngAzEl(2) && interpBinAng>=peakCensorRngAzEl(3) && interpBinAng<=peakCensorRngAzEl(4))
            
            strout = [strout sprintf('Interp. Range meters %0.2f\n',interpBinRng) sprintf('Interp. Angle Degrees %0.2f\n',interpBinAng) ...
                sprintf('Interp. X(m) down range %0.3f\n',interpXm) sprintf('Interp. Y(m) cross range %0.3f\n\n',interpYm)]
        end
    end
end

end