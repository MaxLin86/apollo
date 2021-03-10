% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

function [strout, interpMagRng, interpBinRng, interpMagAng, interpBinAng] =  printInterpPeaks(staticSlices, dBpeak, peakCensorRngAzEl, rangebinsm, anglebinsd)

[maxVal indmax ]=max(staticSlices(:));
strout =[];
[rB,aB,dB]=ind2sub(size(staticSlices),find(mag2db(staticSlices(:))>=mag2db(maxVal)+dBpeak));
markedbins= zeros(length(rB));
SSP = ones(size(staticSlices)+2)*eps;
if ndims(staticSlices)==2 % one doppler bin
    SSP(2:end-1, 2:end-1) = staticSlices;
    rBpAll=rB+1;
    aBpAll=aB+1;
    count=0;
    for itb = 1:length(rBpAll)
        rBp = rBpAll(itb);
        aBp = aBpAll(itb);
        if      SSP(rBp,aBp)> SSP(rBp+1,aBp) & SSP(rBp,aBp) >= SSP(rBp-1,aBp) & ... %  line1
                SSP(rBp,aBp)> SSP(rBp,aBp+1) & SSP(rBp,aBp) >= SSP(rBp,aBp-1) & ... %  line2
                SSP(rBp,aBp)> SSP(rBp+1,aBp+1) & SSP(rBp,aBp) >= SSP(rBp-1,aBp-1) & ... %  line3
                SSP(rBp,aBp)> SSP(rBp+1,aBp-1) & SSP(rBp,aBp) >= SSP(rBp-1,aBp+1) 
            count = count+1;
            if exist('rangebinsm','var')
                rangebinsmp = [rangebinsm(1); rangebinsm; rangebinsm(end)];
                anglebinsdp = [anglebinsd(1); anglebinsd; anglebinsd(end)];
                [ interpMagRng(count), interpBinRng(count) ] = getInterpPeak( [ SSP(rBp-1,aBp) SSP(rBp,aBp) SSP(rBp+1,aBp) ], rangebinsmp([ rBp-1 rBp rBp+1]) );
                [ interpMagAng(count), interpBinAng(count) ] = getInterpPeak( [ SSP(rBp,aBp-1) SSP(rBp,aBp) SSP(rBp,aBp+1) ], anglebinsdp([ aBp-1 aBp aBp+1]));
                
            else
                [ interpMagRng(count), interpBinRng(count) ] = getInterpPeak( [ SSP(rBp-1,aBp) SSP(rBp,aBp) SSP(rBp+1,aBp) ], [ rBp-1 rBp rBp+1] );
                [ interpMagAng(count), interpBinAng(count) ] = getInterpPeak( [ SSP(rBp,aBp-1) SSP(rBp,aBp) SSP(rBp,aBp+1) ], [ aBp-1 aBp aBp+1] );
            end
            
            if (interpBinRng(count)>=peakCensorRngAzEl(1) && interpBinRng(count)<=peakCensorRngAzEl(2) && interpBinAng(count)>=peakCensorRngAzEl(3) && interpBinAng(count)<=peakCensorRngAzEl(4));
                if exist('rangebinsm','var')
                    strout = [strout sprintf('Interp. Range meters %0.2f\n',interpBinRng(count)) sprintf('Interp. Angle degrees %0.2f\n\n',interpBinAng(count))];
                else
                    strout = [strout sprintf('Interp. Range bin %0.2f\n',interpBinRng(count)-1) sprintf('Interp. Angle bin %0.2f\n\n',interpBinAng(count)-1)];
                end
            end
        end
    end
else
    SSP(2:end-1, 2:end-1, 2:end-1) = staticSlices;
    
    rBpAll=rB+1;
    dBpAll=dB+1;
    aBpAll=aB+1;
    count=0;
    for itb = 1:length(rBpAll)
        rBp = rBpAll(itb);
        aBp = aBpAll(itb);
        dBp = dBpAll(itb);
        
        if      SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp,dBp) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp,dBp) & ... %  line1
                SSP(rBp,aBp,dBp)> SSP(rBp,aBp+1,dBp) & SSP(rBp,aBp,dBp) >= SSP(rBp,aBp-1,dBp) & ... %  line2
                SSP(rBp,aBp,dBp)> SSP(rBp,aBp,dBp+1) & SSP(rBp,aBp,dBp) >= SSP(rBp,aBp,dBp-1) & ... %  line3
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp+1,dBp) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp-1,dBp) & ... %  line4
                SSP(rBp,aBp,dBp)> SSP(rBp,aBp+1,dBp+1) & SSP(rBp,aBp,dBp) >= SSP(rBp,aBp-1,dBp-1) & ... %  line5
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp,dBp+1) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp,dBp-1) & ... %  line6
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp-1,dBp) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp+1,dBp) & ... %  line7
                SSP(rBp,aBp,dBp)> SSP(rBp,aBp+1,dBp-1) & SSP(rBp,aBp,dBp) >= SSP(rBp,aBp-1,dBp+1) & ... %  line8
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp,dBp-1) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp,dBp+1) & ... %  line9
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp+1,dBp-1) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp-1,dBp+1) & ... %  line10
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp-1,dBp+1) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp+1,dBp-1) & ... %  line11
                SSP(rBp,aBp,dBp)> SSP(rBp-1,aBp+1,dBp+1) & SSP(rBp,aBp,dBp) >= SSP(rBp+1,aBp-1,dBp-1) & ... %  line12
                SSP(rBp,aBp,dBp)> SSP(rBp+1,aBp+1,dBp+1) & SSP(rBp,aBp,dBp) >= SSP(rBp-1,aBp-1,dBp-1) %  line13
            count = count+1;
            if exist('rangebinsm','var')
                rangebinsmp = [0; rangebinsm; 0];
                anglebinsdp = [0; anglebinsd; 0];
                [ interpMagRng(count), interpBinRng(count) ] = getInterpPeak( [ SSP(rBp-1,aBp,dBp) SSP(rBp,aBp,dBp) SSP(rBp+1,aBp,dBp) ], rangebinsmp([ rBp-1 rBp rBp+1]) );
                [ interpMagAng(count), interpBinAng(count) ] = getInterpPeak( [ SSP(rBp,aBp-1,dBp) SSP(rBp,aBp,dBp) SSP(rBp,aBp+1,dBp) ], anglebinsdp([ aBp-1 aBp aBp+1]));
                
            else
                [ interpMagRng(count), interpBinRng(count) ] = getInterpPeak( [ SSP(rBp-1,aBp,dBp) SSP(rBp,aBp,dBp) SSP(rBp+1,aBp,dBp) ], [ rBp-1 rBp rBp+1] );
                [ interpMagAng(count), interpBinAng(count) ] = getInterpPeak( [ SSP(rBp,aBp-1,dBp) SSP(rBp,aBp,dBp) SSP(rBp,aBp+1,dBp) ], [ aBp-1 aBp aBp+1] );
            end
            [ interpMagDop(count), interpBinDop(count) ] = getInterpPeak( [ SSP(rBp,aBp,dBp-1) SSP(rBp,aBp,dBp) SSP(rBp,aBp,dBp+1) ], [ dBp-1 dBp dBp+1] );
            
            if (interpBinRng(count)>=peakCensorRngAzEl(1) && interpBinRng(count)<=peakCensorRngAzEl(2) && interpBinAng(count)>=peakCensorRngAzEl(3) && interpBinAng(count)<=peakCensorRngAzEl(4))
                if exist('rangebinsm','var')
                    strout = [strout sprintf('Interp. Range meters %0.2f\n',interpBinRng(count)) sprintf('Interp. Doppler bin %0.2f\n',interpBinDop(count)-1)  sprintf('Interp. Angle degrees %0.2f\n\n',interpBinAng(count))];
                else
                    strout = [strout sprintf('Interp. Range bin %0.2f\n',interpBinRng(count)-1) sprintf('Interp. Doppler bin %0.2f\n',interpBinDop(count)-1)  sprintf('Interp. Angle bin %0.2f\n\n',interpBinAng(count)-1)];
                    
                end
            end
        end
    end
end
end

