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

