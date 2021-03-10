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