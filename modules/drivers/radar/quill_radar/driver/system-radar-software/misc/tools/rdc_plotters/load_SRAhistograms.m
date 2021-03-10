% START_SOFTWARE_LICENSE_NOTICE
% -------------------------------------------------------------------------------------------------------------------
% Copyright (C) 2019 Uhnder, Inc. All rights reserved.
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
function [out] = load_SRAhistograms(histFile)

numHistBins=96; % 1dBto 96dB

[fp1, fp2, fp3] = fileparts(histFile);
SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_histograms')) 'info.json'];
[PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(SDjsonfilepath);
Rbins = PMCW.R; % number of range bins

f = fopen(histFile, 'r');
if f>0
    disp('Loading histogram data ... ')
    out.rangebins = detection_params.rangeGatesMid;
    out.Exponent = zeros(Rbins,1);
    out.histData = zeros(Rbins,96);    
    for kk=1:Rbins
        out.histData(kk,:) = fread(f, numHistBins, 'uint16');
        out.Exponent(kk) = fread(f,1,'int8');
        out.maxBinIdx(kk) = fread(f,1,'uint8');
        reserved = fread(f,1,'uint16');
        crc = fread(f,1,'uint32');
    end
    fclose(f);

else
    disp('Histogram data cannot be loaded!!! ')
end


%% Correct histogram structure for range bin ordering
goodmask = detection_params.rbininfo.rangeorder>=0;
reorder = detection_params.rbininfo.rangeorder(goodmask)+1;
out.Exponent = out.Exponent(goodmask);
out.Exponent = out.Exponent(reorder);

out.histData = out.histData(goodmask,:);
out.histData = out.histData(reorder,:);

out.maxBinIdx = out.maxBinIdx(goodmask);
out.maxBinIdx = out.maxBinIdx(reorder);

end
