% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
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

%% Main
%
% Capture data from a radar using a command such as this:
%      capture 20 --preset VP104 --cap-hist --json-info
% Then run this program:
%      plot_DLCR_histograms('path/scan_000000_histograms.bin');
%

function [out] = plot_DLCR_histograms(histFile)

num_HistBins = 96; % 1dB to 96dB
new_HistBins = 200;

sw_exponent = 0;

[fp1, fp2, fp3] = fileparts(histFile);
SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_histograms')) 'info.json'];
[PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(SDjsonfilepath);
Rbins = PMCW.R; % number of range bins

f = fopen(histFile, 'r');
if f>0
    disp('Loading histogram data ... ')
    out.rangebins = detection_params.rangeGatesMid;
    out.Exponent = zeros(Rbins,1);
    out.histData = zeros(Rbins,num_HistBins);    
    for kk=1:Rbins
        out.histData(kk,:) = fread(f, num_HistBins, 'uint16');
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


%% Shift histograms according to exponents

new = zeros(Rbins, new_HistBins);
for rr = 1 : Rbins
    full_exp = out.Exponent(rr) + sw_exponent;
    shift_db = 6 * full_exp;
    
    new(rr,  1 + shift_db : num_HistBins + shift_db) = out.histData(rr, :);
    
    db = out.maxBinIdx(rr) + shift_db;
    out.maxBinIdx(rr) = db;
    
    [~, x] = quadratic_interpolate(new(rr,db), new(rr,db+1), new(rr,db+2));
    
    out.maxBinInterp(rr) = x + db;
end

out.histData = new;


%% Plotting

figure(100); clf; subplot(2,1,1); hold on; grid minor;
xlim([0 Rbins+10]);
plot(db2mag(out.maxBinIdx).^2,    '.-');
plot(db2mag(out.maxBinInterp).^2, '.-');
title('Noise Floor ^2  ---   LINEAR');
ylabel('Magnitude (Linear)');
xlabel('Range bin');

subplot(2,1,2); hold on; grid minor;
xlim([0 new_HistBins]);
plot(out.histData.');
title('HW DLCR Histograms (per-range-bin)');
ylabel('Count');
xlabel('dB');



end


%% Basic quadratic interpolation

function [ y , x ] = quadratic_interpolate(yl,yc,yu)

    % X coordinates
    xl = -1.0;
    xc =  0.0;
    xu =  1.0;

    % Quadratic interpolation
    d2 = ((yu - yc) / (xu - xc) - (yl - yc) / (xl - xc)) * 2.0 / (xu - xl);
    d1 = ((yu - yc) / (xu - xc)) - (d2 * (xu - xc) * 0.5);

    if (d2 == 0.0)
        x = xc;
        y = yc;
    else
        x = xc - (d1 / d2);
        y = yc + (d1 * ((x - xc) * 0.5));
    end
end
