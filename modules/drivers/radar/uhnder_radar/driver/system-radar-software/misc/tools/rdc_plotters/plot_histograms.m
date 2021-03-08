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
function [histData,Exponent,Rbins] = plot_histograms(histFile,histbmFile,plotRbins,allowPlots,sparsThrBinfile)
%%% Plot the rdc3 histogram and sparsifier thresholds
%%% e.g., plot_histograms('\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\sampledata\scan_000001_hist.bin','\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\sampledata\scan_000001_histbm.bin',[49:50],1,'C:\Uhnder\BitBucket\sam-sim-clone\testVectors\verifVectors\OEMscan1case3_UMSK_Lc4096_N360_sRDC3\scan_000001_spthrlin.bin')
% if plotRbins is populated, then we only plot those rbins, 1 based
% plotting thresholds (provided a good  sparsThrBinfile) for many range
% bins is not recommended

shiftExponents = 1;
if ~exist('allowPlots','var')
   allowPlots=1;
end
f = fopen(histFile, 'r');
if f>0
    disp('Loading histogram data ... ')
    f2 = fopen(histbmFile, 'r');
    bitmap = fread(f2, 'uint32');
    bitmap = dec2bin(flipud(bitmap));
    fclose(f2);
    Rbins = length(find(bitmap=='1')); % number of range bins
    Exponent = zeros(Rbins,1);
    histData = zeros(Rbins,96);    
    for kk=1:Rbins
        histData(kk,:) = fread(f, 96, 'uint16');
        word64_bin = dec2bin(fread(f,1,'uint64'),64);
        Exponent(kk) = bin2dec(word64_bin(end-7:end));
        if shiftExponents
            histData(kk,:) = [zeros(1,round(mag2db(2^Exponent(kk)))) histData(kk,1:end-round(mag2db(2^Exponent(kk))))];
        end
    end
    fclose(f);

    if allowPlots > 0
    clines = colormap('lines');
    if nargin>2
        figure,         
        hax=axes;
        plot(0:95,histData(plotRbins,:),'.-')
        legend(num2str(plotRbins'));
    else
        figure,
        hax=axes;
        plot(0:95,histData,'.-')
        legend(num2str([1:Rbins]'));
    end
    xlim([0,97]),xlabel('dB'),ylabel('Count')
    if shiftExponents
        title('HW Histogram Data per Rangebin, Exponent Corrected');
    else
        title('HW Histogram Data per Rangebin');
    end
    end
else
    disp('Histogram data cannot be loaded!!! ')
end
if nargin>4 && allowPlots > 0
    disp('Plotting threshold data ')
        title('HW Histogram Data per Rangebin with Thresholds')
    hold;
    % Threshold data
    f = fopen(sparsThrBinfile, 'r');
    if f>0
        
        for rr = 1:Rbins
            mantissaL = fread(f, 1, 'uint8');
            expL = fread(f, 1, 'int8');
            mantissaU = fread(f, 1, 'uint8');
            expU = fread(f, 1, 'int8');
            
            %calculate thresholds, samSim uses the noisefloor and
            %sp_params.FirstactivationThreshdB to create peaks
            thrshU(rr)=mantissaU*2^(expU+8);
            thrshL(rr)=mantissaL*2^(expL+8);
            
            % calc noise floor from the threshold...
%             noiseFloor(rr) = mag2db(thrshU(rr)) - sp_params.FirstactivationThreshdB;
%             checknoiseFloor = mag2db(thrshL(rr)) - sp_params.SecondactivationThreshdB;
%             assert(noiseFloor(rr)-checknoiseFloor<1,'Threshold/Noise floor check has failed, more that 1dB delta ')
        end
        fclose(f);
        rrin = plotRbins;

        colorl=1;
        for rr = rrin
%             line([noiseFloor(rr) noiseFloor(rr)], get(hax,'YLim'), 'Color', 'r');
            curcolor = mod(colorl,64);
            if curcolor == 0, curcolor=1; end
            if shiftExponents
                shU = mag2db(thrshU(rr))+round(mag2db(2^Exponent(rr)));
                shL = mag2db(thrshL(rr))+round(mag2db(2^Exponent(rr)));
                line([mag2db(thrshU(rr)) mag2db(thrshU(rr))]+round(mag2db(2^Exponent(rr))), get(hax,'YLim'), 'Color', clines(curcolor,:));
                line([mag2db(thrshL(rr)) mag2db(thrshL(rr))]+round(mag2db(2^Exponent(rr))), get(hax,'YLim'), 'Color', clines(curcolor,:));
                
            else
                line([mag2db(thrshU(rr)) mag2db(thrshU(rr))], get(hax,'YLim'), 'Color', clines(curcolor,:));
                line([mag2db(thrshL(rr)) mag2db(thrshL(rr))], get(hax,'YLim'), 'Color', clines(curcolor,:));
            end
            colorl=colorl+1;
        end
        
    else
        disp('Threshold data cannot be loaded!!! ')
    end
end
end
