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
function [ RDC2out, exponents ] = plot_iRDC2data(RDC2binFile,RDC2expFile,RangeBins,Nvrx,DopplerBins,DCiters,plotVrx,DbinPadBytes)
% eg.
% [rdc2plot] = plot_iRDC2data('\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\forZC\samsimGendata\lrr1-1_rdc2.bin','\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\forZC\samsimGendata\lrr1-1_rdc2exp.bin',80,32,320,0,3,0)
% [rdc2plot] = plot_iRDC2data('\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\forZC\samsimGendata\lrr1-1_rdc2ch.bin','\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\forZC\samsimGendata\lrr1-1_rdc2expch.bin',80,32,128,2,1,0)
% input parameters: R=16, Iter=11, Vrx=96, Ch_dopp=72, 1, 0

% min is set to the max of this value and the smallest 2^exponent
minval = db2mag(10);
redhist =1; 
dofftshift=0;
if ~exist('plotVrx','var')
    plotVrx = 1;
end

if ~exist('DbinPadBytes','var')
    DbinPadBytes = 8;
end

%string to number conversion needed for executable input parameters 
if ischar(RangeBins)
    RangeBins=str2num(RangeBins);
end
if ischar(Nvrx)
    Nvrx=str2num(Nvrx);
end
if ischar(DopplerBins)
    DopplerBins=str2num(DopplerBins);
end
if ischar(DCiters)
    DCiters=str2num(DCiters);
end
if ischar(plotVrx)
    plotVrx=str2num(plotVrx);
end
if ischar(DbinPadBytes)
    DbinPadBytes=str2num(DbinPadBytes);
end

% File load from binary
f1 = fopen(RDC2binFile, 'r');
f2 = fopen(RDC2expFile, 'r');
if f1>0 && f2>0
    disp('Loading RDC2')
    if DCiters > 0
        % channelizer data, todo make this faster with block load/reshape
        for ii = 1:RangeBins
            for kk = 1:DopplerBins
                for ll = 1:DCiters
                    for jj = 1:Nvrx
                        nr = fread(f1, 1, '*int16');
                        ni = fread(f1, 1, '*int16');
                        doppler_rdc(ii, jj, kk, ll) = complex(nr, ni);
                    end
                    pad = fread(f1, DbinPadBytes, '*int8');
                end
            end
        end
        fclose(f1);
        
        
        dopplerTransExp = int8(ones(RangeBins,Nvrx,DCiters));
        % Load the Doppler Channelizer exponents for MUSIC
        disp('Loading RDC1 data and exponents')
        for rr = 1:RangeBins
            for dr = 1:DCiters
                for vr = 1:Nvrx
                    dopplerTransExp(rr, vr, dr) = fread(f2, 1, 'int8');
                    % iRDC2 exponent is 32 bit wide of which only the lower 8 bits are valid.
                    % Rest of the 24 bits are don't cares
                    fread(f2, 1, 'int8'); 
                    fread(f2, 1, 'int8');
                    fread(f2, 1, 'int8');
                end
            end
        end
        fclose(f2);
        
        
        % quick rdc2 exp correction
        dopplerTransExp = double(dopplerTransExp);
        maxDCTransExp = max(max(dopplerTransExp,[],3),[],2);
        RDC2DCiter_scaleFactor = dopplerTransExp-repmat(maxDCTransExp,[1 size(dopplerTransExp,2),size(dopplerTransExp,3)]);
        
        tempsizes = size(doppler_rdc);
        % normalize it to 16 bit values
        doppler_rdc = double(doppler_rdc) .* 2.^permute(repmat(RDC2DCiter_scaleFactor, [1 1 1 tempsizes(3)]),[1 2 4 3]);
        % bring up all vrxs together
        doppler_rdc = double(doppler_rdc) .* 2.^ repmat(maxDCTransExp, [1 tempsizes(2:end)]);
        RDC2out = doppler_rdc;
        exponents = maxDCTransExp.';        
        if plotVrx>0
            doppler_rdc_plot_Vrx = squeeze(max(abs(doppler_rdc(:,plotVrx,:,:)),[],4));
        end
    else
        % fft data
        for ii = 1:RangeBins
            for jj = 1:DopplerBins
                dd = fread(f1,Nvrx*2, '*int16');
                dd = dd(1:Nvrx*2);
                tmpdata = complex(dd(1:2:end), dd(2:2:end));
                tmp_doppler_rdc(ii,:,jj)=tmpdata;
                pad = fread(f1, DbinPadBytes, '*int8');
            end
        end
        fclose(f1);
        
        % Load the RDC2 FFT exponents for detection processing path
        for rr = 1:RangeBins
            for vr = 1:Nvrx
                rough_dopplerTransExp(rr, vr) = fread(f2, 1, 'int8');
                fread(f2, 1, 'uint8'); % 8bit dont-care
                fread(f2, 1, 'uint8'); % 8bit dont-care
                fread(f2, 1, 'uint8'); % 8bit dont-care
            end
        end
        fclose(f2);
        
        % quick rdc2 exp correction
        rough_dopplerTransExp = double(rough_dopplerTransExp);
        maxRoughDopTransExp =  max(rough_dopplerTransExp,[],2);
        RDC2FFT_scaleFactor = rough_dopplerTransExp-repmat(maxRoughDopTransExp,[1 Nvrx]);
        tempsizes = size(tmp_doppler_rdc);
        % normalize it to 16 bit values
        tmp_doppler_rdc = double(tmp_doppler_rdc) .* 2.^repmat(RDC2FFT_scaleFactor, [1 1 tempsizes(3:end)]);
        % bring up all vrxs together
        tmp_doppler_rdc = double(tmp_doppler_rdc) .* 2.^ repmat(maxRoughDopTransExp, [1 tempsizes(2:end)]);
        RDC2out = tmp_doppler_rdc;
        exponents = maxRoughDopTransExp.';        
        if plotVrx>0
            doppler_rdc_plot_Vrx = abs(squeeze(tmp_doppler_rdc(:,plotVrx,:)));
        end
    end
    
    if plotVrx>0
    histdata = plotHistRDC(doppler_rdc_plot_Vrx,redhist);

    title(['Histogram of nonzero iRDC2 magnitude (dB) Vrx:' num2str(plotVrx) ])
    xlabel 'Magnitude (dB)'
    
    tooMuchdB = 150;
    deltadB = max(histdata)-min(histdata) ;
    
    if DCiters==0
        minval = max(minval, 2^min(maxRoughDopTransExp)); % set minval to min exponent
    else
        minval = max(minval, 2^min(maxDCTransExp));
    end
    doppler_rdc_plot_Vrx(abs(doppler_rdc_plot_Vrx)<minval) = minval;
    %     subplot(1,2,2)
    figure,
    if DCiters == 0
        rdc2p = mag2db(doppler_rdc_plot_Vrx);
        if dofftshift
            surf(fftshift(rdc2p,2),'FaceColor','interp');
            title(['Raw iRDC2 Doppler FFT data, VRx: ' num2str(plotVrx) ', FFTshifted in doppler'])
        else
            surf(rdc2p,'FaceColor','interp');
            title(['Raw iRDC2 Doppler FFT data, VRx: ' num2str(plotVrx) ', no FFTshift in plotter'])
        end
        xlabel 'Doppler Bin'
        view([ 90 0 ])
    else
        figure,
        rdc2p = mag2db(doppler_rdc_plot_Vrx);
        if dofftshift
            surf(fftshift(rdc2p,2),'FaceColor','interp');
            title(['Raw iRDC2 Channelizer data, VRx: ' num2str(plotVrx) ', Max DCiters, FFTshifted in doppler'])
        else
            surf(rdc2p,'FaceColor','interp');
            title(['Raw iRDC2 Channelizer data, VRx: ' num2str(plotVrx) ', Max DCiters, no FFTshift in plotter'])
        end
        title(['Raw iRDC2 Channelizer data, VRx: ' num2str(plotVrx) ', Max DCiters, FFTshifted in doppler'])
        xlabel 'DopplerChan Bin'
        view([ 90 0 ])
    end
    axis tight;
    if deltadB > tooMuchdB
        zlim([max(histdata)-deltadB max(histdata)+3]);
    end
    ylabel 'Range Bin'
    end
else
    disp(['*** Cant find RDC2 or exp data file ***']);
end
end
