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
function [angle_rdc, exponents] = plot_RDC3data(RDC3binFile,RDC3expFile,RangeBins,DopplerBins,AngleBins, DoppAngCut_RngBin, isComplexRDC3)
% eg.
% [rdc3plot] = plot_RDC3data('lrr1-1_rdc3.bin','lrr1-1_rdc3exp.bin',80,32,320,0,3,0)
AngExp2d = 0; % angle exponents 2d (range by doppler) or 1d (range only)
plotFFTshift = 0;
% SET DoppAngCut_RngBin<1 TO DISABLE PLOTTING
if exist('DoppAngCut_RngBin','var')
    if DoppAngCut_RngBin<1
        plotRDC3 = 0;
    else
        plotRDC3 = 1;
    end
else
    plotRDC3 = 1;
end

% complex RDC3
if ~exist('isComplexRDC3','var')
    isComplexRDC3 = 0;
end

% min is set to the max of this value and the smallest 2^exponent
minval = db2mag(10);
redhist = 1;

% File load from binary
f1 = fopen(RDC3binFile, 'r');
f2 = fopen(RDC3expFile, 'r');
if f1>0 && f2>0
    
    % Load the RDC3 exponents
    angle_exp = zeros(RangeBins, DopplerBins);
    for rr = 1:RangeBins
        if AngExp2d
            angle_exp(rr, :) = fread(f2, DopplerBins, 'int8').'; % should this be 1d or 2d
        else
            angle_exp(rr, :) = fread(f2, 1, 'int8').'; 
        end
    end
    fclose(f2);
    exponents = max(angle_exp,[],2);
    % Load the RDC3 data
    for rr = 1:RangeBins
        for dd = 1:DopplerBins
            if isComplexRDC3
                ang_vec = fread(f1, 2*AngleBins, 'int16');
                ang_vec = complex(ang_vec(1:2:end),ang_vec(2:2:end));
                angle_rdc(rr, :, dd) = abs(ang_vec) .* (2^angle_exp(rr, dd)); % just take abs of complex data for now
            else
                ang_vec = fread(f1, AngleBins, 'uint16');
                angle_rdc(rr, :, dd) = ang_vec .* (2^angle_exp(rr, dd));
            end
        end
    end
    fclose(f1);
    
    %figure;
    %mesh(squeeze(angle_rdc(:,1,:)));
    
    %figure;
    %mesh(angle_exp);
    
    % ------------
if plotRDC3
histdata = plotHistRDC(angle_rdc,redhist);
title('Histogram of nonzero sparse RDC3 magnitude (dB)')
xlabel 'Magnitude (dB)'

tooMuchdB = 150;
deltadB = max(histdata)-min(histdata);

minval = max(minval, 2^min(exponents)); 
angle_rdc_plotMin = angle_rdc;
angle_rdc_plotMin(angle_rdc_plotMin<minval) = minval;

% subplot(1,2,2)
figure;
rdc3 = mag2db(squeeze(max(angle_rdc_plotMin,[],2)));
if plotFFTshift
    surf(fftshift(rdc3,2),'FaceColor','interp');
    title 'Raw sparse RDC data, Max RoughAoA,FFT shifted in doppler'
else
    surf(rdc3,'FaceColor','interp');
    title 'Raw sparse RDC data, Max RoughAoA, plotting no FFT shift in doppler'
end
xlabel 'Doppler Bin'
ylabel 'Range Bin'
view([ 90 0 ])
axis tight;

if deltadB > tooMuchdB
    zlim([max(histdata)-deltadB max(histdata)+3]);
end

if exist('DoppAngCut_RngBin','var')
    figure,
    dap = mag2db(fftshift(squeeze(angle_rdc_plotMin(DoppAngCut_RngBin,:,:)),2));
    a=ver;
    if any(strcmp({a.Name},'MATLAB'))
        b = bar3(dap,1);
        set(b,'EdgeColor','none')
        %set(b,'LineWidth',0.1)
        for k = 1:length(b)
            zdata = b(k).ZData;
            b(k).CData = zdata;
            b(k).FaceColor = 'interp';
        end
        zoom(2.0);
        zoom reset;
        doppStep = floor(DopplerBins/8);
        xts = 1:doppStep:DopplerBins;
        %             set(gca,'XTick',xts);
        %             set(gca, 'XTickLabel', num2str(useDoppPlot(xts)','%1.1f'));
        %             alphaLen = length(detection_params.angleGatesMid);
        alphaStep = floor((AngleBins-1)/8);
        yts = 1:alphaStep:AngleBins;
        set(gca,'YTick',yts);
        %     set(gca, 'YTickLabel', detection_params.angleGatesMid(yts));
        %         set(gca,'ZTick',[0:10:100]);
        %         set(gca, 'ZTickLabel', num2str([-100:10:0]','%1.0f'));
        axis([ ...
            0.5 DopplerBins+0.5 ...
            0.5 AngleBins+0.5 ...
            0 max(dap(:))])
        axis vis3d
        view([-180 90]);
        caxis([ mag2db(minval) max(dap(:))]);
    else
        surf(dap,'LineWidth',3,'EdgeColor','interp','FaceColor','interp');

    end
    
    ylabel('Angle Bin');
    xlabel('Doppler Bin');
    title(sprintf('Doppler(fftshifted)/Angle Cut, Range bin %d', DoppAngCut_RngBin))
end
end
else
    disp(['*** Cant find RDC3 or RDC3 exponent data file ***']);
end
end
