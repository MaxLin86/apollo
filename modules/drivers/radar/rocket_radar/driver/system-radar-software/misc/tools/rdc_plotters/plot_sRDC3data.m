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
function [sparsifiedOutput, exponents] = plot_sRDC3data(filestr,ULSstr,binAxes,DoppAngCut_RngBin,RangeBins,AngleBins,DopplerBins,plotdBFS)
% ULSstr is a string containing the types of activation data files to load
% u == upper, l== lower, s== special summary and RDC data
% If any sparsified file summary file isnt available that ULS type will be skipped

% Send a PROPERLY NAMED! (SEE CONFLUENCE FOR NAMING) spsum*.bin file
% For complex input include a complex, 'c' summary filename eg scan_000001_spsumlc.bin
% e.g. plot_sRDC3data('\\192.168.44.114\uhnder-nas\software\jonathan\ADC_2016-10-03_18-24-07_scan90\scan90_spsumu.bin','ul',256,141,2048,28);
plotFFTshift = 0;

if ~exist('ULSstr','var')
    ULSstr = 'u';
end

if ~exist('plotdBFS','var')
    plotdBFS = 0;
end

% SET DoppAngCut_RngBin<1 TO DISABLE PLOTTING
if exist('DoppAngCut_RngBin','var')
    if ischar(DoppAngCut_RngBin)
        DoppAngCut_RngBin=str2num(DoppAngCut_RngBin);
    end
    if DoppAngCut_RngBin<1
        plotRDC3 = 0;
    else
        plotRDC3 = 1;
    end
else
    plotRDC3 = 1;
end

if ~exist('RangeBins','var') || isempty(RangeBins) || RangeBins==0
    % file parts
    [fp1, fp2, fp3] = fileparts(filestr);
    SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_spsum')) 'info.json'];
    [PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(SDjsonfilepath);
    RangeBins = length(detection_params.rbininfo.rangeorder);
    AngleBins =  length(detection_params.angleGatesMid);
    DopplerBins = length(detection_params.dopplerGatesMidFFT);
    if ~exist('binAxes','var')
        binAxes = 0;
    end
else
    if ~exist('binAxes','var')
        binAxes = 1;
    end
end

%string to number conversion needed for executable input parameters 
if ischar(binAxes)
    binAxes=str2num(bunAxes);
end
if ischar(RangeBins)
    RangeBins=str2num(RangeBins);
end
if ischar(AngleBins)
    AngelBins=str2num(AngleBins);
end
if ischar(DopplerBins)
    DopplerBins=str2num(DopplerBins);
end
if ischar(plotdBFS)
    plotdBFS=str2num(plotdBFS);
end
% Calc dBFS offsets, this will match SRS dBFS
if plotdBFS==2
    dBFScorr_lin.RDC1 = dBFScorr_lin.RDC1max;
    dBFScorr_lin.RDC2 = dBFScorr_lin.RDC2max;
    dBFScorr_lin.RDC3 = dBFScorr_lin.RDC3max;
end


% tighten axes to populated rdbins
tightaxes = 0;
redhist = 1;

% min is set to the max of this value and the smallest 2^exponent
minval = db2mag(10);

% filestr(1:max(findstr(filestr,'_')));
angle_rdc_plot = zeros(RangeBins,AngleBins,DopplerBins);
exponents = -1*ones(1,RangeBins);
numFoundCells = 0; % total found cell count
foundCellsFile = 0; % track found cells per file

fileIDstr=filestr(max(findstr(filestr,'_'))+1:end-4);
if isempty(strfind(fileIDstr,'sum'))
    error('File Identifier string must be a correctly named summary file, eg scan_000001_spsumuc.bin')
end

isComplexRDC3 = 0;

% complex RDC3 check
if ~isempty(strfind(fileIDstr,'c'))
    if ~isempty(strfind(fileIDstr,'h'))
        error('Do not use a channelizer summary file for RDC3 plotting')
    end
    isComplexRDC3 = 1;
end

for itf = 1:length(ULSstr)
    if isComplexRDC3
        sRDC3sumFile = [filestr(1:max(findstr(filestr,'_'))) 'spsum' ULSstr(itf) 'c.bin' ];
        sRDC3binFile = [filestr(1:max(findstr(filestr,'_'))) 'sprdc3' ULSstr(itf) 'c.bin' ];
    else
        sRDC3sumFile = [filestr(1:max(findstr(filestr,'_'))) 'spsum' ULSstr(itf) '.bin' ];
        sRDC3binFile = [filestr(1:max(findstr(filestr,'_'))) 'sprdc3' ULSstr(itf) '.bin' ];
    end
f = fopen(sRDC3sumFile, 'r');
if f>0
    a=dir(sRDC3sumFile); % for file size
    numSkewers = a.bytes/8;
    for sk = 1:numSkewers
        % zero based range and doppler bin data to 1based
        sparsifiedOutput.activationDopplerBin(foundCellsFile+sk, 1) = fread(f, 1, 'uint16')+1;
        sparsifiedOutput.activationRangeBin(foundCellsFile+sk, 1) =  fread(f, 1, 'uint16')+1;
        % max mag from sparsified non-static RDC3
        sparsifiedOutput.activationMaxMag(foundCellsFile+sk, 1) = fread(f, 1, 'uint16');
        % RDC2 FFT exponent that goes along with that RDC3 max mag
        sparsifiedOutput.RDC2exp(foundCellsFile+sk, 1) = fread(f, 1, 'int8');
        fread(f, 1, 'uint8');% pad
    end
    fclose(f);
    
    if numSkewers== (RangeBins*DopplerBins)
        disp([ num2str(numSkewers) ' activations in this file, FULL SET!']);
    else
        disp([ num2str(numSkewers) ' activations in this file'])
    end
    
    f = fopen(sRDC3binFile, 'r');
    if isComplexRDC3
        temp = fread(f,2*AngleBins*numSkewers,'int16');
        temp = complex(temp(1:2:end),temp(2:2:end));
        sparsifiedOutput.RDC3Activations(foundCellsFile+1:foundCellsFile+numSkewers,:) = transpose(reshape(temp,[AngleBins numSkewers]) );
    else
        sparsifiedOutput.RDC3Activations(foundCellsFile+1:foundCellsFile+numSkewers,:) = transpose(reshape(fread(f,AngleBins*numSkewers,'uint16'),[AngleBins numSkewers]) );
    end
    fclose(f);
    
    % just abs complex data for now
    sparsifiedOutput.RDC3Activations = abs(sparsifiedOutput.RDC3Activations);
    
    %Data plot from RoughAoA plot
    if numSkewers>0
    for irng=1:RangeBins
        for idop =1:DopplerBins
            foundcell=(sparsifiedOutput.activationRangeBin(foundCellsFile+1:end) == irng & sparsifiedOutput.activationDopplerBin(foundCellsFile+1:end) == idop);
            fcells = find(foundcell);
            if length(fcells)>1
                error('bad summary data found')
%                 angle_rdc_plot(irng,:,idop)=minval;
            else
                if ~isempty(fcells)
                    numFoundCells = numFoundCells+1;
                    sparsifiedOutput.RDC3Activations(foundCellsFile+fcells, :) = transpose(sparsifiedOutput.RDC3Activations(foundCellsFile+fcells, :).*2^sparsifiedOutput.RDC2exp(foundCellsFile+fcells)).*2^swExponent.RDC3;
                    
                    
                    angle_rdc_plot(irng,:,idop)= sparsifiedOutput.RDC3Activations(foundCellsFile+fcells, :);

                    if plotdBFS
                       angle_rdc_plot(irng,:,idop)= angle_rdc_plot(irng,:,idop)/dBFScorr_lin.RDC3(sparsifiedOutput.activationRangeBin(foundCellsFile+fcells));
                    end
                    if exponents(irng)==-1;
                        exponents(irng)=sparsifiedOutput.RDC2exp(foundCellsFile+fcells);
                    else
                        assert( exponents(irng)==sparsifiedOutput.RDC2exp(foundCellsFile+fcells),'Inconsistent Sparsified Exponents');
                    end
                else
%                     angle_rdc_plot(irng,:,idop)=minval;
                end
            end
        end
    end
    end
else
    disp(['*** Cant find RDC3 data file ***']);
end
% check that all present data is plotted
if exist('numSkewers','var')
    if numSkewers>(numFoundCells-foundCellsFile)
        warning('All input skewers not in valid range doppler bins');
    end
end
foundCellsFile=numFoundCells;
end


% a=figure;
% set(a,'Position',[50 400 1200 400])
% subplot(1,2,1)

angle_rdc_plot = angle_rdc_plot(detection_params.rangeGatesOrdering,:,:);

if plotRDC3
histdata = plotHistRDC(angle_rdc_plot,redhist);
title('Histogram of nonzero sparse RDC3 magnitude (dB)')
xlabel 'Magnitude (dB)'

tooMuchdB = 150;
deltadB = max(histdata)-min(histdata);

minval = max(minval, 2^min(sparsifiedOutput.RDC2exp)); 
if plotdBFS
    minval = minval/max(dBFScorr_lin.RDC3);
end

                    
angle_rdc_plotMin = angle_rdc_plot;
angle_rdc_plotMin(angle_rdc_plotMin<minval) = minval;

% subplot(1,2,2)
figure;
rdc3 = mag2db(squeeze(max(angle_rdc_plotMin,[],2)));
if binAxes
    if plotFFTshift
        surf(fftshift(rdc3,2),'FaceColor','interp','EdgeColor','none');
        title 'Raw sparse RDC data, Max RoughAoA,FFT shifted in doppler'
    else
        surf(rdc3,'FaceColor','interp','EdgeColor','none');
        title 'Raw sparse RDC data, Max RoughAoA'
    end
    xlabel 'Doppler Bin'
    ylabel 'Range Bin'
else
    if plotFFTshift
        error('ToDo: dopplerFFTshift on non-bin doppler axes');
    else
        surf(detection_params.dopplerGatesMidFFT,detection_params.rangeGatesMid,rdc3,'FaceColor','interp','EdgeColor','none');
        title 'Raw sparse RDC data, Max RoughAoA'
    end
    xlabel 'Doppler (mps)'
    ylabel 'Range (m)'
end
view([ 90 0 ])
axis tight;
colormap jet;
caxis([median(rdc3(:)) max(rdc3(:))-3]);

if tightaxes
    % tighten axes to populated activations
    if(min(sparsifiedOutput.activationDopplerBin)~=max(sparsifiedOutput.activationDopplerBin))
        if plotFFTshift
            mapDbin=fftshift(1:DopplerBins);
        else
            mapDbin=(1:DopplerBins);
        end
        xlim([min(mapDbin(sparsifiedOutput.activationDopplerBin)) max(mapDbin(sparsifiedOutput.activationDopplerBin))]);
    end
    if(min(sparsifiedOutput.activationRangeBin)~=max(sparsifiedOutput.activationRangeBin))
        ylim([ min(sparsifiedOutput.activationRangeBin) max(sparsifiedOutput.activationRangeBin)]);
    end
end
if deltadB > tooMuchdB
    zlim([max(histdata)-deltadB max(histdata)+3]);
end

if exist('DoppAngCut_RngBin','var')
    figure,
    if plotFFTshift
        if ~binAxes
            error('ToDo: dopplerFFTshift on non-bin doppler axes');
        end
        dap = mag2db(fftshift(squeeze(angle_rdc_plotMin(DoppAngCut_RngBin,:,:)),2));
    else
        dap = mag2db(squeeze(angle_rdc_plotMin(DoppAngCut_RngBin,:,:)));
    end
    a=ver;
    if 0
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
        if max(dap(:))>mag2db(minval)
            caxis([ mag2db(minval) max(dap(:))]);
        end
    else
        
        
        if binAxes
            
            surf(dap,'LineWidth',3,'EdgeColor','interp','FaceColor','interp');
        else
            if PMCW.MIMO2D
                % for 2d just plot angle bins in the activation plot
                surf(detection_params.dopplerGatesMidFFT,1:size(detection_params.angleGatesMid,1),dap,'LineWidth',3,'EdgeColor','interp','FaceColor','interp');
                xlabel 'Doppler (mps)'
                ylabel 'Angle Bin'
            else
                surf(detection_params.dopplerGatesMidFFT,rad2deg(detection_params.angleGatesMid(:,1)),dap,'LineWidth',3,'EdgeColor','interp','FaceColor','interp');                
                xlabel 'Doppler (mps)'
                ylabel 'Azimuth Angle (deg)'
            end
        end
        
    end
    
    axis tight
    if plotFFTshift
        title(sprintf('Doppler(fftshifted)/Angle Cut, Range bin %d', DoppAngCut_RngBin))
    else
        title(sprintf('Doppler/Angle Cut, Range bin %d', DoppAngCut_RngBin))
    end
    shading flat
    colormap jet
    view([0 90])
end
end

RangeBins = length(detection_params.rangeGatesMid);
correctedActivationRangeBin = detection_params.rbininfo.rangeorder(sparsifiedOutput.activationRangeBin).'+1;

% remove invalid activations
goodact = correctedActivationRangeBin>0;
sparsifiedOutput.activationRangeBin = correctedActivationRangeBin(goodact);
sparsifiedOutput.activationDopplerBin = sparsifiedOutput.activationDopplerBin(goodact);
sparsifiedOutput.activationMaxMag = sparsifiedOutput.activationMaxMag(goodact);
sparsifiedOutput.RDC2exp = sparsifiedOutput.RDC2exp(goodact);
sparsifiedOutput.RDC3Activations = sparsifiedOutput.RDC3Activations(goodact,:);

sparsifiedOutput.detection_params = detection_params;
