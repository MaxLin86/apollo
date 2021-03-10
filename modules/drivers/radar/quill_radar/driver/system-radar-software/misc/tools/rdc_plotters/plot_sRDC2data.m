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
function [sparsifiedOutput, doppler_rdc_plot, exponents] = plot_sRDC2data(filestr,ULSstr,RangeBins,Nvrx,DopplerBins,DCiters,plotVrx,plotDCiter)
% filestr = string, file path to a of the PROPERLY NAMED! (SEE CONFLUENCE FOR NAMING) spsum*.bin or spRDC3*.bin file
% ULSstr is a string containing the types of activation data files to load
%   u == upper, l== lower, s== special summary and RDC data
%   if any sparsified file summary file isnt available that ULS type will be skipped
% RangeBins = scan_info.num_range_bins
% Nvrx = scan_info.total_vrx
% DopplerBins = scan_info.num_pulses
% DCiters = scan_info.num_channelizer_iters
% plotVrx, Vrx to plot
% plotDCiter, DC iteration to plot

% e.g.channelizer
% plot_sRDC2data('\\192.168.44.114\srs-data\Drive5_minimal\scan_000002\scan_000002_spsumu.bin','u',80,64,128,3,1);
% e.g. fft
% plot_sRDC2data('\\192.168.44.114\uhnder-nas\software\jonathan\ADC_2016-10-03_18-24-07_scan90\scan90_spsumu.bin','uls',256,28,2048,0,1);

% tighten axes to populated rdbins
tightaxes = 0;
redhist = 1;
% min is set to the max of this value and the smallest 2^exponent
minval = db2mag(10);
dofftshift=0;
shiftFilledVrxs = 1;

if ~exist('RangeBins','var') || isempty(RangeBins) || RangeBins==0
    [fp1 fp2 fp3 ] = fileparts(filestr);
    scanData=loadjson([fp1 '/' fp2(1:findstr(fp2,'_sp')) 'info.json']);
    RangeBins = scanData.num_range_bins;
    Nvrx = scanData.total_vrx;
    DopplerBins = scanData.num_channelizer_doppler_bins;
    DCiters = scanData.num_channelizer_iters;
    plotVrx = 4;
    plotDCiter = 1;
    ULSstr = 'us';
end

exponents = -1*ones(1,RangeBins);

[fp1, fp2, fp3] = fileparts(filestr);
if nargin<6
    disp('*** Reading scan info file ***');
    scaninfofile = [fp1 '/' fp2(1:findstr(fp2,'_spsumch')) 'info.json'];
    [~, ~, detection_params, ~, ~, ~] = processSabineScanInfo(scaninfofile);
else
    detection_params.rangeGatesOrdering = 1:RangeBins;
end

% set DC iters 0 to plot FFT data
% rdc2fft=plot_sRDC2data('\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\forZC\ss1SamSimBringup\dvVectors-1.spSumCHAN','\\192.168.44.114\uhnder-nas\general\SaMSim-TestData\forZC\ss1SamSimBringup\dvVectors-1.spRDC2CHAN',128,64,8,0,2)
% plot_sRDC2data('C:\work\oem1_srdc2\output_rdc2_exp.bin','C:\work\oem1_srdc2\output_rdc2_data.bin',128,64,8,0,1);
if ~exist('plotVrx','var')
    plotVrx = 1;
end
if ~exist('plotDCiter','var')
    plotDCiter = 1;
end
if DCiters > 0
    doppler_rdc_plot = zeros(RangeBins,Nvrx,DopplerBins,DCiters);
    disp('Channelizer data being displayed')
else
    doppler_rdc_plot = zeros(RangeBins,Nvrx,DopplerBins);
    disp('FFT data being displayed')
end

numFoundCells = 0; % total found cell count
foundCellsFile = 0; % track found cells per file

for itf = 1:length(ULSstr)
    numSkewers = 0;
    disp(['Reading ' ULSstr(itf)]);
    % create appropriate file strings
    if DCiters > 0
        if  ULSstr(itf)=='l'
            error('lower channelizer data not supported')
        end
        sRDC2sumFile = [filestr(1:max(findstr(filestr,'_'))) 'spsumch'  ULSstr(itf) '.bin' ];
        sRDC2binFile = [filestr(1:max(findstr(filestr,'_'))) 'sprdc2ch'  ULSstr(itf) '.bin' ];
    else
        sRDC2sumFile = [filestr(1:max(findstr(filestr,'_'))) 'spsum' ULSstr(itf) '.bin' ];
        sRDC2binFile = [filestr(1:max(findstr(filestr,'_'))) 'sprdc2' ULSstr(itf) '.bin' ];
    end
    if DCiters > 0
        
        % Channelizer data
        f = fopen(sRDC2sumFile, 'r');
        if f>0;
            a=dir(sRDC2sumFile); % for file size
            numSkewers = a.bytes/8;
            for sk = 1:numSkewers
                % zero based range and doppler bin data
                sparsifiedOutput.activationDopplerBinChan(foundCellsFile+sk, 1) = fread(f, 1, 'uint16')+1;
                temp = fread(f, 1, 'uint16')+1;
%                 temp1 = fread(f, 1, 'uint16')+1;
%                 temp2 = detection_params.rangeGatesOrdering(detection_params.rangeGatesMid>=0);
%                 temp2 = temp2-min(temp2);
                sparsifiedOutput.activationRangeBinChan(foundCellsFile+sk, 1) =  find(detection_params.rangeGatesOrdering==temp);
                % max mag from sparsified non-static RDC3
                sparsifiedOutput.activationMaxMagChan(foundCellsFile+sk, 1) = fread(f, 1, 'uint16');
                % RDC2 channelizer exponent that goes along with that RDC3 max mag
                sparsifiedOutput.RDC2DCexp(foundCellsFile+sk, 1) = fread(f, 1, 'int8');
                fread(f, 1, 'uint8');% pad
            end
            fclose(f);
            a=dir(sRDC2binFile);
            
            % check file sizes
            if(numSkewers*Nvrx*4*DCiters~=a.bytes)
                warning('Summary to binary RDC data file size error! Check your dumps');
            end
            
            if numSkewers== (RangeBins*DopplerBins)
                disp([ num2str(numSkewers) ' activations in this file, FULL SET!']);
            else
                disp([ num2str(numSkewers) ' activations in this file'])
            end
            
            % Read in CHANNELIZER data all at once for efficiency
            f = fopen(sRDC2binFile, 'r');
            vrr = fread(f, Nvrx*2*DCiters*numSkewers, 'int16');
            vrr = vrr(1:Nvrx*2*DCiters*numSkewers);
            Vrcomp = complex(vrr(1:2:end), vrr(2:2:end));
            Vrcomp = reshape(Vrcomp,[Nvrx DCiters numSkewers]);
            sparsifiedOutput.RDC2ActivationsChan(foundCellsFile+1:foundCellsFile+numSkewers,:,:) = permute(Vrcomp,[3 1 2]);
            if shiftFilledVrxs       
                fullyfilledVRX = find(sum(sum(sparsifiedOutput.RDC2ActivationsChan,3),1)~=0);
                disp(['*** Shifting Vrxs over by ' num2str(shiftFilledVrxs) ' ***'])
                temp = zeros(size(sparsifiedOutput.RDC2ActivationsChan));
                temp(:,fullyfilledVRX,:,:)= sparsifiedOutput.RDC2ActivationsChan(:,1:length(fullyfilledVRX),:);
                sparsifiedOutput.RDC2ActivationsChan = temp;
            end
            fclose(f);
            
            t1 = sum(real(sparsifiedOutput.RDC2ActivationsChan(:))==-32768  | real(sparsifiedOutput.RDC2ActivationsChan(:))==32767 | imag(sparsifiedOutput.RDC2ActivationsChan(:)==-32768  | imag(sparsifiedOutput.RDC2ActivationsChan(:))==32767));
            disp(['*** Found ' num2str(100*t1/numel(sparsifiedOutput.RDC2ActivationsChan)) '% saturated RDC2 data points'])
                
            
            for irng=1:RangeBins
                for idop =1:DopplerBins
                    foundcell=(sparsifiedOutput.activationRangeBinChan(foundCellsFile+1:end) == irng & sparsifiedOutput.activationDopplerBinChan(foundCellsFile+1:end) == idop);
                    fcells = find(foundcell);
                    if length(fcells)>1
                        error('bad summary data found')
                    else
                        if ~isempty(fcells)
                            doppler_rdc_plot(irng,:,idop,:)= squeeze(sparsifiedOutput.RDC2ActivationsChan(foundCellsFile+fcells, :, :).*2^sparsifiedOutput.RDC2DCexp(foundCellsFile+fcells));
                            numFoundCells = numFoundCells + 1;
                            if exponents(irng)==-1;
                                exponents(irng)=sparsifiedOutput.RDC2DCexp(foundCellsFile+fcells);
                            else
                                assert( exponents(irng)==sparsifiedOutput.RDC2DCexp(foundCellsFile+fcells),'Inconsistent Sparsified Exponents');
                            end
                        else
%                             doppler_rdc_plot(irng,:,idop,:)=0;
                        end
                    end
                end
            end
            
        else
            disp(['*** Cant find Channelizer data file ***']);
        end
        % check that all present data is plotted
        assert(numFoundCells-foundCellsFile==numSkewers,'All input skewers not found based on current input range doppler params');
        foundCellsFile=numFoundCells;
        
    else
        % FFT data
        f = fopen(sRDC2sumFile, 'r');
        if f>0;
            a=dir(sRDC2sumFile); % for file size
            numSkewers = a.bytes/8;
            for sk = 1:numSkewers
                % zero based range and doppler bin data
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
            
            % Read in FFT data all at once for efficiency
            f = fopen(sRDC2binFile, 'r');
            if f>0
                a=dir(sRDC2binFile);
                if(numSkewers*Nvrx*4~=a.bytes)
                    warning('Summary to binary RDC data file size error! Check your dumps');
                end
                vrr = fread(f, Nvrx*2*numSkewers, 'int16');
                vrr = vrr(1:Nvrx*2*numSkewers);
                compVr = complex(vrr(1:2:end), vrr(2:2:end));
                sparsifiedOutput.RDC2ActivationsFFT(foundCellsFile+1:foundCellsFile+numSkewers,:) = reshape(compVr,[Nvrx,numSkewers]).';
                fclose(f);
                
                for irng=1:RangeBins
                    for idop =1:DopplerBins
                        foundcell=(sparsifiedOutput.activationRangeBin(foundCellsFile+1:end) == irng & sparsifiedOutput.activationDopplerBin(foundCellsFile+1:end) == idop);
                        fcells = find(foundcell);
                        if length(fcells)>1
                            error('bad summary data found')
                        else
                            if ~isempty(fcells)
                                doppler_rdc_plot(irng,:,idop)= transpose(sparsifiedOutput.RDC2ActivationsFFT(foundCellsFile+fcells, :).*2^sparsifiedOutput.RDC2exp(foundCellsFile+fcells));
                                numFoundCells = numFoundCells + 1;
                                if exponents(irng)==-1;
                                    exponents(irng)=sparsifiedOutput.RDC2exp(foundCellsFile+fcells);
                                else
                                    assert( exponents(irng)==sparsifiedOutput.RDC2exp(foundCellsFile+fcells),'Inconsistent Sparsified Exponents');
                                end
                            else
                                %  doppler_rdc_plot(irng,:,idop)=0;
                            end
                        end
                    end
                end
                
            else
                disp(['No Channelizer Binary Data file ']);
            end
        else
            disp(['No Summary file ']);
        end
        % check that all present data is plotted
        if f>0
            assert(numFoundCells-foundCellsFile==numSkewers,'All input skewers not found based on current input range doppler params');
            foundCellsFile=numFoundCells;
        end
    end
end
if plotVrx>0
    if DCiters==0
        doppler_rdc_plot_Vrx = abs(squeeze(doppler_rdc_plot(:,plotVrx,:)));
    else
        doppler_rdc_plot_Vrx = squeeze(abs(doppler_rdc_plot(:,plotVrx,:,plotDCiter)));
    end
%     doppler_rdc_plot_Vrx=abs(squeeze(sum(sum(doppler_rdc_plot,4),2)));

    % a=figure;
    % set(a,'Position',[50 400 1200 400])
    % subplot(1,2,1)
    figure;
    histdata=mag2db(doppler_rdc_plot_Vrx(doppler_rdc_plot_Vrx>0));
    if redhist
        [linhist, bincenters]=hist(histdata,50);
        bar(bincenters,linhist)
        redthresh= 10^sum(10.^(0:10) < max (linhist))/1000;
        linhistred = zeros(size(linhist)); linhistred(linhist>0 & linhist<max(linhist)/redthresh) = linhist(linhist>0 & linhist<max(linhist)/redthresh)*redthresh;
        hold; bar(bincenters,linhistred,'r')
        ylabel(['Hits (Red = hits x ' num2str(redthresh) ') '])
    else
        hist(histdata,50);
        ylabel 'Hits'
    end
    title(['Histogram of nonzero RDC2 magnitude (dB) Vrx:' num2str(plotVrx) ])
    xlabel 'Magnitude (dB)'
    
    tooMuchdB = 150;
    deltadB = max(histdata)-min(histdata) ;
    
    if DCiters==0
        minval = max(minval, 2^min(sparsifiedOutput.RDC2exp)); % set minval to min exponent
    else
        minval = max(minval, 2^min(sparsifiedOutput.RDC2DCexp)); % set minval to min exponent
    end
    doppler_rdc_plot_Vrx(abs(doppler_rdc_plot_Vrx)<minval) = minval;
    % subplot(1,2,2)
    figure;
    if DCiters == 0
        rdc2p = mag2db(doppler_rdc_plot_Vrx);
        if dofftshift
            surf(fftshift(rdc2p,2),'FaceColor','interp','EdgeColor','none');
            title(['Raw sRDC2 Doppler FFT data, VRx: ' num2str(plotVrx) ', FFTshifted in doppler'])
        else
            surf(rdc2p,'FaceColor','interp','EdgeColor','none');
            title(['Raw sRDC2 Doppler FFT data, VRx: ' num2str(plotVrx) ', no FFTshift in plotter'])
        end
        xlabel 'Doppler Bin'
        view([ 90 0 ])
        axis tight;
        colormap jet;
        caxis([median(rdc2p(:)) max(rdc2p(:))-3]);
        if tightaxes
            % tighten axes to populated activations
            if(min(sparsifiedOutput.activationDopplerBin)~=max(sparsifiedOutput.activationDopplerBin))
                mapDbin=fftshift(1:DopplerBins);
                xlim([min(mapDbin(sparsifiedOutput.activationDopplerBin)) max(mapDbin(sparsifiedOutput.activationDopplerBin))]);
            end
            if(min(sparsifiedOutput.activationRangeBin)~=max(sparsifiedOutput.activationRangeBin))
                ylim([ min(sparsifiedOutput.activationRangeBin) max(sparsifiedOutput.activationRangeBin)]);
            end
        end
    else
        rdc2p = mag2db(doppler_rdc_plot_Vrx);
        if dofftshift
            surf(fftshift(rdc2p,2),'FaceColor','interp','EdgeColor','none');
            title(['Raw sRDC2 Channelizer data, VRx: ' num2str(plotVrx) ', DCiter ' num2str(plotDCiter) ', FFTshifted in doppler'])
        else
            surf(rdc2p,'FaceColor','interp','EdgeColor','none');
            title(['Raw sRDC2 Channelizer data, VRx: ' num2str(plotVrx) ', DCiter ' num2str(plotDCiter) ', no FFTshift in plotter'])
        end
        xlabel 'DopplerChan Bin'
        view([ 90 0 ])
        axis tight;
        colormap jet;
        caxis([median(rdc2p(:)) max(rdc2p(:))-3]);

        if tightaxes
            % tighten axes to populated activations
            if(min(sparsifiedOutput.activationDopplerBinChan)~=max(sparsifiedOutput.activationDopplerBinChan))
                if dofftshift
                    mapDbin=fftshift(1:DopplerBins);
                else
                    mapDbin=(1:DopplerBins);
                end
                xlim([min(mapDbin(sparsifiedOutput.activationDopplerBinChan)) max(mapDbin(sparsifiedOutput.activationDopplerBinChan))]);
            end
            if(min(sparsifiedOutput.activationRangeBinChan)~=max(sparsifiedOutput.activationRangeBinChan))
                ylim([ min(sparsifiedOutput.activationRangeBinChan) max(sparsifiedOutput.activationRangeBinChan)]);
            end
        end
    end
    
    
    if deltadB > tooMuchdB
        zlim([max(histdata)-deltadB max(histdata)+3]);
    end
    ylabel 'Range Bin'
    zlabel 'Mag (dB)'
end
