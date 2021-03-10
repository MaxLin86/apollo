% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [targData_lin, noisemean_dB] = checkHALdata(InputTestFile)
% checkHALdata data validation script
% This script performs a very basic data comparison on the input data with
% some stored off test vectors.

% Scan params
inputData.DopplerBins = 360;
inputData.ChanDoppBins = 120;
inputData.Nvrx = 64;
inputData.Nangle = 128;
inputData.RangeBins = 126;
inputData.DCiters = 3;
inputData.Nrx = 8;

% RDC1,2
inputData.targRbin = [ 28 46 63 118 ];
inputData.targDbin = [ 181 181 181 181 ] ;
inputData.targAbin = [ 65 65 65 65 ];
inputData.noiseRbins = [ 32:42 50:60 67:75];
inputData.noiseDbins = 181;

% Sparsified Data, only moving target
inputData.targRbinChan= inputData.targRbin;
doppChanRatio = (inputData.DopplerBins/inputData.ChanDoppBins);
inputData.targDbinChan = mod(floor(((inputData.targDbin)+(doppChanRatio/2)-1)./doppChanRatio),inputData.ChanDoppBins)+1;
inputData.targRbinFFT= inputData.targRbin;
inputData.targDbinFFT= inputData.targDbin;
inputData.noiseRbinsFFT = 70:110;
inputData.noiseDbinsFFT = 180;
inputData.noiseDbinsChan = ceil(inputData.noiseDbins/(inputData.DopplerBins/inputData.ChanDoppBins));

inputData.allowPlots = 1; % -1 to suppress

inputData.iRDC2padBytes = 0; % will be 8 on Palladium data, 0 on samsim

% Static Slice
inputData.halfWidthDopBins = 5;
inputData.loadStaticSliceExponents=1;
inputData.isComplexRDC3=0;

inputData.targRbinSS = 11;
inputData.targAbinSS = 85;
inputData.targDbinSS = 6;

inputData.noiseRbinsSS = 1:19;

[a1,a2,~]=fileparts(InputTestFile);
mark = max(strfind(a2,'_'));

%% RDC1 Analysis
if ~isempty(strfind(a2,'rdc1'))
    disp('Loading RDC1')
    RDC1binFile = [a1 '/' a2(1:mark) 'rdc1.bin'];
    RDC1expFile = [a1 '/' a2(1:mark) 'rdc1exp.bin'];
    [GenTestData, ~] = plot_RDC1dataSD(RDC1binFile);
    
    % Check Data
    noisemean_dB.RDC1 = mag2db(mean(abs(reshape(GenTestData(inputData.noiseRbins,:,:),1,[]))));
    targData_lin.RDC1 = GenTestData(inputData.targRbin,:,:);
    
    %% RDC2 Analysis
elseif ~isempty(strfind(a2,'sprdc2'))
    
    disp('Loading Sparisfied RDC2FFT')
    [GenTestData, ~] = plot_sRDC2data(InputTestFile,'us',inputData.RangeBins, inputData.Nvrx, inputData.DopplerBins, 0, inputData.allowPlots );
    noiseSamps = GenTestData.RDC2ActivationsFFT(ismember(GenTestData.activationRangeBin,inputData.noiseRbins) & ismember(GenTestData.activationDopplerBin,inputData.noiseDbins),: );
    noisemean_dB = mag2db(mean(abs(noiseSamps(:))));
    for itt=1:length(inputData.targRbinChan)
        targData_lin(itt,:) = GenTestData.RDC2ActivationsFFT(GenTestData.activationRangeBin==inputData.targRbinFFT(itt) & GenTestData.activationDopplerBin==inputData.targDbinFFT(itt),: );
    end
elseif ~isempty(strfind(a2,'rdc2'))
    RDC2binFile = [a1 '/' a2(1:mark) 'rdc2.bin'];
    RDC2expFile = [a1 '/' a2(1:mark) 'rdc2exp.bin'];
    % FFT DATA
    [GenTestData, ~] = plot_iRDC2data(RDC2binFile,RDC2expFile,inputData.RangeBins, inputData.Nvrx, inputData.DopplerBins, 0, inputData.allowPlots, inputData.iRDC2padBytes );
    
    % Check Data
    noisemean_dB = mag2db(mean(abs(reshape(GenTestData(inputData.noiseRbins,:,inputData.noiseDbins),1,[]))));
    for itt=1:length(inputData.targRbin)
        targData_lin(itt,:) = GenTestData(inputData.targRbin(itt),:,inputData.targDbin(itt));
    end
    %% sparse RDC2/3 Analysis
elseif ~isempty(strfind(a2,'spsumch'))
    disp('Loading Sparisfied RDC2chan')
    [GenTestData, ~] = plot_sRDC2data(InputTestFile,'us',inputData.RangeBins, inputData.Nvrx, inputData.ChanDoppBins, inputData.DCiters, inputData.allowPlots);
    noiseSamps = GenTestData.RDC2ActivationsChan(ismember(GenTestData.activationRangeBinChan,inputData.noiseRbins) & ismember(GenTestData.activationDopplerBinChan,inputData.noiseDbinsChan),:,:);
    
    % Check Data
    noisemean_dB = mag2db(mean(abs(noiseSamps(:))));
    for itt=1:length(inputData.targRbinChan)
        targData_lin(itt,:,:) = GenTestData.RDC2ActivationsChan(GenTestData.activationRangeBinChan==inputData.targRbinChan(itt) & GenTestData.activationDopplerBinChan==inputData.targDbinChan(itt),:,:);
    end
elseif ~isempty(strfind(a2,'spsum'))
    disp('Loading Sparisfied Palladium RDC3')
    [GenTestData, ~] = plot_sRDC3data(InputTestFile,'us',inputData.RangeBins, inputData.Nangle, inputData.DopplerBins, inputData.allowPlots );
    
    for itt=1:length(inputData.targRbinChan)
        targData_lin(itt,:) = GenTestData.RDC3Activations(GenTestData.activationRangeBin==inputData.targRbinFFT(itt) & GenTestData.activationDopplerBin==inputData.targDbinFFT(itt),:);
    end
    noiseSamps = GenTestData.RDC3Activations(ismember(GenTestData.activationDopplerBin,inputData.noiseDbinsFFT) & ismember(GenTestData.activationRangeBin,inputData.noiseRbinsFFT),:);
    noisemean_dB = mag2db(mean(abs(noiseSamps(:))));
    
elseif ~isempty(strfind(a2,'stsli'))
    stslbinFile = [a1 '/' a2(1:mark) 'stslice.bin'];
    [GenTestData ] = plot_staticSliceImgSD(stslbinFile);
    for itt=1:length(inputData.targRbinSS)
        targData_lin(itt,:) = GenTestData(inputData.targRbinSS,inputData.targAbinSS,inputData.targDbinSS);
    end
    noisemean_dB = mag2db(mean(reshape(abs(GenTestData(:,:,inputData.noiseRbinsSS)),1,[])));
elseif ~isempty(strfind(a2,'hist'))
    histdatFile = [a1 '/' a2(1:mark) 'histograms.bin'];
    [GenTestData ] = plot_SRAhistograms(histdatFile,1:inputData.RangeBins);
    targData_lin = GenTestData;
    noisemean_dB = 0;
else
    error('shouldnt get here');
end

