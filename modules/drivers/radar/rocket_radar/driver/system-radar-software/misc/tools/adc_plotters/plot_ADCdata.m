% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [rxsig] = plot_ADCdata(ADCbinFile,Lc,Nrx,Npris,sampsPerChip,DataSpray8bit,DataSpray11bit,plotRx,Fs,shiftsamps)
% eg.
% Data Spray UMSK sinusoid 
% ADCout = plot_ADCdata('\\192.168.44.114\uhnder-nas\software\jonathan\output_rdc1_adc.bin',4096,1,360,4,1,1,1e9);
% DAFE format Tone Data
% ADCout = plot_ADCdata('\\192.168.44.114\uhnder-nas\software\jonathan\scan_000001_adc.bin',13568,1,512,1,0,1,1e9);

%*Plot single dataspray pulsed mode capture, 1 ping per "PRI"
% plot_ADCdata('W:\software\ramakris\share\outdoor_exp_aug07\pul_buchanan_bigtgt_postdccal_adc_cap\scan_000000_adc_dataspray.bin',1024,1,5760,8,1,1,1e9);
%*Plot the same data as 1 Lc, scrolling
% plot_ADCdata('W:\software\ramakris\share\outdoor_exp_aug07\pul_buchanan_bigtgt_postdccal_adc_cap\scan_000000_adc_dataspray.bin');
%*Plot it as 960, 6-ping PRIs
% plot_ADCdata('W:\software\ramakris\share\outdoor_exp_aug07\pul_buchanan_bigtgt_postdccal_adc_cap\scan_000000_adc_dataspray.bin',6144,1,960,8,1,1,1e9);

addpath('../jsonlab/');
global result
result.plotADCphysRx = 1; %leave as 1 for single Rx captures

result.plotADCdispSampStart = 1; % Sample to start IQ ADC time plot
result.plotADCdispSampStop = 1000; % Sample to stop IQ ADC time plot

analyzeSelectionOnly = 0; % set this to limit all analysis plots to the start/stop samples above
removeLaneBias = 0;
removePingBias = 0;
pulsedMode=1;
% result.plotADCsurfIQ_RX = 1; % plot ADC IQ surface plot: use this receiver (choose one only) (0=off)
% result.plotADCsurfIQ_RX = 1; % plot ADC IQ surface plot: select enabled transmit codes

% if 1 argument attempt to extract data for 1Rx and plot samples based on file size
% SINGLE BIN FILE ARGUMENT SCROLLS DATA ACROSS IN A SINGLE PRI
if nargin == 1
    scrollon=1;
    disp('Scrolling ADC display mode, using file size to determing num samples for single pulse, single Rx plotting')
    % data size check
    a1 = dir(ADCbinFile);
    assert(~isempty(a1),'no file found');
    DataSpray8bit = 1;
    DataSpray11bit = 0;
    if DataSpray8bit
        numADCsamples  = a1.bytes/2;
    else
        numADCsamples  = a1.bytes/4;
    end
    sampsPerChip = 1;
    Npris = 1;
    Nrx = 1;
    Lc = numADCsamples;
    
else
    scrollon=0;
    % if 2 arguments, first arg is data file, second arg Lc is scaninfo file
    if nargin == 2
        disp('Loading scan params from scaninfo.json');
        if(findstr(ADCbinFile,'dataspray'))
            disp('Setting ADC Read To Data Spray Mode'); pause(1);
            disp('Setting ADC Read To Data Spray Mode'); pause(1);
            disp('Setting ADC Read To Data Spray Mode'); pause(1);
            DataSpray8bit = 1;
            Nrx = 1; % dataspray is only 1rx
        else
            disp('Setting ADC Read To DAFE Data Mode'); pause(1);
            disp('Setting ADC Read To DAFE Data Mode'); pause(1);
            disp('Setting ADC Read To DAFE Data Mode'); pause(1);
            DataSpray8bit = 0;
            Nrx = 1;
            disp('Assuming 1 Receiver');
            pause(1);
        end
        [fp1 fp2 fp3] = fileparts(ADCbinFile);
        scanData=loadjson(Lc);
        Lc = scanData.chips_per_pulse;
        Npris = scanData.num_pulses;
        Fs = scanData.sample_rate;
        warning('hardcoding sampsPerChip to 8, bug in scanData.chip_rate');
        sampsPerChip = 8;
    end
    numADCsamples = Lc*Nrx*Npris*sampsPerChip;
end

PMCW.Lc = Lc; 
PMCW.M = 1;
PMCW.G = 1;
PMCW.K = 1;
PMCW.RxOSFnum = sampsPerChip;
PMCW.RxOSFden = 1;
PMCW.N = Npris;

if ~exist('shiftsamps','var')
    shiftsamps = 0;
end
if ~exist('plotVrx','var')
    plotRx = 1;
end
if ~exist('Fs','var')
    disp('Assuming Fs=0.95625e9');
    Fs = 0.95625e9;
end
if ~exist('DataSpray11bit','var')
    DataSpray11bit = 0;
end


minval = db2mag(10);

% assert(a1.bytes>=2*(numADCsamples),'ADC Bin file size is not correct')

% File load from binary
f = fopen(ADCbinFile, 'r');
if f>0
    
    if DataSpray11bit
        disp('Loading 11(16)-bit Dataspray format RSU ADC data')
        assert(Nrx==1,'todo: update for multi Rx');
        temp = fread(f, 2*numADCsamples, 'int16');
        temp = complex(temp(1:2:end),temp(2:2:end));
        temp=reshape(temp,16,[]);
        temp=temp(reshape([1:8; 9:16],16,[]).',:);
        rxsig=reshape(temp,numel(temp),[]);
    elseif DataSpray8bit
        disp('Loading 8-bit Dataspray format ADC data')
        assert(Nrx==1,'todo: update for multi Rx');
        temp = fread(f, 2*numADCsamples, 'int8');
        temp = complex(temp(1:2:end),temp(2:2:end));
        temp=reshape(temp,16,[]);
        temp=temp(reshape([1:8; 9:16],16,[]).',:);
        rxsig=reshape(temp,numel(temp),[]);
    else
        disp('Loading 8-bit DAFE format ADC data')
        temp = fread(f, 2*numADCsamples, 'int8');
        r = temp(1:2:end);
        i = temp(2:2:end);
        rxsig = transpose(reshape(complex(r, i),Nrx,numADCsamples/Nrx));
    end
    
    rxsig= circshift(rxsig,shiftsamps);
    
    disp(['Before mean I ' num2str(mean(real(rxsig)))]);
    disp(['Before mean Q ' num2str(mean(imag(rxsig)))]);
    
    fclose(f);
    if removeLaneBias
        if pulsedMode
            %some fancy footwork to get the mean Squelched sig
            % this has a lot of hardcoding assuming 8192 samps per ping
            validADC=ones(size(rxsig));
            validADC([1:4105 8192:end],:)=0; %invalid data
            validADC=logical(reshape(validADC,size(rxsig)));
            tempADC = [rxsig; zeros(9-rem(length(validADC),9),1)];
            validADC=[validADC; zeros(9-rem(length(validADC),9),1)];
            maskLanes=reshape(validADC,9,[]);
            tempADC=reshape(tempADC,9,[]);
            for medi = 1:9
                tempLane=squeeze(tempADC(medi,:));
                tempMask=logical(squeeze(maskLanes(medi,:)));
                rxsig_i(medi) = mean(real(tempLane(tempMask))); % was median
                rxsig_q(medi) = mean(imag(tempLane(tempMask))); % was median
                rxsig(medi:9:end) = complex(real(rxsig(medi:9:end))-rxsig_i(medi),imag(rxsig(medi:9:end))-rxsig_q(medi));
            end
            rxsig_i
            rxsig_q
            
        else
            for medi = 1:9
                rxsig_i(medi) = mean(real(rxsig(medi:9:end))); % was median
                rxsig_q(medi) = mean(imag(rxsig(medi:9:end))); % was median
                rxsig(medi:9:end) = complex(real(rxsig(medi:9:end))-rxsig_i(medi),imag(rxsig(medi:9:end))-rxsig_q(medi));
            end
        end
    end
    if removePingBias
       [ rxsig ] = removePerPingBias( rxsig,result.plotADCdispSampStart, result.plotADCdispSampStop, 8192 ) ;
    end
    %     rxsig=conv(rxsig,1/(2*sampsPerChip).*ones(1,(2*sampsPerChip)),'same');
    
    %     rxsig=2^7*exp(pi*j*199188/4194304*[0:4194304-1]).';
    %         rxsig=circshift(rxsig,10);
    %     scratch;
    
    %%%% Plot ADC
    if(plotRx>0)
        ADCmultiplot( double(rxsig), PMCW, Fs, scrollon, analyzeSelectionOnly);
        %         displayISI( double(rxsigFULL), ...
        %         rxsig, ...
        %         result.plotADCsurfIQ_RX, ...
        %         result.plotADCsurfIQ_TX, ...
        %         result.plotADCsurfIQ_TgtBinRanks);
    end
else
    disp(['*** Cant find ADC data file ***']);
end
end
