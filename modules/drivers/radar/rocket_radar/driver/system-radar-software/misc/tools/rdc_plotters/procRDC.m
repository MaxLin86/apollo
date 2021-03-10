% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2017 Uhnder Inc
% Author:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: plotSimpleRDC1()
%
% Option:
%
% Input:     None
%
% Output:
%
% Calls:
%
%
% Description: Plots coherent RDC1 sum over all Vrx.  Also will perform
%              Correlation and/or Decorrelation on ADC/RDC data from bin file
% CORRELATOR WORKS FOR BPSK ONLY !!!!!!!!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ADCdata, RDC1data, RDC2data, RDC3bsdata] = procRDC(...
    scanInfoFile,cmxin,sw2din,rbinch, dbinch, procMUSIC,useRDC2Ch,useSVDdata, MusNsThr)
global antenna
global sp_params
global cref
global PMCW
global digital_BE
global scenario
global detection_params
global dopplerChannelizer

if strfind(pwd,'support_scripts')
    cd ..
    addPaths
end



useZeroD = 1;

if(~exist('scanInfoFile','var'))
    startpath ='C:\Jonathan\SCC\sessions\20180213172306\init_192.168.95.1';
    startpath = 'C:\Jonathan\SCC\sessions\vrxCheck3\init_192.168.95.1';
    startpath = 'C:\repos\system_analysis\TargetGenVrx\TC4';
    startpath = 'C:\Jonathan\SCC\sessions\20180410122303\vp1aridgetest';
    startpath = 'Z:\Sabine_Data_Shoots\20180420-FreezeTempTestRDC1';
    startpath = 'C:\uhnderOLD\TESTDATA\RDC1BRSHORT';
%     % for 1D RDC1 test, outdoor test rbin 49
%     scanInfoFile = 'Z:\Sabine_Data_Shoots\20180125_3Car_and_Debris_data\Threecar_in_threelane\0125_threecar_dist30\0125_3car30_rdc1\scan_000015_info.json';
%     cmxin= 'Z:\Sabine_Data_Shoots\20180125_3Car_and_Debris_data\cal_az_only';
%     sw2din=0;
%     rbinch = 49;
%     dbinch = 57;
%     procMUSIC = 1;
%     useRDC2Ch = 0;
%     useSVDdata = 0;
%     MusNsThr = -3;

% for 1D MUSIC test, chamber range bins 12 and 13
% scanInfoFile= 'Z:\Sabine_Data_Shoots\20180120_AudiUseCases\chamber_test\chamber_dist7_ant1d\vp4\detection7p20\scan_000030_info.json';
scanInfoFile= [ startpath '\scan_000250_info.json'];
% cmx not used
% cmxin= 'C:\Jonathan\SCC\sessions\20180213172306\init_192.168.95.1\cal_az_only';

% % for 2D High Res RDC1 test
scanInfoFile = 'C:\TESTDATA\20190718_resolutionAzElTests\test2Dtradel_in_newcamtilt\scan_001371_info.json';
sp_params.sabineCmx = 'C:\TESTDATA\20190711_shaolin_marketlaunch_resolution_data\diag2d'; 
sp_params.switch2DMIMO = 1;
% 
% scanInfoFile = 'C:\TESTDATA\20190703_RDC1and2zd_collect\rdcc1d\scan_000037_info.json';
% sp_params.sabineCmx='C:\TESTDATA\20190703_RDC1and2zd_collect\ang1d'; 
% sp_params.switch2DMIMO = 0;


else
    sp_params.sabineCmx=cmxin;
sp_params.switch2DMIMO = sw2din;

end


% sw2din=1;
rbinch = 1;
dbinch = 420;    
procMUSIC = 0;
useSVDdata = 0;
MusNsThr = -3;
maxRbinplot=126;
useRDC2Ch = 0;

%USER INPUT

sp_params.AdaptTh = 1;
sp_params.BeamformApplySVA = 1;
sp_params.BeamformUseBoxcar = 1;
sp_params.desiredSidelobeLevel = 30;


ADB = 2; % 1 = ADB1.6, 2= ADB2.0 
plotRDC3 = 1;
maxmaxrbinsearch=999;
MUSIC.rbinch = rbinch;
MUSIC.dbinch = dbinch;
haxDchanOutputs=120; %for RDC1 to doppler channelizer use
% MUSIC.dbinch=PMCW.DoppChanOutputs/2+1; %assume zero doppler
MUSIC.rdmatch = [];
MUSIC.allSmoothBuf = 30;
MUSIC.anglesfine = (-60:.1:60);
MUSIC.allMusThr= MusNsThr;
MUSIC.anglesfine = -30:.05:30;
MUSIC.matchProcRDC3 = 0;
%         MUSIC.allMusThr = -9:2:-3;
%         MUSIC.allSmoothBuf = 15:25;
activeHWTX=[ 2 4 6 7 8 10 11 ];
% Tx Rx channels are fed in as arguments if not theyre hardcoded here
% setup params
plotVrx=1;
overrideN = [];
allowPlots = 1;
writeJenkinsRDC1CSV=0;
dofftshift = 0;
showAllVRx=1;
spatialOrdering = 0; % set to zero for HW Tx/Rx Major (not Spatial) VRx
plotRDC2Vrx = [ 1 ]; % plot the RDC2 of 1-based, HW-Major VRx;
plotrxMajVrx = 1:64;
plotdBFS = 0;
plotRbinMinMax = [];

sp_params.decorrelationIters = 0;
sp_params.extraPRI = 1;
%     sp_params.pulsedMode = 0;
cref.prbs31 = 1;
cref.decorrelator = 0;
sp_params.Enable_IQ_Correction = 0;
sp_params.Enable_Doppler_Comp = 0;
sp_params.Enable_Doppler_Comp=0;
sp_params.fft_correlator_blocksize = 256;
sp_params.decorrelatorThresholdDb=65;
sp_params.decorrSwamperRbin = 0;
sp_params.dopplerFFTshift = 1;
sp_params.FFTstartPulseN = 1;

%     B = sp_params.fft_correlator_blocksize / 2;
digital_BE.dopplerOutputBitWidth = 16;
digital_BE.roughAoAOutputBitWidth = 16;
digital_BE.adcOutputBitDepth = 8;
digital_BE.accumulatorInternalBitDepth = 22;
digital_BE.correlatorOutputBitDepth = 16;
digital_BE.adcformat = 'DataSpray8bit'; %ADC data format, can be DataSpray8bit or 8bitVrxInterleaved
%digital_BE.adcformat = '8bitVrxInterleaved';
scenario.Fc = 7.6477667857e10;
scenario.c = 299792458;
scenario.lambda_c = scenario.c/scenario.Fc; % operational carrier wavelength

sp_params.BeamformUseAllVRx = 1;

sp_params.MusicCase = 1;
sp_params.ForwardBackwardAveraging = 1;
sp_params.BeamformXCalMat=[];
sp_params.BeamformPhaseError=[];
sp_params.BeamformGainError = [];
sp_params.noNFcorr = 0;
sp_params.BeamformOptimizedCalc =1;

sp_params.tempSkipCal = 0;

sp_params.BeamformSVA_NyqFactor = 5;
sp_params.AdaptTh_ScanOnlyAngle = 1;          % Margin in terms of estimated noise std
sp_params.AdaptTh_stdvMargin = 4;
sp_params.AdaptTh_DetectMargindB = 3;
sp_params.AdaptTh_Max_dynamicrg = 20;

ADCdata = []; % ADC outpus is FULL RATE
RDC1data = []; % RDC1 output is AFTER decorr (if enabled), Ymajor!
RDC2data = []; % RDC2 output is FFT only, Ymajor!
RDC3bsdata = [];

if ~exist('loadjson','file')
    addpath('../jsonlab/');
end

% file parts
[fp1, fp2, fp3] = fileparts(scanInfoFile);
RDC1file = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'rdc1.bin'];
chanfile = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'spsumchu.bin'];
RDC3file = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'spsumu.bin'];
SVDfile = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'usmxup.bin']; 
MUSfile = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'music.bin']; 
zeroDfile = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'rdc2zdop.bin'];

% RDC1expFile = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'RDC1exp.bin'];

% Load Scan Info Json
[PMCW, ~, detection_params, dBFScorr_lin, swExponent, ~] = processSabineScanInfo(scanInfoFile);

sp_params.pulsedMode = PMCW.pulsedMode;
chipsPerPing = PMCW.txPumpWidth*2;

if(~isempty(overrideN) && overrideN>0)
    warning('Overwriting N pulses with override value')
    PMCW.N =overrideN;
end

% Bring in some hardcoded(HC) antenna params that are not in scaninfo
antenna = loadAntennaData(sp_params.switch2DMIMO,ADB);
antenna.scan_alpha_deg = rad2deg(detection_params.angleGatesMid(:,1)).';
antenna.scan_theta_deg = rad2deg(detection_params.angleGatesMid(:,2)).';

[ tempAlpha, tempTheta] = calc_angles(sp_params);
antenna.scan_alpha_deg = rad2deg(unique(tempAlpha)).';
antenna.scan_theta_deg = rad2deg(unique(tempTheta)).';
% tempAlpha = unique(round(antenna.scan_alpha_deg*1000000)/1000000);

% antenna.scan_alpha_deg = interp1(1:128, antenna.scan_alpha_deg, 1:.4:128);
% antenna.scan_theta_deg = interp1(1:128, antenna.scan_theta_deg, 1:.4:128);

if sp_params.switch2DMIMO
    antenna.fullyFilled = 64;
    antenna.filledIndices = 1:64;
else
%     antenna.fullyFilled = 25 ;
% antenna.filledIndices = antenna.filledIndices(6:30);
end
% for tvrx = 1 : antenna.NVrx
%     antenna.mapToVrx2(tvrx) = find(antenna.map == tvrx);
% end
  
% HAX% HAX% HAX% HAX% HAX% HAX
% oldantenna=antenna;
% clear antenna
% load('C:\repos\system_analysis\TargetGenVrx\TGantenna.mat')
% HAX% HAX% HAX% HAX% HAX% HAX% HAX% HAX


% antenna.Ntx = HCantenna.Ntx;
% antenna.Nrx = HCantenna.Nrx;
% antenna.realhwRxMap = HCantenna.realhwRxMap;
% antenna.realhwTxMap = HCantenna.realhwTxMap;
% antenna.filledIndices = HCantenna.filledIndices;
% antenna.map(antenna.mapToVrx)=1:64;

% Load Cmatrix
if(~isempty(sp_params.sabineCmx))
    C= loadSabineCmx(sp_params.sabineCmx);
else
    C=eye(antenna.NVrx);
end

antenna.CalculatedCalMatrix = C;
sp_params.pulsedModeNumBlocks = PMCW.numPumps;
sp_params.pulsedModeCorrPump = [1:PMCW.numPumps];

%CONVERT TO DEGREES HERE
detection_params.angleGatesMid = rad2deg(detection_params.angleGatesMid);

if plotdBFS==2
    dBFScorr_lin.RDC1(:) = dBFScorr_lin.RDC1max;
    dBFScorr_lin.RDC2(:) = dBFScorr_lin.RDC2max;
    dBFScorr_lin.RDC3(:) = dBFScorr_lin.RDC3max;
end

samples = PMCW.Lc * PMCW.N * PMCW.RxOSFnum/PMCW.RxOSFden; % Lc  * OSF *N
Fs = PMCW.Fs;

% Code Gen
if sp_params.decorrelationIters
    [PMCWcode] = genCodesSimple(PMCW.Lc, PMCW.N, antenna.Ntx);
    PMCW.code_tx = PMCWcode.code_tx;
    PMCW.B = PMCWcode.B;
    PMCW.codetype = PMCWcode.codetype;
    PMCW.Ntmrx = PMCWcode.Ntmrx;
    %     PMCW.code_tx=circshift(PMCW.code_tx,4,1);
    fcodes = repmat(complex(double(PMCW.code_tx), 0),1,1,antenna.Nrx);
end

% PMCW Struct
PMCW.skipChips=0; % assume no skip chips, fixme
PMCW.chipsPerBin = ones(1, PMCW.R);
PMCW.numCorrelators=PMCW.R;
PMCW.Tc=(PMCW.RxOSFnum/PMCW.RxOSFden)/Fs;
PMCW.M =1;
PMCW.G = 1;
PMCW.K =1;

% Calc dBFS offsets
if plotdBFS
    % SW exponents for RDC2 and RDC3 are zeroed bc the FFT and AoA are done in samsim
    if exist('scanData','var')
        scanData.rdc2_software_exponent = 0;
        scanData.rdc3_software_exponent = 0;
        [dBFScorr1, dBFScorr2, dBFScorr3] = dBFScorrection(sp_params.pulsedMode,PMCW, antenna, digital_BE, scanData);
    else
        [dBFScorr1, dBFScorr2, dBFScorr3] = dBFScorrection(sp_params.pulsedMode,PMCW, antenna, digital_BE);
    end
else
    rtemp=sum(detection_params.rangeGatesMid>=0);
    dBFScorr1 = zeros(rtemp,1);
    dBFScorr2 = zeros(rtemp,1);
    dBFScorr3 = zeros(rtemp,1);
end

if useRDC2Ch
    %skip cal on chan data, already done
        sp_params.tempSkipCal = 1; %MUSIC%MUSIC%MUSIC%MUSIC skip cal mx if 1
    [sparsifiedOutputDC, doppler_rdc_plot, exponents] =plot_sRDC2data(chanfile,'u');
    MUSIC.rbinch = sparsifiedOutputDC.activationRangeBinChan;
    currAct=find(sparsifiedOutputDC.activationRangeBinChan==MUSIC.rbinch & sparsifiedOutputDC.activationDopplerBinChan==MUSIC.dbinch)

    MUSIC.rdmatch =[];
    if MUSIC.matchProcRDC3
        [sparsifiedOutput,~] = plot_sRDC3data(RDC3file,'us');
        % Channelizer activation to run music on
        DoppChanRatio = (size(sparsifiedOutputDC.RDC2ActivationsChan,3)+3)/2;
        MUSIC.rdmatch = find(sparsifiedOutputDC.activationRangeBinChan(currAct)==sparsifiedOutput.activationRangeBin & ...
            sparsifiedOutputDC.activationDopplerBinChan(currAct)==ceil(sparsifiedOutput.activationDopplerBin/DoppChanRatio));
        [~, rdmaxmatch] = max(sparsifiedOutput.activationMaxMag(MUSIC.rdmatch));
        
        % Max matching RDC3 skewer
        MUSIC.rdmatch= MUSIC.rdmatch(rdmaxmatch);
    end
    MUSIC.inputSkewer = sparsifiedOutputDC.RDC2ActivationsChan(currAct, :, :);
    if useSVDdata
        SVDout = loadUSmx(SVDfile);
        runMusicIters(MUSIC, sp_params, detection_params, SVDout.U, SVDout.S)
    else
        runMusicIters(MUSIC, sp_params, detection_params);
    end
    hold on, plot_musicSpectrum(MUSfile);
else
%% Begin Processing
removeDCbias = 0;
% RDC1 data binary is in spatial order
if ~useZeroD
RDC1data=plot_RDC1dataSD(RDC1file,0,0);
if ~isempty(plotRbinMinMax)
    ylim(plotRbinMinMax);
end

%RDC1 Decorr
if sp_params.decorrelationIters > 0
    RDC1data = RDC1data(:,antenna.mapToVrx,:); %remap to rxmajor
    fcodes = repmat(complex(double(PMCW.code_tx), 0),1,1,antenna.Nrx);
    
    RDC1data = decorrelate(RDC1data, fcodes, sp_params.decorrelationIters, ...
        sp_params.decorrelatorThresholdDb, PMCW);
    RDC1data=RDC1data(:,antenna.map,:); %back to spatial
end

% Write RDC2 zero dopp in dB to Jenkins CSV file
if writeJenkinsRDC1CSV
    temp = abs(squeeze(sum(RDC1data,3)));
    temp(temp==0)=1;
    csvwrite('RDC2zeroDopp.csv',mag2db(temp));
end

% FFT doppler transformation
window = nuttallwin(PMCW.N);
[RDC2data, fftExponent] = calcFFT(RDC1data, window);
RDC2data = RDC2data.*repmat(2.^fftExponent,[1 1 PMCW.N]);

if procMUSIC
    % hack RDC1 doppler channelizer count
    PMCW.DoppChanOutputs=haxDchanOutputs;
    %% Doppler Channelizer settings
    dopplerChannelizer.numTapsPerBranch = 2; % this is the overlap factor, 2==50%
    dopplerChannelizer.numBranches = PMCW.DoppChanOutputs;
    dopplerChannelizer.numSamplesPerIteration = dopplerChannelizer.numBranches / 2; % number of pulses input each iteration, always numBranches/2
    dopplerChannelizer.PFregisters = zeros(dopplerChannelizer.numBranches,dopplerChannelizer.numTapsPerBranch); % size of regs and prototype func
    dopplerChannelizer.resetRegisterEveryScan = true;
    dopplerChannelizer.numIterationsPerScan = PMCW.N / dopplerChannelizer.numSamplesPerIteration; % no *PMCW.Ntmrx since 2 elev RDCs are processed
    dopplerChannelizer.firstValidIteration = dopplerChannelizer.numTapsPerBranch*2;
    dopplerChannelizer.validIterations = dopplerChannelizer.numIterationsPerScan - dopplerChannelizer.firstValidIteration +1;
    
    numTaps = numel(dopplerChannelizer.PFregisters)-1;
    prototypeFilterTaps = getAmplitudeTaper('remez', numTaps, []);
    [doppler_rdc, dopplerTransExp] = channelizeDoppler(RDC1data, prototypeFilterTaps);
    MUSIC.inputSkewer(1,:,:) = squeeze(doppler_rdc(MUSIC.rbinch,:,MUSIC.dbinch,:));
    %         MUSIC.allMusThr = -9:2:-3;
    %         MUSIC.allSmoothBuf = 15:25;
    %MUSIC skip cal mx
    sp_params.tempSkipCal = 1; %MUSIC%MUSIC%MUSIC
    
    for itc= 1:size(MUSIC.inputSkewer,3)
        MUSIC.inputSkewer(1,:,itc) = squeeze(MUSIC.inputSkewer(1,:,itc))* C;
    end
    runMusicIters(MUSIC, sp_params, detection_params)
    %     xlim([-10 10])
end

if allowPlots
    % Plot RDC1
    if spatialOrdering
        
        HWtxmap = repelem(antenna.realhwTxMap, 8);
        HWrxmap = repmat(antenna.realhwRxMap,[1 8]);
        if showAllVRx
            plotmask=ismember(HWtxmap,antenna.realhwTxMap) & ismember(HWrxmap,antenna.realhwRxMap);
        else
            plotmask=zeros(1,64);
            plotmask(plotrxMajVrx)=1;
        end
        plotRDC1=RDC1data;
        
        %masking off hidden antennas
        plotRDC1(:,~plotmask(antenna.map),:)=0;
        
        % plot Tx colored
        lin=lines;
        temp=[lin(1:7,:); .5 .5 .5];
        cmap=[repelem(temp(:,1),8), repelem(temp(:,2),8), repelem(temp(:,3),8)];
        %     if numel(activeHWRX)==1
        %         figure(activeHWRX+1);
        %     else
        %         figure(1);
        %     end
        figure;
        hplot=ribbon(mag2db(abs(squeeze(sum(plotRDC1,3))))-repmat(dBFScorr2,1,size(plotRDC1,2)));
        
        if ~isempty(plotRbinMinMax)
            ylim(plotRbinMinMax);
        end
        
        
        if sp_params.decorrelationIters
            if length(activeHWTX)==0
                set( hplot, 'EdgeColor', 'k' );
            elseif length(activeHWTX) == 1
                set( hplot, 'EdgeColor', 'm' );
            else
                set( hplot, 'EdgeColor', 'c' );
            end
        else
            if length(activeHWTX)==0
                set( hplot, 'EdgeColor', 'k' );
            elseif length(activeHWTX) == 1
                set( hplot, 'EdgeColor', 'r' );
            else
                set( hplot, 'EdgeColor', 'k' );
            end
        end
        
        
        %     if sp_params.decorrelationIters
        %         set( hplot, 'EdgeColor', 'k' );
        %     else
        %         set( hplot, 'EdgeColor', 'r' );
        %     end
        %         set( hplot, 'LineStyle', 'none' );
        colormap(cmap((antenna.map),:));
        xlabel 'Spatial VRx Ordering', ylabel Rangebin, zlabel Magnitude(dB);
        title 'RDC1 Coherent Sum, Spatial Order';
        if removeDCbias
            title(['Simulated-RDC1 Coh Sum, removed mu(I,Q)=(' num2str(round(meanI(activeHWRX+1),2)) ',' num2str(round(meanQ(activeHWRX+1),2)) ') Spatial Order, Tx:' num2str(activeHWTX) ', Rx:' num2str(activeHWRX)]);
        else
            title 'Simulated-RDC1 Coherent Sum, Spatial Order';
        end
    else
        %masking off hidden antennas
        RDC1data(:, antenna.map, :) = RDC1data; %remap to rxmajor
        plotRDC1=RDC1data;
        HWtxmap = repelem(antenna.realhwTxMap, 8);
        HWrxmap = repmat(antenna.realhwRxMap,[1 8]);
        if showAllVRx
            plotmask=ismember(HWtxmap,antenna.realhwTxMap) & ismember(HWrxmap,antenna.realhwRxMap);
        else
            plotmask=zeros(1,64);
            plotmask(plotrxMajVrx)=1;
        end
        plotRDC1(:,~plotmask,:)=0;
        
        % plot Tx colored
        lin=lines;
        temp=[lin(1:7,:); .5 .5 .5];
        cmap=[repelem(temp(:,1),8), repelem(temp(:,2),8), repelem(temp(:,3),8)];
        %     if numel(activeHWRX)==1
        %         figure(activeHWRX+1);
        %     else
        %         figure(1);
        %     end
        
        figure;
        hplot = ribbon(mag2db(abs(squeeze(sum(plotRDC1,3))))-repmat(dBFScorr2,1,size(plotRDC1,2)));
        if ~isempty(plotRbinMinMax)
            ylim(plotRbinMinMax);
        end
    end
    
    if sp_params.decorrelationIters
        if length(activeHWTX)==0
            set( hplot, 'EdgeColor', 'k' );
        elseif length(activeHWTX) == 1
            set( hplot, 'EdgeColor', 'm' );
        else
            set( hplot, 'EdgeColor', 'c' );
        end
    else
        if length(activeHWTX)==0
            set( hplot, 'EdgeColor', 'k' );
        elseif length(activeHWTX) == 1
            set( hplot, 'EdgeColor', 'r' );
        else
            set( hplot, 'EdgeColor', 'k' );
        end
    end
    %     if sp_params.decorrelationIters
    %         set( hplot, 'EdgeColor', 'k' );
    %     else
    %         set( hplot, 'EdgeColor', 'r' );
    %     end
    %         set( hplot, 'LineStyle', 'none' );
    grid minor;
    colormap(cmap);
    xlabel 'HW Tx/Rx Major (not Spatial) VRx', ylabel Rangebin, zlabel Magnitude(dB);
    if removeDCbias
        ADCdata_i
        ADCdata_q
        title(['Simulated-RDC1 Coh Sum, mu(I,Q)=(' num2str(round(meanI(activeHWRX+1),2)) ',' num2str(round(meanQ(activeHWRX+1),2)) ') RxMajor Order, Tx:' num2str(activeHWTX) ', Rx:' num2str(activeHWRX)]);
    else
        title 'Simulated-RDC1 Coherent Sum, RxMajor Order, Rx Tx';
    end
    
    
    % savefig([fp1 '/' fp2 'ribbons.fig']);
    
    if plotRDC2Vrx
        plotRDC2data(:, antenna.map, :) = RDC2data; %remap to rxmajor
        if allowPlots
            for itp = 1:length(plotRDC2Vrx)
                figure,
                if dofftshift
                    toplot=mag2db(abs(fftshift(squeeze(plotRDC2data(:,plotRDC2Vrx(itp),:)),2)))-repmat(dBFScorr2,1,size(plotRDC2data,3));
                    surf(toplot,'FaceColor','interp','EdgeColor','none');
                    title(['Simulated-RDC2 Doppler FFT data, RxMajVRx: ' num2str(plotRDC2Vrx(itp)) ', FFTshifted in doppler'])
                else
                    toplot=mag2db(abs(squeeze(plotRDC2data(:,plotRDC2Vrx(itp),:))))-repmat(dBFScorr2,1,size(plotRDC2data,3));
                    surf(toplot,'FaceColor','interp','EdgeColor','none');
                    title(['Simulated-RDC2 Doppler FFT data, RxMajVRx: ' num2str(plotRDC2Vrx(itp)) ', no FFTshift in plotter'])
                end
                xlabel 'Doppler Bin'
                ylabel 'Range Bin'
                if plotdBFS
                    zlabel('Mag dBFS');
                else
                    zlabel('Mag dB');
                end
                %         zlim(mag2db(abs(squeeze(plotRDC2data(:,plotRDC2Vrx(itp),:))))
                axis tight;
                colormap jet;
                caxis([median(toplot(:))-20 max(toplot(:))-3]);
                if ~isempty(plotRbinMinMax)
                    ylim(plotRbinMinMax);
                end
                zlim([median(toplot(:))-20 max(toplot(:))+3]);
            end
        end
        %     savefig([fp1 '/' fp2 'RDC2.fig']);
    end
end
else
    
end

if plotRDC3
    indd = length(detection_params.dopplerGatesMidFFT)/2+1-1:length(detection_params.dopplerGatesMidFFT)/2+1+1;
    detection_params.dopplerGatesMidFFT = detection_params.dopplerGatesMidFFT(indd);
    if useZeroD
        clear RDC2data;
        RDC2data = plot_rdc2_zerod(zeroDfile,128,0);
        RDC2data = permute(repmat(RDC2data,[ 1 1 3]),[2 1 3]);
        warning('Cover up for ZeroDbug, bad rangebins are at the end');
        RDC2data = RDC2data(1:sum(detection_params.rbininfo.rangeorder>0),:,:);
    else
        RDC2data = RDC2data(:,:,indd);
    end
    %     VrxPhase = angle(RDC2data(targRbin,:,targDbin));
    if(1)    
        
%         vrxvec=BScal.vrx;

        %     % Create C based on range to targ
        %     range = 7.2;%detection_params.rangeGatesMid(result.dispVRxPhaseForRangeBin);
        %     [ A, ~, tau, upsilon, rho ] = calcSteeringVectorULA(0, pi/2, range);
        %     phi = tau' * upsilon * rho(1);
        %     B = exp(1j .* phi);
        %     W = B .* A;
        %     phaseC = (angle(vrxvec) - angle(W));
        %     deltamag = mag2db(abs(vrxvec)/max(abs(vrxvec)));
        %     magC = db2mag(max(deltamag)) ./ db2mag(deltamag);
        %     C = diag(magC .* exp(-1j * phaseC));
        %create STV for all rbins
%         detection_params.rangeGatesMid = 0:1.2:1.2*(PMCW.R-1); %FIXME ADD REAL RANGE RES

        
        % ONLY MAX MAG RANGE BIN WILL BE PROCESSED FOR ROUGH AOA
        % find max range bin over all vrx, print out results for use
        minmaxrbinsearch=13;
        maxmaxrbinsearch=min(maxmaxrbinsearch,size(RDC2data,1));
        for itvrx = 1: size(RDC2data,2)
            [~,rbinmaxvrx(itvrx)] = max(max(squeeze(RDC2data(minmaxrbinsearch:maxmaxrbinsearch,itvrx,:)),[],2));
            
        end
        rbinmaxvrx = rbinmaxvrx + minmaxrbinsearch - 1;
        userbinmax = mode(rbinmaxvrx);
%         userbinmax=1:maxRbinplot;
        templist=unique(rbinmaxvrx);
        for ittt= 1:length(templist)
            disp(['Range bin ' num2str(templist(ittt)) ' has ' num2str(sum(rbinmaxvrx==templist(ittt))) ' max mags across the Vrxs'])
        end
        
        
        [ A, ~, tau, upsilon, rho ] = calcSteeringVectorULAsabine( ...
            deg2rad(antenna.scan_alpha_deg), ...
            deg2rad(antenna.scan_theta_deg), ...
            detection_params.rangeGatesMid(userbinmax));
        if 0 %this version uses all range bins
        if 1 % skep NF calc
            roughAoAstv = repmat((A*C),1,1,size(RDC2data,1));
        else
            % incl nearfield, LOTS OF TIME
            roughAoAstv = zeros([size(A) size(RDC2data,1)]);
            for rr = 1 : length(size(RDC2data,1))
                rr
                phi = tau' * upsilon * rho(rr);
                B = exp(1j .* phi);         % TODO: implement Taylor series approximation for e^i
                %         B = (1 - ((phi^2)/2)) + 1j * phi
                if sp_params.noNFcorr
                    B(:)=1; % turn off NF by mult by 1
                end
                if sp_params.BeamformOptimizedCalc
                    roughAoAstv(:, :, rr) = B .* (A*C);        % what the hardware will do
                else
                    roughAoAstv(:, :, rr) = ( B .* A ) * C; % what we should really do
                end
            end
        end
        [RDC3bsdata, roughaoaExponent] = rough_aoa(RDC2data(userbinmax,:,:), roughAoAstv(:,:,userbinmax), 1);
        end
        roughAoAstv = (A*C);
        [RDC3bsdata, roughaoaExponent] = rough_aoa(RDC2data(userbinmax,:,:), roughAoAstv, 1);
        RDC3bsdata = RDC3bsdata .* repmat(2.^roughaoaExponent,1,size(RDC3bsdata,2),size(RDC3bsdata,3),size(RDC3bsdata,4));
        
        if sp_params.BeamformApplySVA || sp_params.AdaptTh
            for rr=1: size(RDC3bsdata,1)
                tempr = squeeze(RDC3bsdata(rr,:,:,:));
                cheapNF(rr)=median(abs(tempr(:)))/db2mag(5);
            end
        end
        if sp_params.BeamformApplySVA
            if sp_params.switch2DMIMO
                [RDC3bsdataSVA] = SVA_on_2Drough_aoa(RDC3bsdata, 0);
            else
                [RDC3bsdataSVA] = SVA_on_rough_aoa(RDC3bsdata, 0);
            end
            for rr=1: size(RDC3bsdata,1)
                mask = squeeze(RDC3bsdataSVA(rr,:,:,:) < cheapNF(rr)) | isnan(squeeze(RDC3bsdataSVA(rr,:,:,:)));
                RDC3bsdataSVA(rr,mask) = cheapNF(rr);
            end
        else
            RDC3bsdataSVA = RDC3bsdata;
        end
        % Adaptative Robust Thresholding
        if sp_params.AdaptTh
            [RDC3bsdataAT,  ~ ]= Adapt_Simple_Th( RDC3bsdataSVA, mag2db(cheapNF), sp_params.AdaptTh_stdvMargin, sp_params.AdaptTh_DetectMargindB, sp_params.AdaptTh_Max_dynamicrg );
        else
            RDC3bsdataAT = RDC3bsdataSVA;
        end
        
        
        %         temp1=max(abs(RDC3bsdata),[],2);
        %         temp2=max(abs(temp1),[],3);
        %         figure, surf(mag2db(abs(squeeze(temp2))))
        %         figure, surf(HCantenna.scan_theta_deg.',HCantenna.scan_alpha_deg.',mag2db(abs(squeeze(RDC3bsdata(2,:,:,505)))))
        %         colorbar
        %         colormap jet
        %         axis tight
        %         view(90,90)
        if sp_params.switch2DMIMO
            
            figure;
            curpos=get(gcf,'Position');
            set(gcf,'Units','pixels','outerposition',[ 0 0 1024 768]);
            set(gcf,'Color',[1 1 1]);
            surf((antenna.scan_theta_deg.'),(antenna.scan_alpha_deg.'),(mag2db(abs(squeeze(RDC3bsdataSVA(1,:,:,2))))),'EdgeColor','none','FaceColor','interp')
            if sp_params.BeamformApplySVA
                title(['Magnitude (dB) of Target in Range bin ' num2str(userbinmax) ', Rough AoA']);
            else
                title(['Magnitude (dB) of Target in Range bin ' num2str(userbinmax) ', Rough AoA with ' num2str(sp_params.desiredSidelobeLevel) ' dB chebwin']);
            end
            xlabel('Azimuth (degrees)')
            ylabel('Elevation (degrees)')
            colorbar
            colormap jet
            axis tight
            view(90,90);
            
            
        else
            figure, 
            if sp_params.switch2DMIMO 
                surf(squeeze(mag2db(max(abs(RDC3bsdata),[],2))));
            else
                plot(antenna.scan_alpha_deg,squeeze(mag2db(max(abs(RDC3bsdata),[],3))));
            end
            xlabel('Angle(deg)');
            ylabel('Mag (dB)');
            title('RDC3 Range/Angle, Dop')
        end
        
    else
        % old boresight only
        RDC2dataFF=RDC2data(:,antenna.filledIndices,:);
        VrxPhase = angle(RDC2dataFF(targRbin,:,targDbin));
        VrxMagAdj=repmat(max(abs(RDC2dataFF(targRbin,:,targDbin)))./(abs(RDC2dataFF(targRbin,:,targDbin))),size(RDC2dataFF,1), 1, size(RDC2dataFF,3));
        if allowPlots
            RDC3bsdata = squeeze(sum(VrxMagAdj.* RDC2dataFF.*repmat(exp(-1j*VrxPhase),[size(RDC2dataFF,1) 1 size(RDC2dataFF,3)]),2));
            toplot=mag2db(abs(double(RDC3bsdata)))-repmat(dBFScorr3,1,size(RDC3bsdata,2));
            figure, surf(toplot,'FaceColor','interp','EdgeColor','none');
            %     figure, plot(mag2db(abs(squeeze(double(RDC3bsdata(:,targDbin)))))-dBFScorr3)
            xlabel('Doppler Bin');
            ylabel('Range Bin');
            if plotdBFS
                zlabel('Mag dBFS');
            else
                zlabel('Mag dB');
            end
            title('Simulated-RDC3 Zero Doppler Boresight Phase Aligned')
            axis tight;
            colormap jet;
            caxis([median(toplot(:))-20 max(toplot(:))-3]);
            if ~isempty(plotRbinMinMax)
                ylim(plotRbinMinMax);
            end
            zlim([median(toplot(:))-20 max(toplot(:))+3]);
        end
    end

    %         phaseC = deg2rad(sp_params.BeamformPhaseError);
    %         if isempty(sp_params.BeamformGainError)
    %             magC = ones(NVrx);
    %         else
    %             magC = db2mag(max(sp_params.BeamformGainError)) ./ db2mag(sp_params.BeamformGainError);
    %         end
    %         C = diag(magC .* exp(-1j * phaseC));
    %     disp(['Range Bin: ',num2str(r)]);
    %     disp(['Phase per VRx: [ ',sprintf('%.2f, ',rad2deg(phase - angle(W))),' ]']);
    %     RDC3bsdata = squeeze(sum(RDC2data./repmat(BScal.vrx,[size(RDC2data,1) 1 size(RDC2data,3)]),2));
    
end
end
if ~isdeployed
    cd support_scripts
end
end
function [aa_az_angles, aa_el_angles] = calc_angles(sp_params)

if (sp_params.switch2DMIMO)
    Ad=1.5; N=8; aa_Nyq_az=sp_params.BeamformSVA_NyqFactor; aa_wrap_az=1;
    aa_az_angles = asin(-1/(2*Ad) + (0:(N*aa_Nyq_az-1))/(Ad*N*aa_Nyq_az)).';
    Ad=2.0; N=8; aa_Nyq_el=sp_params.BeamformSVA_NyqFactor; aa_wrap_el=1;
    aa_el_angles = asin(-1/(2*Ad) + (0:(N*aa_Nyq_el-1))/(Ad*N*aa_Nyq_el)).';
    aa_n_az = length(aa_az_angles);
    aa_n_el = length(aa_el_angles);
    aa_az_angles = repmat(aa_az_angles,aa_n_el,1);
    aa_el_angles = reshape(repmat(aa_el_angles,1,aa_n_az).',[],1);
else
    Ad=0.5; N=50; aa_Nyq_az=sp_params.BeamformSVA_NyqFactor; aa_Nyq_el=1; aa_wrap_az=0; aa_wrap_el=0;
    aa_az_angles = asin(-1/(2*Ad) + (0:(N*aa_Nyq_az-1))/(Ad*N*aa_Nyq_az)).';
    aa_el_angles = zeros(size(aa_az_angles,1),1);
end
end