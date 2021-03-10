% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [] = pallRegrTest(RDCFileA,RDCFileB,targbinRDC,RangeBins,Nvrx,DopplerBins,DCiters)

if nargin ==0
% file paths
% PalFile = '\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/db360_rdc_out_case5/output_spsums.bin';
RDCFileA = '\\192.168.44.114/uhnder-nas/software/jonathan/snr_plot_data/Aoutput_sprdc2s.bin';
RDCFileB = '\\192.168.44.114/uhnder-nas/software/jonathan/snr_plot_data/Boutput_sprdc2s.bin';

RDCFileA = '\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/db360_rdc_out_case5/output_sprdc2s.bin';
RDCFileB = '\\192.168.44.114/uhnder-nas/software/jpb/for_Jonathan/socsim_000001_rdc2.bin';
% scaling factors, leave this empty to skip scaling
% RegFile = '\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/db360_rdc_out_case5/reg_fields.csv';

RegFileA = '\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/db360_rdc_out_case5/reg_fields.csv';
RegFileB = '\\192.168.44.114/uhnder-nas/software/jpb/for_Jonathan/reg_fields.csv';

% target bin per RDC dimension, 0==all bins in that dimension
targbinRDC = [41 0 1];

% scan params
DopplerBins = 360;
Nvrx = 64;
Nangle = 128;
RangeBins = 128;
DCiters = 0;
Nrx = 8;
end

allowPlots = -1; % -1 to suppress 

[a1,a2,~]=fileparts(RDCFileA);
[b1,b2,~]=fileparts(RDCFileB);

[ RDC1scA,RDC2scA,RDC3scA ] = getScaleFactors(RegFileA, DopplerBins);
[ RDC1scB,RDC2scB,RDC3scB ] = getScaleFactors(RegFileB, DopplerBins);

if ~isempty(strfind(a2,'rdc1'))
    analyzeRDC_123 = 1;
    RDCscA = RDC1scA;
    RDCscB = RDC1scB;
elseif ~isempty(strfind(a2,'rdc2'))
    analyzeRDC_123 = 2;
    RDCscA = RDC2scA;
    RDCscB = RDC2scB;
elseif ~isempty(strfind(a2,'rdc3'))
    analyzeRDC_123 = 3;
    RDCscA = RDC3scA;
    RDCscB = RDC3scB;
else
    error('Check file naming, must have name containing rdc1/2/3');
end


%% RDC1 Analysis
if analyzeRDC_123==1
    disp('Loading Palladium RDC1')
    mark = max(strfind(a2,'_'));
    RDC1binFile = [a1 '/' a2(1:mark) 'rdc1.bin'];
    RDC1expFile = [a1 '/' a2(1:mark) 'rdc1exp.bin'];
    [RDCdataA, Exp1A] = plot_RDC1data(RDC1binFile,RDC1expFile,RangeBins,Nvrx,DopplerBins,allowPlots);
    mark = max(strfind(b2,'_'));
    RDC1binFile = [b1 '/' b2(1:mark) 'rdc1.bin'];
    RDC1expFile = [b1 '/' b2(1:mark) 'rdc1exp.bin'];
    [RDCdataB, Exp1B] = plot_RDC1data(RDC1binFile,RDC1expFile,RangeBins,Nvrx,DopplerBins,allowPlots);
    assert(all(Exp1A>-1),'all exponents not filled')
    assert(all(Exp1B>-1),'all exponents not filled')
%% RDC2 Analysis
elseif analyzeRDC_123==2
    disp('Loading Sparisfied Palladium RDC2')
    [RDCdataA, Exp2A] = plot_sRDC2data(RDCFileA,'s',RangeBins, Nvrx, DopplerBins, DCiters, allowPlots );
    % [PalRDCdataB, PalExp2B] = plot_sRDC2data(PalFileB,'s',RangeBins, Nvrx, DopplerBins, DCiters, suppressPlots );
    % SOC Sim uses FULL RDC, not sparsified
    mark = max(strfind(b2,'_'));
    RDC2binFile = [b1 '/' b2(1:mark) 'rdc2.bin'];
    RDC2expFile = [b1 '/' b2(1:mark) 'rdc2_exponents.bin'];
    [RDCdataB, Exp2B] = plot_iRDC2data(RDC2binFile,RDC2expFile,RangeBins, Nvrx, DopplerBins, DCiters, allowPlots );

    assert(all(Exp2A>-1),'all exponents not filled')
    assert(all(Exp2B>-1),'all exponents not filled')

%% RDC3 Analysis
elseif analyzeRDC_123 ==3
%     f = fopen(SamSimRDC3expbin, 'r');
%     roughAoAexp = fread(f, RangeBins, 'uint8')';
%     fclose(f);
    disp('Loading Sparisfied Palladium RDC3')
    [RDCdataA, Exp2A] = plot_sRDC3data(RDCFileA,'s',RangeBins, Nangle, DopplerBins, allowPlots );
    % [PalRDCdataB, PalExp2B] = plot_sRDC3data(PalFileB,'s',RangeBins, Nangle, DopplerBins, allowPlots );
    % SOC Sim uses FULL RDC, not sparsified
    mark = max(strfind(b2,'_'));
    RDC3binFile = [b1 '/' b2(1:mark) 'rdc3.bin'];
    RDC3expFile = [b1 '/' b2(1:mark) 'rdc3_exponents.bin'];
    [RDCdataB, Exp2B] = plot_RDC3data(RDC3binFile,RDC3expFile,RangeBins,DopplerBins, Nangle,  allowPlots );

    assert(all(Exp2A>-1),'all exponents not filled')
    assert(all(Exp2B>-1),'all exponents not filled')
    % subtract Samsim Rough AoA exponent (plot_sRDC3data only returns the RDC2exp from the summary file)
%     SamExp3 = unique(roughAoAexp-SamExp2);
%     assert(numel(SamExp3)==1,'SamSim RDC3 changes over rangebins this is not sabine compliant, check sp_params.BeamformSabineScaling')

else
    error('shouldnt get here');
end
    [ SNRpowA_linear, SigPowA_linear ] = calcPallSNR(RDCdataA, targbinRDC, RDCscA )
    [ SNRpowB_linear, SigPowB_linear ] = calcPallSNR(RDCdataB, targbinRDC, RDCscB )
   SNRpowA_dB = 10*log10(SNRpowA_linear)
 SigpowA_dB = 10*log10(SigPowA_linear)
 SNRpowB_dB = 10*log10(SNRpowB_linear)
 SigpowB_dB = 10*log10(SigPowB_linear)
