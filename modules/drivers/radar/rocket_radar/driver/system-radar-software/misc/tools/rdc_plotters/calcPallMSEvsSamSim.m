% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

analyzeRDC1 = 0;
analyzeRDC2 = 1;
analyzeRDC3 = 1;
allowPlots = 1;
% file paths
SamFile ='\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/scan1case5_Lc4096_N360/UpdatedRDC3Scaling/scan1case5a_Lc4096_N360/dvVectors_spsums.bin';
PalFile = '\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/db360_rdc_out_case5/output_spsums.bin';
SoCFile = '\\192.168.44.114/uhnder-nas/software/jpb/for_Jonathan/socsim_000001_rdc2.bin';
PalRegFile = '\\192.168.44.114/uhnder-nas/software/unas-srikanth/sabine_scanwars/db360_rdc_out_case5/reg_fields.csv';
SoCRegFile = [];

[a1, a2, ~]=fileparts(SamFile);
[b1, b2, ~]=fileparts(PalFile);
[c1, c2, ~]=fileparts(SoCFile);

SamSimRDC3expbin = [a1 '/dvVectors_rdc3exp.bin'];

% target bin per RDC dimension, 0==all bins in that dimension
targbinRDC1 = [41 0 1];
targbinRDC2 = [41 0 1];
targbinRDC3 = [41 0 1];

% scan params
DopplerBins = 360;
Nvrx = 64;
Nangle = 128;
RangeBins = 128;
DCiters = 0;
Nrx = 8;

[ RDC1scA,RDC2scA,RDC3scA ] = getScaleFactors(PalRegFile, DopplerBins);
[ RDC1scB,RDC2scB,RDC3scB ] = getScaleFactors(SoCRegFile, DopplerBins);


%% RDC1 Analysis
if analyzeRDC1
    disp('Loading Sparisfied Samsim RDC1')
    mark = max(strfind(a2,'_'));
    RDC1binFile = [a1 '/' a2(1:mark) 'rdc1.bin'];
    RDC1expFile = [a1 '/' a2(1:mark) 'rdc1exp.bin'];
    [SamRDC1Data, SamExp1] = plot_RDC1data(RDC1binFile,RDC1expFile,RangeBins,Nvrx,DopplerBins,allowPlots)
    disp('Loading Sparisfied Palladium RDC1')
    mark = max(strfind(b2,'_'));
    RDC1binFile = [b1 '/' b2(1:mark) 'rdc1.bin'];
    RDC1expFile = [b1 '/' b2(1:mark) 'rdc1exp.bin'];
    [PalRDC1Data, PalExp1] = plot_RDC1data(RDC1binFile,RDC1expFile,RangeBins,Nvrx,DopplerBins,allowPlots)
    mark = max(strfind(c2,'_'));
    RDC1binFile = [c1 '/' c2(1:mark) 'rdc1.bin'];
    RDC1expFile = [c1 '/' c2(1:mark) 'rdc1exp.bin'];
    [SocRDC1Data, SocExp1] = plot_RDC1data(RDC1binFile,RDC1expFile,RangeBins,Nvrx,DopplerBins,allowPlots);

    % check for missing exponents
    assert(all(SamExp1>-1),'all exponents not filled')
    assert(all(PalExp1>-1),'all exponents not filled')
    assert(all(SocExp1>-1),'all exponents not filled')

    % remap Vrxs for pal and soc
    antenna_map = [1,9,2,17,10,3,25,18,11,4,26,19,12,5,27,20,13,6,28,21,14,33,29,22,41,34,30,49,42,35,57,50,43,36,58,51,44,37,59,52,45,38,60,53,46,7,61,54,15,8,62,23,16,31,24,32,39,47,40,55,48,63,56,64];
    PalRDC1Data = PalRDC1Data(:,antenna_map,:);
    SocRDC1Data = SocRDC1Data(:,antenna_map,:);
    
    % scale samsim data down to pal level and floor it to make it cheezier
    PalRDC1Data = floor(PalRDC1Data.*(2^RDC1scA));
    SocRDC1Data = floor(SocRDC1Data.*(2^RDC1scB));

    [ MSEAll1_sam_pal, MSETarg1_sam_pal ] = calcMSE( SamRDC1Data, PalRDC1Data, targbinRDC1 )
    [ MSEAll1_soc_pal, MSETarg1_soc_pal ] = calcMSE( SocRDC1Data, PalRDC1Data, targbinRDC1 )
    [ MSEAll1_sam_soc, MSETarg1_sam_soc ] = calcMSE( SamRDC1Data, SocRDC1Data, targbinRDC1 )
        
        

end

%% RDC2 Analysis
if analyzeRDC2
    disp('Loading Sparisfied Samsim RDC2')
    [SamRDC2Data, SamExp2] = plot_sRDC2data(SamFile,'uls',RangeBins, Nvrx, DopplerBins, DCiters, allowPlots );
    disp('Loading Sparisfied Palladium RDC2')
    [PalRDC2Data, PalExp2] = plot_sRDC2data(PalFile,'s',RangeBins, Nvrx, DopplerBins, DCiters, allowPlots );

    disp('Loading SocSim iRDC2')
    mark = max(strfind(c2,'_'));
    RDC2binFile = [c1 '/' c2(1:mark) 'rdc2.bin'];
    RDC2expFile = [c1 '/' c2(1:mark) 'rdc2_exponents.bin'];
    [SocRDC2Data, SocExp2] = plot_iRDC2data(RDC2binFile,RDC2expFile,RangeBins, Nvrx, DopplerBins, DCiters, allowPlots );

    % check for missing exponents
    assert(all(SamExp2>-1),'all exponents not filled')
    assert(all(PalExp2>-1),'all exponents not filled')
    assert(all(SocExp2>-1),'all exponents not filled')

    % remap Vrxs for pal and soc
    antenna_map = [1,9,2,17,10,3,25,18,11,4,26,19,12,5,27,20,13,6,28,21,14,33,29,22,41,34,30,49,42,35,57,50,43,36,58,51,44,37,59,52,45,38,60,53,46,7,61,54,15,8,62,23,16,31,24,32,39,47,40,55,48,63,56,64];
    PalRDC2Data = PalRDC2Data(:,antenna_map,:);
    SocRDC2Data = SocRDC2Data(:,antenna_map,:);
        
    % scale samsim data down to pal level and floor it to make it cheezier
    % SamRDC1Data = floor(SamRDC1Data/(2^RDC1scA));
    PalRDC2Data = floor(PalRDC2Data.*(2^RDC2scA));
    SocRDC2Data = floor(SocRDC2Data.*(2^RDC2scB));
    
    [ MSEAll2_sam_pal, MSETarg2_sam_pal ] = calcMSE( SamRDC2Data, PalRDC2Data, targbinRDC2 )
    [ MSEAll2_soc_pal, MSETarg2_soc_pal ] = calcMSE( SocRDC2Data, PalRDC2Data, targbinRDC2 )
    [ MSEAll2_sam_soc, MSETarg2_sam_soc ] = calcMSE( SamRDC2Data, SocRDC2Data, targbinRDC2 )

end

%% RDC3 Analysis
if analyzeRDC3
    f = fopen(SamSimRDC3expbin, 'r');
    roughAoAexp = fread(f, RangeBins, 'uint8')';
    fclose(f);
    disp('Loading Sparisfied Samsim RDC3')
    [SamRDC3Data, SamExp2] = plot_sRDC3data(SamFile,'uls',RangeBins, Nangle, DopplerBins );
    disp('Loading Sparisfied Palladium RDC3')
    [PalRDC3Data, PalExp2] = plot_sRDC3data(PalFile,'s',RangeBins, Nangle, DopplerBins );
    
    disp('Loading SocSim iRDC2')
    mark = max(strfind(c2,'_'));
    RDC3binFile = [c1 '/' c2(1:mark) 'rdc3.bin'];
    RDC3expFile = [c1 '/' c2(1:mark) 'rdc3_exponents.bin'];
    [SocRDC3Data, SocExp3B] = plot_RDC3data(RDC3binFile,RDC3expFile,RangeBins,DopplerBins, Nangle,  allowPlots );

    
    assert(all(SamExp2>-1),'all exponents not filled')
    assert(all(PalExp2>-1),'all exponents not filled')
    % subtract Samsim Rough AoA exponent (plot_sRDC3data only returns the RDC2exp from the summary file)
    SamExp3 = unique(roughAoAexp-SamExp2);
    assert(numel(SamExp3)==1,'SamSim RDC3 changes over rangebins this is not sabine compliant, check sp_params.BeamformSabineScaling')
        
    % scale samsim data down to pal level and floor it to make it cheezier
    SamRDC3Data = floor(SamRDC3Data.*(2^SamExp3));
    PalRDC3Data = floor(PalRDC3Data.*(2^RDC3scA));
    SocRDC3Data = floor(SocRDC3Data.*(2^RDC3scB));
        
    [ MSEAll3_sam_pal, MSETarg3_sam_pal ] = calcMSE( SamRDC3Data, PalRDC3Data, targbinRDC3 )
    [ MSEAll3_soc_pal, MSETarg3_soc_pal ] = calcMSE( SocRDC3Data, PalRDC3Data, targbinRDC3 )
    [ MSEAll3_sam_soc, MSETarg3_sam_soc ] = calcMSE( SamRDC3Data, SocRDC3Data, targbinRDC3 )

    
    %[ MSEAll3, MSETarg3 ] = calcMSE(SamRDC3Data, PalRDC3Data, targbinRDC3, RDC3sc - SamExp3 )
end

