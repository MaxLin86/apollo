% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [ RDC1sc,RDC2sc,RDC3sc ] = getScaleFactors(RegFile, DopplerBins )
% FEU stages 1==disabled (no scaling)
FEU_FFT_stages=[[ 105,    5, 7, 3, 1 ];
    [ 112,    1, 7, 1, 16 ];
    [ 120,    5, 1, 3, 8 ];
    [ 126,    1, 7, 9, 2 ];
    [ 140,    5, 7, 1, 4 ];
    [ 144,    1, 1, 9, 16 ];
    [ 168,    1, 7, 3, 8 ];
    [ 180,    5, 1, 9, 4 ];
    [ 210,    5, 7, 3, 2 ];
    [ 240,    5, 1, 3, 16 ];
    [ 252,    1, 7, 9, 4 ];
    [ 280,    5, 7, 1, 8 ];
    [ 315,    5, 7, 9, 1 ];
    [ 336,    1, 7, 3, 16 ];
    [ 360,    5, 1, 9, 8 ];
    [ 420,    5, 7, 3, 4 ];
    [ 504,    1, 7, 9, 8 ];
    [ 560,    5, 7, 1, 16 ];
    [ 630,    5, 7, 9, 2 ];
    [ 720,    5, 1, 9, 16 ];
    [ 840,    5, 7, 3, 8 ];
    [ 1008,   1, 7, 9, 16 ];
    [ 1260,   5, 7, 9, 4 ];
    [ 1680,   5, 7, 3, 16 ];
    [ 2520,   5, 7, 9, 8 ];
    [ 5040,   5, 7, 9, 16 ]];

enabledFEUstages = (FEU_FFT_stages(FEU_FFT_stages(:,1)==DopplerBins,2:end)~=1);
maxAllowedScaleFactor = [ 0, 0, 19, 0, 21, 0, 22, 19, 23, 0, 0, 0, 0, 0, 0, 21]  ;
magicImplicitShift    = [ 0, 0, 21, 0, 21, 0, 21, 21, 21, 0, 0, 0, 0, 0, 0, 21]  ;
magicFEU = sum(magicImplicitShift(FEU_FFT_stages(FEU_FFT_stages(:,1)==DopplerBins,2:end)));

% Scalin Factors Table Work
if ~isempty(RegFile)
    % read and parse array
    out = textread(RegFile, '%s', 'whitespace',',');
    out=out(~strcmp(out,'')); % octave bug compensation
    out = reshape(out,[4 numel(out)/4])';
    
    % CIU scale fact
    InputCIUsc = sum(str2num(char(out(logical([0; hex2dec(out(2:end,1))>=hex2dec('b1160008') & hex2dec(out(2:end,1))<=hex2dec('b1160014')]),4))));
    
    % FEU scale fact
    currScaleFact = str2num(char(out(logical([0; hex2dec(out(2:end,1))==hex2dec('b119006c')]),4)));
    allowedMaxSc = maxAllowedScaleFactor(FEU_FFT_stages(FEU_FFT_stages(:,1)==DopplerBins,2:end));
    testAll = currScaleFact.'<=allowedMaxSc;
    if ~all(testAll(enabledFEUstages))
        warning(['FEU Scale factor(' num2str(currScaleFact(enabledFEUstages&~testAll)) ') is set greater than max allowable(' num2str(allowedMaxSc(enabledFEUstages&~testAll)) ') for the current Winograd set'])
%         currScaleFact(enabledFEUstages&~testAll)=allowedMaxSc(enabledFEUstages&~testAll);
    end
    feu_mult_scal = sum(currScaleFact(enabledFEUstages));
    feu_fxd_scale_en=str2num(char(out(logical(strcmp(out(:,3),'feu_fxd_scale_en')),4)));
    if feu_fxd_scale_en
        % fixed scaling
        feu_fxd_scale_val = sum(str2num(char(out(logical([0; hex2dec(out(2:end,1))==hex2dec('b119002c')]),4))));
        InputFEUsc = feu_mult_scal+feu_fxd_scale_val;
    else
        % dyn scaling
        InputFEUsc = feu_mult_scal;
    end
    
    % RAU Scale Fact
    InputRAUsc = sum(str2num(char(out(logical(strcmp(out(:,3),'matrix_normalize')),4))));
    
    % magic numbers rom bit growth in HW
    magicCIU = 195; % magic number for CIU correlation bits
%     magicFEU = 21*tot_FEU_bits; % magic number for CIU doppler trans bits
    magicRAU = 15; % magic number for RAU beamforming bits
    
    % delta scaling factors to use on data comparison
    RDC1sc = InputCIUsc - magicCIU; % Palladium to Sam Sim RDC1 scaling factor
    RDC2sc = RDC1sc + InputFEUsc - magicFEU; % Palladium to Sam Sim RDC2 scaling factor
    RDC3sc = RDC2sc + InputRAUsc - magicRAU; % Palladium to Sam Sim RDC3 scaling factor
else
    RDC1sc = 0; %  RDC1 scaling factor
    RDC2sc = 0; %  RDC2 scaling factor
    RDC3sc = 0; %  RDC3 scaling factor
end

end

