% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2016 Uhnder Inc
% Author:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: dBFSfactor()
%
% Option:
%
% Input:     None
%
% Output:    Many global structs
%
% Calls:
%
%
% Description: This function is called from samSim.m, calculates dependent
%               parameters used elsewhere in the sim
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dBFSfac1, dBFSfac2, dBFSfac3] = dBFSfactor(pulsedMode,PMCW, antenna, digital_BE, scanData)

if exist('scanData','var')
    RSUbits = 3;
    PreCorrelatorScaling = (2^(digital_BE.adcOutputBitDepth+RSUbits-1)-1);
    CodeScaling = 2^(11-1)-1;
else
    RSUbits = 0;
    PreCorrelatorScaling = (2^(digital_BE.adcOutputBitDepth+RSUbits-1)-1);
    CodeScaling = 2^0;
end
    
%RDC 1 factor
if pulsedMode
    %fixme for RDC1 load
    cpp = (PMCW.Lc/2)/ PMCW.txPumpWidth;
    dBFSfac1 = [cpp:cpp:PMCW.R*cpp].' * PMCW.M * PMCW.G * PMCW.K * PreCorrelatorScaling * CodeScaling;
else
    % dBFS adjustment calculation for RDC1 RDC2 RDC3
    dBFSfac1 = ones(PMCW.R,1).* PMCW.Lc * PMCW.M * PMCW.G * PMCW.K * PreCorrelatorScaling * CodeScaling;
end
dBFSfac1 = dBFSfac1;

%RDC 2 factor
dBFSfac2 = dBFSfac1 * PMCW.N;

%RDC 3 factor
dBFSfac3 = dBFSfac2 * antenna.fullyFilled; %fixme use fully filled

dBFSfac1 = mag2db(dBFSfac1);
dBFSfac2 = mag2db(dBFSfac2);
dBFSfac3 = mag2db(dBFSfac3);