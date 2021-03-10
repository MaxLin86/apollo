% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2016 Uhnder Inc
% Author: J. Preussner
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage:
%       outputData = reorderPulsedTimeRdc(inputData)
% Option:
%       inputData - time domain correlator data input
%           size: RangeBins, NumVrx, Npulses
%
%       outputData - freq domain doppler output data
%           size: RangeBins, NumVrx, numChannelizerBranches, numValidBranches
%
% Calls:
%       remez, myfrf.m
%
% Description:
%               Implementation of a polyphase channelizer designed by Fred
%               Harris.  This script takes in time domain rdc data and
%               outputs frequency domain doppler data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [time_rdc, baseRangeExponent, rangeExponentsMx, rangeExpReserved] = reorderPulsedTimeRdc(RangeBins,sp_params,time_rdc, baseRangeExponent, rangeExponentsMx, rangeExpReserved)

if nargin<=3
    RDConly=1;
else
    RDConly=0;
end

if sp_params.combinedLoadRDCPumpDat == 4 && RangeBins == 128
    disp(' Reordering 4 Pumps with combined range bins')
    reord= [4:128 3 2 1 ];
    time_rdc = time_rdc(reord,:,:);
    if ~RDConly
        baseRangeExponent = baseRangeExponent(reord);
        rangeExponentsMx = rangeExponentsMx(reord,:);
        rangeExpReserved = rangeExpReserved(reord);
    else
        baseRangeExponent= [];
        rangeExponentsMx = [];
        rangeExpReserved = [];
    end
elseif sp_params.combinedLoadRDCPumpDat == 2 && RangeBins == 128
    disp(' Reordering 4 Pumps with combined range bins')
    reord= [2:128 1 ];
    time_rdc = time_rdc(reord,:,:);
    if ~RDConly
        baseRangeExponent = baseRangeExponent(reord);
        rangeExponentsMx = rangeExponentsMx(reord,:);
        rangeExpReserved = rangeExpReserved(reord);
    else
        baseRangeExponent= [];
        rangeExponentsMx = [];
        rangeExpReserved = [];
    end
elseif (sp_params.combinedLoadRDCPumpDat == 0 )
    disp(' Reordering Pumps uncombined pumps')
    temp=[fliplr(1:128:size(time_rdc,1)); size(time_rdc,1):-128:1];
    newtime_rdc = [];
    newbaseRangeExponent= [];
    newrangeExponentsMx = [];
    newrangeExpReserved = [];
    for itjum=1:size(temp,2)
        newtime_rdc= [ newtime_rdc; time_rdc(temp(1,itjum):temp(2,itjum),:,:) ];
        if ~RDConly
            newbaseRangeExponent = [newbaseRangeExponent; baseRangeExponent(temp(1,itjum):temp(2,itjum)) ];
            newrangeExponentsMx = [ newrangeExponentsMx rangeExponentsMx(temp(1,itjum):temp(2,itjum),:) ];
            newrangeExpReserved = [ newrangeExpReserved rangeExpReserved(temp(1,itjum):temp(2,itjum)) ];
        end
    end
    time_rdc = newtime_rdc;
    baseRangeExponent = newbaseRangeExponent;
    rangeExponentsMx = newrangeExponentsMx;
    rangeExpReserved = newrangeExpReserved;
else
    error('undefined config FIXME');
end
    


 
