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
    


 
