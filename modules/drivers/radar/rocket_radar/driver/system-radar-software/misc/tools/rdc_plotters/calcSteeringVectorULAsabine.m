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
% Author: Steve Borho
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: calcSteeringVectorULA(azimuths, ranges)
%       
% Option:
%
% Input:    azimuths(theta)  - row vector of azimuth angles, in radians
%           ranges(rangebin) - row vector of ranges, in meters
%           vrxTx(3,vrx)     - row vector of NVrx transmitter positions (X,Y,Z)
%           vrxRx(3,vrx)     - row vector of NVrx receiver positions (X,Y,Z)
%
% Output:   
%
% Calls:    none
%       
%
% Description:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ A, C, tau, upsilon, rho ] = calcSteeringVectorULA(azimuths, elevations, ranges, vrxTx, vrxRx, flag_win)

global antenna
global scenario
global sp_params
global detection_params

if ~isfield(antenna,'hwRxMap')
    antenna.hwRxMap = 1:8
end
if ~isfield(antenna,'hwTxMap8')
    antenna.hwTxMap8 = 1:8
end
    
lambda = scenario.lambda_c;
% Calculate per-VRx positions of Tx and Rx antennas
if nargin > 3%2                       % Use positions provided by caller
    if size(vrxTx) ~= size(vrxRx);
        error('vrxTx and vrxRx must be same size');
    end
    if size(vrxTx,1) ~=3 || size(vrxRx,1) ~= 3
        error('vrxTx and vrxRx must have 3 rows containing X,Y,Z positions');
    end
    NVrx = size(vrxTx,2);
else                                % Use positions from antenna data structure
    flag_win = 1;
    NVrx = antenna.Ntx * antenna.Nrx;
    vrxTx = zeros(3, NVrx);
    vrxRx = zeros(3, NVrx);
    for tt = 1 : antenna.Ntx
        ctt=antenna.hwTxMap8(tt);
        for rr = 1 : antenna.Nrx
            crr = antenna.hwRxMap(rr);
            vrx = antenna.mapToVrx((ctt - 1) * antenna.Nrx + crr);
            vrxTx(:,vrx) = antenna.txposoffs(:,ctt);
            vrxRx(:,vrx) = antenna.rxposoffs(:,crr);
        end
    end
end

% Calculate window
if sp_params.switch2DMIMO
    
    if sp_params.BeamformUseAllVRx || nargin > 3%2
        if sp_params.BeamformUseBoxcar
            Waz = getAmplitudeTaper('boxcar', antenna.NVrxAz, []);
            Wel = getAmplitudeTaper('boxcar', antenna.NVrxEl, []);
            W = Waz * Wel.';
        else
            Waz = getAmplitudeTaper('taylor', antenna.NVrxAz, sp_params.desiredSidelobeLevel);
            Wel = getAmplitudeTaper('taylor', antenna.NVrxEl, sp_params.desiredSidelobeLevel);
            W = Waz * Wel.';
        end
    else
        W = zeros(antenna.NVrxAz, antenna.NVrxEl);
        if sp_params.BeamformUseBoxcar
            Waz = getAmplitudeTaper('boxcar', antenna.fullyFilled, []);
            Wel = getAmplitudeTaper('boxcar', antenna.NVrxEl, []); 
        else  
            Waz = getAmplitudeTaper('taylor', antenna.fullyFilled, sp_params.desiredSidelobeLevel);
            Wel = getAmplitudeTaper('taylor', antenna.NVrxEl, sp_params.desiredSidelobeLevel);  
        end
        if isfield(antenna,'filledIndices')
            Waz(antenna.filledIndices) = Waz;
            W = Waz * Wel.';
        else
            WazFF = zeros(antenna.NVrxAz,1);
            pad = (antenna.NVrxAz - antenna.fullyFilled) / 2;
            WazFF(1+pad:end-pad) = Waz;
            W = WazFF * Wel.';
        end
    end
    
else
    
    if 0 %sp_params.BeamformUseAllVRx || nargin > 3%2
        if sp_params.BeamformUseBoxcar
            W = getAmplitudeTaper('boxcar', NVrx, []);
        else
            W = getAmplitudeTaper('chebwin', NVrx, sp_params.desiredSidelobeLevel);
        end
    else
        W = zeros(NVrx, 1);
        if sp_params.BeamformUseBoxcar
            win = getAmplitudeTaper('boxcar', antenna.fullyFilled, []);
        else
            win = getAmplitudeTaper('chebwin', antenna.fullyFilled, sp_params.desiredSidelobeLevel);
        end
        if isfield(antenna,'filledIndices')
            W(antenna.filledIndices) = win;
        else
            pad = (NVrx - antenna.fullyFilled) / 2;
            W(1+pad:end-pad) = win;
        end
    end
end

% Phase and gain error correction
% Use result.dispVRxPhaseForRangeBin = N to generate error estimates
% from a known target in range bin N, at boresight, and at zero Doppler.
if isempty(sp_params.BeamformXCalMat)&&isempty(sp_params.sabineCmx)
    if isempty(sp_params.BeamformPhaseError)
        C = complex(eye(NVrx));
    else
        phaseC = deg2rad(sp_params.BeamformPhaseError);
        if isempty(sp_params.BeamformGainError)
            magC = ones(NVrx);
        else
            magC = db2mag(max(sp_params.BeamformGainError)) ./ db2mag(sp_params.BeamformGainError);
        end
        C = diag(magC .* exp(-1j * phaseC));
    end
else
    if sp_params.tempSkipCal
        C=complex(eye(NVrx));
    else
        C = antenna.CalculatedCalMatrix;
        if ~isempty(sp_params.BeamformPhaseError)
            warning('sp_params.BeamformXCalMat overrides sp_params.BeamformPhaseError');
        end
        if ~isempty(sp_params.BeamformGainError)
            warning('sp_params.BeamformXCalMat overrides sp_params.BeamformGainError');
        end
    end
end

if sp_params.switch2DMIMO
%     elevations = deg2rad(antenna.scan_theta_deg);
    
    % Calculate far-field matrix A for 2D Beamforming
    A = zeros(length(azimuths)*length(elevations), antenna.NVrx);
    for a = 1 : length(azimuths)
        theta = azimuths(a);
        for el = 1: length(elevations)
            phi = elevations(el);
            k = [cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi)];
            aPos = (a-1)*length(elevations) + el;
            
            for vrx = 1 : antenna.NVrx
                d = vrxTx(:, vrx) + vrxRx(:, vrx);
                alpha = 2 * pi * k * d   / lambda;
                
                A(aPos, vrx) = exp(1j * (alpha));
            end
            
            if flag_win == 1
                
                for winSep = 1: antenna.NVrxEl
                    A(aPos, (winSep - 1)*antenna.NVrxAz + 1:winSep*antenna.NVrxAz) = A(aPos, (winSep - 1)*antenna.NVrxAz + 1:winSep*antenna.NVrxAz) .* W(:,winSep).';   % Apply window to each row of A
                end
            end 
        end
    end
else
    % Calculate far-field matrix A for 1D Beamforming
    A = zeros(length(azimuths), NVrx);
    for a = 1 : length(azimuths)
        theta = azimuths(a);
        for vrx = 1 : NVrx
            d = vrxTx(2, vrx) + vrxRx(2, vrx);
            alpha = 2 * pi * d * sin(theta) / lambda;
            A(a, vrx) = exp(1j * alpha);
        end
        if flag_win == 1
            A(a, :) = A(a, :) .* W.';   % Apply window to each row of A
        end
    end
end

% Calculate three vectors needed for HW to calculate near-field correction matrix B
if sp_params.switch2DMIMO

% Tau(theta & phi)
tau = zeros(3,length(azimuths)*length(elevations));
for a = 1 : length(azimuths)
    for el = 1: length(elevations)
        aPos = (a-1)*length(elevations) + el;
        tau(:,aPos) =  [0; pi * cos(azimuths(a)) .^ 2 / lambda; pi* cos(elevations(el)).^ 2 / lambda];
    end
end

% Upsilon(vrx)
upsilon = (vrxTx .^ 2) + (vrxRx .^ 2);

else
    
% Tau(theta)
tau = pi * cos(azimuths) .^ 2 / lambda;

% Upsilon(vrx)
upsilon = (vrxTx(2,:) .^ 2) + (vrxRx(2,:) .^ 2);

end

% Rho(range)
% For Rho, use the min non-zero range bin as range bin zero 
if isfield(detection_params,'rangeGatesMid')
    ranges(find(ranges == 0)) = min(detection_params.rangeGatesMid(detection_params.rangeGatesMid>0));       % prevent division by zero
else
    ranges=ranges;
end
rho = 1.0 ./ ranges;
