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

 function [PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(SDjsonfilepath)

scanData = loadjson(SDjsonfilepath);
if ~isfield(scanData,'vp_scan_mode')
        warning('No vp_scan_mode found, making assumptions!!!!');
        pause(5)
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 256; %Half of chips-per-ping
        PMCW.checkOSF=4;
        PMCW.numPumps=2;
        PMCW.corrLength = 5120;
        PMCW.rbCombine = 1;
else
switch scanData.vp_scan_mode
    case 0
        % variable power modes VP1a, VP1b, VP1bb VP1c
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 256; %Half of chips-per-ping
        PMCW.checkOSF=4;
        PMCW.numPumps=2;
        PMCW.corrLength = 5120;
        PMCW.rbCombine = 1;
    case 1 
        % NOT USED
        error('invalid scanmode')
    case 2 
        % NOT USED
        error('invalid scanmode')
    case 3 
        % variable power modes VP4, VP12, VP13
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 512; %Half of chips-per-ping
        PMCW.checkOSF=4;
        PMCW.numPumps=4;
        PMCW.corrLength = 5120;
        PMCW.rbCombine = 1;
    case 4 
        % NOT USED
        error('invalid scanmode')
    case 5 
        % NOT USED
        error('invalid scanmode')
    case 6 
        % NOT USED
        error('invalid scanmode')
    case 7 
        % variable power mode VP8
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 512; %Half of chips-per-ping
        PMCW.checkOSF=2;
        PMCW.numPumps=4;
        PMCW.corrLength = 5120;
        PMCW.rbCombine = 1;
    case 8 
        % variable power mode VP9
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 512; %Half of chips-per-ping
        PMCW.checkOSF=1;
        PMCW.numPumps=4;
        PMCW.corrLength = 5120;
        PMCW.rbCombine = 1;
    case 9 
        % NOT USED 
        error('invalid scanmode')
    case 10
        % variable power mode VP11
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 512; %Half of chips-per-ping
        PMCW.checkOSF=1;
        PMCW.numPumps=4;
        PMCW.corrLength = 2048;
        PMCW.rbCombine = 1;
    case 11 
        % NOT USED 
    case 12
        % NOT USED 
    case 13
        % variable power mode VP14
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 512; %Half of chips-per-ping
        PMCW.checkOSF=1;
        PMCW.numPumps=4;
        PMCW.corrLength = 2048; 
        PMCW.rbCombine = 2;
    case -1
        % experimental modes
        disp('Experimental Scan Mode')
        PMCW.pulsedMode=1;
        PMCW.txPumpWidth = 256; %Half of chips-per-ping
        PMCW.checkOSF=4;
        PMCW.numPumps=1;
    otherwise
        % continuous power modes
        PMCW.pulsedMode=0;
        PMCW.txPumpWidth = -1; %Half of chips-per-ping
        PMCW.checkOSF=4;
        PMCW.numPumps=1;
end
end

% host vel
if isfield(scanData,'ego_velocity_X')
    egoMotion.velocity.x = scanData.ego_velocity_X;
else
    disp('No ego velocity, setting to 0mps');
    egoMotion.velocity.x = 0;
end


if isfield(scanData,'estimated_ego_velocity_X')
    egoMotion.velocity.est_x = scanData.estimated_ego_velocity_X;
else
    disp('No SRS estimated ego velocity, setting to 0mps');
    egoMotion.velocity.est_x = 0;
end

egoMotion.scan_timestamp = scanData.scan_timestamp;

% get range and angle bins from the files
[fp1, fp2, fp3] = fileparts(SDjsonfilepath);
frangebins = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'range_bins.bin'];
f = fopen(frangebins,'r');
if f<0
    disp('No rng bin file. making up bin numbers');
    rawrangebins = ((1:scanData.num_range_bins)-1).';
else
    rawrangebins = fread(f,scanData.num_range_bins,'int16');
    fclose(f);
end
fanglebins = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'angle_bins.bin'];
f = fopen(fanglebins,'r');

if f<0
    disp('No ang bin file. making up bin numbers');
    detection_params.angleGatesMid = zeros(2*scanData.num_beamforming_angles,1);
    warning('no angle bins file');
    warning('no angle bins file');
    warning('no angle bins file');
    warning('no angle bins file');
else
    detection_params.angleGatesMid = fread(f,2*scanData.num_beamforming_angles,'float');
    fclose(f);
end

detection_params.angleGatesMid = reshape(detection_params.angleGatesMid,2,[]).';
if all(rawrangebins==0)
    disp('All range bins zero. making up bin numbers')
    rawrangebins = ((1:scanData.num_range_bins)-1).';
%     rawrangebins = fftshift(rawrangebins);
end

detection_params.num_azimuth_angles = scanData.num_azimuth_angles;

% get "range bin info" aka hist peak and pfa? info
frbininfo = [fp1 '/' fp2(1:findstr(fp2,'_info')) 'rb_info.bin'];
f = fopen(frbininfo,'r');
if f<0
    disp('No rbin_info file. making up bin numbers');
    detection_params.rbininfo = [];
else
    for itsc = 1:scanData.num_range_bins
        detection_params.rbininfo.rangeorder(itsc) = fread(f,1,'int16');
        fread(f,1,'uint16'); %unused pad
        detection_params.rbininfo.nf_max_peak(itsc) = fread(f,1,'float');
        detection_params.rbininfo.nf_pfa(itsc) = fread(f,1,'float');
    end
    fclose(f);
end

if isfield(scanData,'chip_temp_C')
    detection_params.num_detections = scanData.num_detections;
    detection_params.chip_temp_C = scanData.chip_temp_C;
else
    detection_params.num_detections = -999;
    detection_params.chip_temp_C = -999;
end

%basic scan params
PMCW.R = scanData.num_range_bins;
PMCW.Lc = scanData.chips_per_pulse;
antenna.NVrx = scanData.total_vrx;
PMCW.N = scanData.num_pulses;
PMCW.MIMO2D = scanData.CI_format>=20;

PMCW.M = 1;
PMCW.G = 1;
PMCW.K = 1;
PMCW.stslice.abins = scanData.SS_size_A;
PMCW.stslice.dbins = scanData.SS_size_D;
PMCW.stslice.rbins = scanData.SS_size_R;
PMCW.RxOSFden = 1;
PMCW.RxOSFnum = round(scanData.sample_rate/(1/scanData.chip_time));
if abs(scanData.sample_rate/(1/scanData.chip_time)-round(scanData.sample_rate/(1/scanData.chip_time)))>.01
    warning('non-integer OSF found something is wrong in scanInfo!');
end
PMCW.Tc = scanData.chip_time;
PMCW.DoppChanOutputs=scanData.num_channelizer_doppler_bins;
PMCW.Fs = scanData.sample_rate;
PMCW.DoppChanOutputs = scanData.num_channelizer_doppler_bins;
PMCW.DoppChanIters = scanData.num_channelizer_iters;
%variable-power mode settings
try
    maxdBFScorr_lin.RDC1 = scanData.rdc1_full_scale_value;
    maxdBFScorr_lin.RDC2 = scanData.rdc2_full_scale_value;
    maxdBFScorr_lin.RDC3 = scanData.rdc3_full_scale_value;
catch
    disp('caught exception on old json format, attempting to continue')
    maxdBFScorr_lin.RDC1 = 1;
    maxdBFScorr_lin.RDC2 = 1;
    maxdBFScorr_lin.RDC3 = 1;
end
swExponent.RDC1 = scanData.rdc1_software_exponent;
swExponent.RDC2 = scanData.rdc2_software_exponent;
swExponent.RDC3 = scanData.rdc3_software_exponent;
swExponent.CI = scanData.clutter_image_exponent;
swExponent.SYS = scanData.system_exponent;
% get the range bins ordered right
% pingsPerPulse = PMCW.Lc/(2*PMCW.txPumpWidth);
goodbins = rawrangebins>=0;
rawrangebins = rawrangebins(goodbins);
[detection_params.rangeBins detection_params.rangeGatesOrdering ] = sort(rawrangebins);
detection_params.rangeGatesMid=detection_params.rangeBins.*scanData.range_bin_width;
detection_params.isCI2D = scanData.CI_format==26; % used in Clutter image plot

PRF=1/scanData.pulse_time;
digital_FE.Vamb = (PRF/2)*(299792458/76.5e9)/2;
FbinFFT = PRF / (PMCW.N * PMCW.M * PMCW.G * PMCW.K); % doppler bin resolution with FFT
digital_FE.VdBinFFT = FbinFFT.*(299792458/76.5e9)/2;  % Doppler speed bin with FFT

if logical(mod(PMCW.N,2))
    detection_params.dopplerGatesMidFFT = -digital_FE.Vamb+digital_FE.VdBinFFT/2:digital_FE.VdBinFFT:digital_FE.Vamb-digital_FE.VdBinFFT/2;
else
    detection_params.dopplerGatesMidFFT = -digital_FE.Vamb:digital_FE.VdBinFFT:digital_FE.Vamb-digital_FE.VdBinFFT+100*eps; % added 100*eps of fudge for rounding
end

%check that the range_bins_in_meters are close based on the given range_bin_width
assert(all(abs(rem(detection_params.rangeGatesMid(detection_params.rangeGatesMid>=0)./ scanData.range_bin_width,1))<0.1),'Range bins are not correct distance apart based on scanData.range_bin_width')
chipposn = 1+detection_params.rangeBins;

% calc per range bin dBFS
if PMCW.pulsedMode
    rbcombine = round(scanData.range_bin_width/(scanData.chip_time*3e8/2));
    if rbcombine>1
        temprangebins = (1:scanData.num_range_bins*rbcombine)';
        chipposn(:)=0;
        for itc=1:rbcombine
            chipposn = chipposn+temprangebins(itc:rbcombine:end);
        end
    end
    perBinFactor=chipposn/(PMCW.txPumpWidth*rbcombine);
    dBFScorr_lin.RDC1 = maxdBFScorr_lin.RDC1*perBinFactor;
    dBFScorr_lin.RDC2 = maxdBFScorr_lin.RDC2*perBinFactor;
    dBFScorr_lin.RDC3 = maxdBFScorr_lin.RDC3*perBinFactor;
    dBFScorr_lin.RDC1max = maxdBFScorr_lin.RDC1*ones(PMCW.R,1);
    dBFScorr_lin.RDC2max = maxdBFScorr_lin.RDC2*ones(PMCW.R,1);
    dBFScorr_lin.RDC3max = maxdBFScorr_lin.RDC3*ones(PMCW.R,1);
else
    dBFScorr_lin.RDC1 = maxdBFScorr_lin.RDC1*ones(PMCW.R,1);
    dBFScorr_lin.RDC2 = maxdBFScorr_lin.RDC2*ones(PMCW.R,1);
    dBFScorr_lin.RDC3 = maxdBFScorr_lin.RDC3*ones(PMCW.R,1);
end


antenna.mapToVrx = scanData.vrx_map+1;

% for RDC3 dBFS
antenna.fullyFilled = scanData.rdc3_full_scale_value/scanData.rdc2_full_scale_value;
disp([ num2str(antenna.fullyFilled) ' fully filled virtual receivers']);

% commented out debug text
% disp(['Scandata RDC1/2/3 dBFS factors: ' num2str(max(mag2db(dBFScorr_lin.RDC1))) 'dB, ',  num2str(max(mag2db(dBFScorr_lin.RDC2))) 'dB, '  num2str(max(mag2db(dBFScorr_lin.RDC3))) 'dB']);
digital_BE.adcOutputBitDepth = 8;
[dBFSfac1, dBFSfac2, dBFSfac3] = dBFSfactor(PMCW.pulsedMode,PMCW, antenna, digital_BE, scanData);
% disp(['Matlab RDC1/2/3 dBFS factors: ' num2str(max(dBFSfac1)) 'dB, ' num2str(max(dBFSfac2)) 'dB, ' num2str(max(dBFSfac3)) 'dB']);
