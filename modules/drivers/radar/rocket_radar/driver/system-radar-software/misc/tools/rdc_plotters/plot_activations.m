% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
% plot_activations is a matlab function to plot the RDC3 activations
% Usage: plot_activations(RDC3_summary_file,ULSstr,AngCut_RngBin)
% ULSstr is a string containing the types of activation data files to load
% u == upper, l == lower, s == special summary and RDC3 data
% AngCut_RngBin is array containing the specific range bin values to plot
% E.g. : plot_activations('scan_000000_spsumu.bin','u',[20,40])

function [sparsifiedOutput] = plot_activations(filestr,ULSstr,AngCut_RngBin)

global sparsifiedOutput;

if ~exist('ULSstr','var')
    ULSstr = 'u';
end
if ~exist('AngCut_RngBin','var')
    AngCut_RngBin = [];
end
if ~exist('loadjson.m','file')
    addpath('../jsonlab');
end

% file parts
[fp1, fp2, ~] = fileparts(filestr);
jsonfilepath = [fp1 '/' fp2(1:strfind(fp2,'_spsum')) 'info.json'];
scanData = loadjson(jsonfilepath);

AngleBins =  scanData.num_beamforming_angles;
isComplexRDC3 = scanData.complex_rdc3;
swExponent = scanData.rdc3_software_exponent;

fileIDstr=filestr(max(strfind(filestr,'_'))+1:end-4);
if isempty(strfind(fileIDstr,'sum'))
    error('File Identifier string must be a correctly named summary file, eg scan_000000_spsumu.bin')
end

sRDC3sumFile = [filestr(1:max(strfind(filestr,'_'))) 'spsum' ULSstr '.bin' ];
sRDC3binFile = [filestr(1:max(strfind(filestr,'_'))) 'sprdc3' ULSstr '.bin' ];
	
f = fopen(sRDC3sumFile, 'r');
if f>0
    a=dir(sRDC3sumFile); % for file size
    numSkewers = a.bytes/8;
    for sk = 1:numSkewers
        % zero based range and doppler bin data to 1based
        sparsifiedOutput.activationDopplerBin(sk) = fread(f, 1, 'uint16')+1;
        sparsifiedOutput.activationRangeBin(sk) =  fread(f, 1, 'uint16')+1;
        % max mag from sparsified non-static RDC3
        sparsifiedOutput.activationMaxMag(sk) = fread(f, 1, 'uint16');
        % RDC2 FFT exponent that goes along with that RDC3 max mag
        sparsifiedOutput.RDC2exp(sk) = fread(f, 1, 'int8');
        fread(f, 1, 'uint8');% pad
    end
    fclose(f);
    
    angle_rdc_plot = zeros(numSkewers,AngleBins);
    
    f = fopen(sRDC3binFile, 'r');
    if isComplexRDC3
        temp = fread(f,2*AngleBins*numSkewers,'int16');
        temp = complex(temp(1:2:end),temp(2:2:end));
        sparsifiedOutput.RDC3Activations = transpose(reshape(temp,[AngleBins numSkewers]) );
    else
        sparsifiedOutput.RDC3Activations = transpose(reshape(fread(f,AngleBins*numSkewers,'uint16'),[AngleBins numSkewers]) );
    end
    fclose(f);
    
    % just abs complex data for now
    sparsifiedOutput.RDC3Activations = abs(sparsifiedOutput.RDC3Activations);
    
    % Scale the data for plotting
    if numSkewers>0
        for numSk=1:numSkewers
            exp=sparsifiedOutput.RDC2exp(numSk);
            angle_rdc_plot(numSk,:) = (sparsifiedOutput.RDC3Activations(numSk, :).*2^exp).*2^swExponent;
        end
    end
else
    disp('*** Cant find RDC3 data file ***');
end

% Plot activations
if isempty(AngCut_RngBin)
    mesh(mag2db(angle_rdc_plot),'FaceColor','interp','EdgeColor','None');
else
    foundcell = ismember(sparsifiedOutput.activationRangeBin, AngCut_RngBin);
    AngCut_skewer = foundcell';
    if sum(AngCut_skewer(:)==1)>1
        sparsifiedOutput.activationDopplerBin = sparsifiedOutput.activationDopplerBin(AngCut_skewer);
        sparsifiedOutput.activationRangeBin =  sparsifiedOutput.activationRangeBin(AngCut_skewer);
        mesh(mag2db(angle_rdc_plot(AngCut_skewer,:)),'FaceColor','interp','EdgeColor','None');
    else
        error('AngCut_RngBin values are not matching with activation range bins');
    end
end
colorbar
title 'Raw sparse RDC data'
xlabel 'Angle Bin'
ylabel 'Skewer num '

a=ver;
if any(strcmp({a.Name},'MATLAB'))
    dcm = datacursormode;
    dcm.Enable = 'on';
    dcm.UpdateFcn = @display;	
end
end

function txt = display(~,info)
% Customizes text of data tips
global sparsifiedOutput;

skewer = round(info.Position(2));
AngleBin = round(info.Position(1));
Mag = round(info.Position(3),3);
txt = { ['Skewer: ',num2str(skewer)],...
        ['AngleBin: ',num2str(AngleBin)],...
        ['Mag(dB): ',num2str(Mag)],...
        ['RangeBin: ',num2str(sparsifiedOutput.activationRangeBin(skewer))],...
        ['DoppBin: ',num2str(sparsifiedOutput.activationDopplerBin(skewer))]};
end
