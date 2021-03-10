% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%% Plot Static Slice as SNR using per-range DLCR and per-angle Variance for estimating noise floor
%
% To capture data, use a command such as this:
%
%     capture 30 --preset VP114 --json-info
%                --cap-ss-full --cap-hist --cap-ci --cap-det
%                --complex-ss --complex-act
%
% Optionally, add the following Rx and Rx gain options to avoid ADC saturation and
% lower the noise floor. (It makes the DLCR noise floor squared become linear.)
%
%     capture 30 --preset VP114 --json-info
%                --cap-ss-full --cap-hist --cap-ci --cap-det
%                --complex-ss --complex-act
%                --rx-gain-enum 31 --tx-gain-enum 7
%
% The Static Slice data MUST be complex.
% The Scan Info must be in JSON format.
% The HW DLCR histograms and full Static Slice (5 Doppler bins) must be captured.
%
%
% After capturing data, invoke this script as follows:
%
%     plot_SS_SNR('path/to/data', scan_sequence_number, range_bins);
%
% For example, if using VP114, with the data in the current working
% directory, use the following to plot scan #10:
%
%     plot_SS_SNR('.', 10, 256);
%

function plot_SS_SNR(path_name, scan_number, range_bins, doppler_bins, angle_bins)

    %% Default parameters:
    
    if ~exist('path_name','var')
        path_name = '.';
    end

    if ~exist('scan_number','var')
        scan_number = 0;
    end

    if ~exist('range_bins','var')
        range_bins = 512;
    end

    if ~exist('doppler_bins','var')
        doppler_bins = 5;
    end

    if ~exist('angle_bins','var')
        angle_bins = 192;
    end

    %% Other less common configurable parameters:
    
    dlcr_bias    =        0;      % DLCR bias, in dB, relative to Standard Deviation
    first_rbin   =       50;      % First range bin to use in variance calculation
    coloraxis    =  [-6 50];      % Color axis (dB SNR) for the SNR plots
    plot_DLCR    =        1;      % Set to 1 to plot DLCR histograms and per-range noise floor
    num_HistBins =       96;      % 1dB to 96dB HW DLCR
    new_HistBins =      200;
    
    %% Load the data

    ss_name = sprintf('%s/scan_%06d_stslice.bin', path_name, scan_number);
    hh_name = sprintf('%s/scan_%06d_histograms.bin', path_name, scan_number);

    [fp1, fp2, ~] = fileparts(hh_name);
    SDjsonfilepath = [fp1 '/' fp2(1:strfind(fp2,'_histograms')) 'info.json'];
    [PMCW, ~, detection_params, ~, swExponent, ~] = processSabineScanInfo(SDjsonfilepath);
    sw_exponent = swExponent.RDC3;

    if ~exist('range_bins','var')
        range_bins = PMCW.R;
    end

    if ~exist('doppler_bins','var')
        doppler_bins = 5;
    end

    if ~exist('angle_bins','var')
        angle_bins = 192;
    end

    fd_ss = fopen(ss_name, 'r');
    fd_hh = fopen(hh_name, 'r');

    if (fd_ss <= 0) || (fd_hh <= 0)
        disp('Scan data cannot be loaded!!!   Did you capture full Static Slice AND Histograms?')
        if fd_ss > 0
            fclose(fd_ss);
        end
        if fd_hh > 0
            fclose(fd_hh);
        end
        return
    end

    disp('Loading Static Slice data ... ')
    ss_raw = fread(fd_ss, range_bins * doppler_bins * angle_bins * 2, 'int16');
    ss_raw = complex(ss_raw(1:2:end), ss_raw(2:2:end));
    ss = reshape(ss_raw, angle_bins, doppler_bins, range_bins);

    disp('Loading Range DLCR Histogram data ... ')
    hh.rangebins = detection_params.rangeGatesMid;
    hh.Exponent = zeros(range_bins,1);
    hh.histData = zeros(range_bins,num_HistBins);    
    for rr=1:range_bins
        hh.histData(rr,:) = fread(fd_hh, num_HistBins, 'uint16');
        hh.Exponent(rr) = fread(fd_hh,1,'int8');
        hh.maxBinIdx(rr) = fread(fd_hh,1,'uint8');
        fread(fd_hh,1,'uint16');
        fread(fd_hh,1,'uint32');
    end

    if fd_ss > 0
        fclose(fd_ss);
    end
    if fd_hh > 0
        fclose(fd_hh);
    end

    disp('Computing ... ')

    %% Correct DLCR histograms for range bin ordering
    
    goodmask = detection_params.rbininfo.rangeorder>=0;
    reorder = detection_params.rbininfo.rangeorder(goodmask)+1;
    hh.Exponent = hh.Exponent(goodmask);
    hh.Exponent = hh.Exponent(reorder);

    hh.histData = hh.histData(goodmask,:);
    hh.histData = hh.histData(reorder,:);

    hh.maxBinIdx = hh.maxBinIdx(goodmask);
    hh.maxBinIdx = hh.maxBinIdx(reorder);


    %% Shift DLCR histograms according to exponents

    new = zeros(range_bins, new_HistBins);
    for rr = 1 : range_bins
        full_exp = hh.Exponent(rr) + sw_exponent;
        shift_db = 6 * full_exp;
        new(rr,  1 + shift_db : num_HistBins + shift_db) = hh.histData(rr, :);
        db = hh.maxBinIdx(rr) + shift_db;
        hh.maxBinIdx(rr) = db;
        [~, x] = quadratic_interpolate(new(rr,db), new(rr,db+1), new(rr,db+2));
        hh.maxBinInterp(rr) = x + db;
    end

    hh.histData = new;
    range_dlcr = transpose(db2mag(hh.maxBinInterp));

    %% Apply exponents to Static Slice

    for rr = 1 : range_bins
        full_exp = hh.Exponent(rr) + sw_exponent;
        ss(:,:,rr) = ss(:,:,rr) .* 2.^full_exp;
    end

    %% Pre-allocate memory for efficiency

    SNR1          = zeros(size(ss));        % SNR based on per-range DLCR only
    SNR2          = zeros(size(ss));        % SNR based on per-angle Std. Dev. and per-range DLCR
    std_dev       = zeros(size(ss,1),1);    % per-angle Standard Deviation
    std_dev_after = zeros(size(ss,1),1);    % per-angle Standard Deviation computed on SNR2 (should be nearly 1.0)

    %% Compute initial SNR by normalizing the Static Slice using per-range noise floor from HW DLCR
    % Divide SS magnitude by the interpolated peak of the DLCR histograms to compute SNR (linear)
    for abin = 1:angle_bins
        for dbin = 1:doppler_bins
            SNR1(abin,dbin,:) = abs(squeeze(ss(abin,dbin,:))) ./ (range_dlcr * db2mag(dlcr_bias)); 
        end
    end
            
    %% Compute Variance and Standard Deviation of Static Slice, per angle-bin

    for abin = 1:angle_bins
        ssa = squeeze(SNR1(abin,:,first_rbin:end));
        v = sum(( ssa(:) .* ssa(:) )) / (length(ssa(:))-1);
        std_dev(abin) = sqrt(v);
    end
    
    %% Compute final SNR based on per-angle Std. Dev. and per-range DLCR

    for dd = 1:doppler_bins
        SNR2(:,dd,:) = squeeze(SNR1(:,dd,:)) ./ repmat(std_dev,1,size(SNR1,3));   % TODO: subtract SD or divide by SD ???
    end
    
    %% As a final check, compute the Variance and Standard Deviation of final SNR (should be nearly 1.0)
    for abin = 1:angle_bins
        ssa = squeeze(SNR2(abin,:,first_rbin:end));
        v = sum(( ssa(:) .* ssa(:) )) / (length(ssa(:))-1);
        std_dev_after(abin) = sqrt(v);
    end
    
    %% Plotting
    
    disp('Plotting ... ')
    
    %% Plot raw Static Slice, in dB
    %  Each Doppler bin is plotted separately
    figure(200); clf;
    maxss = mag2db(max(abs(ss(:))));
    for dd = 1:5
        subplot(2,3,dd);
        surf(mag2db(abs(squeeze(ss(:,dd,:)))),'LineStyle','None');
        view(90,90)
        ylim([1 angle_bins]);
        xlim([1 range_bins]);
        ylabel('Angle bin');
        xlabel('Range bin');
        caxis([maxss-40 maxss]);
        colormap jet;
        title('Static Slice (dB)');
    end
    subplot(2,3,6);
    plot(range_dlcr.^2);
    grid minor;
    title('Range DLCR SQUARED'); xlabel('Range Bin');
    
    %% Plot Static Slice SNR based on per-angle variance (and per-range DLCR)
    %  Each Doppler bin is plotted separately
    figure(201); clf;
    for dd = 1:5
        subplot(2,3,dd);
        s = squeeze(SNR1(:,dd,:));
        s(s < .5) = .5;
        surf(mag2db(s),'LineStyle','None');   
        view(90,90)
        ylim([1 angle_bins]);
        xlim([1 range_bins]);
        ylabel('Angle bin');
        xlabel('Range bin');
        caxis(coloraxis);
        colormap jet;
        colorbar;
        title('Initial SNR (dB), from per-range DLCR only');
    end
    subplot(2,3,6);
    plot(std_dev);
    grid minor; xlim([0 192]); ylim([0 4]);
    title('Per-Angle Noise Floor (Std. Dev.)');
    xlabel('Angle Bin'); ylabel('Standard Deviation within Angle Bin');

    %% Plot Static Slice SNR based only on per-range HW DLCR
    %  Each Doppler bin is plotted separately
    figure(202); clf;
    for dd = 1:5
        subplot(2,3,dd);
        s = squeeze(SNR2(:,dd,:));
        s(s < .5) = .5;
        surf(mag2db(s),'LineStyle','None');
        view(90,90)
        ylim([1 angle_bins]);
        xlim([1 range_bins]);
        ylabel('Angle bin');
        xlabel('Range bin');
        caxis(coloraxis);
        colormap jet;
        colorbar;
        title('Final SNR (dB), after applying per-angle Std. Dev.');
    end
    subplot(2,3,6);
    plot(std_dev_after);
    grid minor; xlim([0 192]); ylim([0 4]);
    title('Per-Angle Standard Deviation of final SNR');
    xlabel('Angle Bin'); ylabel('Standard Deviation within Angle Bin');

    %% Plot HW DLCR histograms and noise floor (interpolated peaks of histograms)

    if plot_DLCR
        figure(203); clf; subplot(2,1,1); hold on; grid minor;
        xlim([0 range_bins+10]);
        plot(db2mag(hh.maxBinIdx).^2,    '.-');
        plot(db2mag(hh.maxBinInterp).^2, '.-');
        title('Noise Floor ^2  ---   LINEAR');
        ylabel('Magnitude (Linear)');
        xlabel('Range bin');

        subplot(2,1,2); hold on; grid minor;
        xlim([0 new_HistBins]);
        plot(hh.histData.');
        title('HW DLCR Histograms (per-range-bin)');
        ylabel('Count');
        xlabel('dB');
    end
    

end



%% Basic quadratic interpolation (used for finding precise DLCR histogram peaks)

function [ y , x ] = quadratic_interpolate(yl,yc,yu)

    % X coordinates
    xl = -1.0;
    xc =  0.0;
    xu =  1.0;

    % Quadratic interpolation
    d2 = ((yu - yc) / (xu - xc) - (yl - yc) / (xl - xc)) * 2.0 / (xu - xl);
    d1 = ((yu - yc) / (xu - xc)) - (d2 * (xu - xc) * 0.5);

    if (d2 == 0.0)
        x = xc;
        y = yc;
    else
        x = xc - (d1 / d2);
        y = yc + (d1 * ((x - xc) * 0.5));
    end
end

