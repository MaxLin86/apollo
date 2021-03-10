% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

%% Plot captured ADC (A-die) samples

% Capture using:
%
%    capture 20 --preset VP105 --json-info --adc adie_capture.json
%    radar-remote-api/python/examples/adc-samples.py --scan-seq 15 .
%
% Example JSON file (adie_capture.json)
%
%    {
%        "adc_capture_channel_bitmap": [0],
%        "adc_capture_mode": 3,
%        "sample_bit_width": 8,
%        "samples_per_channel": 8192,
%        "sel_rx_tx": 0
%    }
%
% Settings for common scans (rx_chips_per_ping, samples_per_chip):
%
%    VP105     512, 2
%    VP104     150, 8
%    VP114     256, 2
%

function plot_adc(file, rx_chips_per_ping, samples_per_chip)

if ~exist('rx_chips_per_ping','var')
    rx_chips_per_ping = 512;
end

if ~exist('samples_per_chip','var')
    samples_per_chip = 2;
end

samples = rx_chips_per_ping * samples_per_chip;

f = fopen(file);
if f>0
    adc = fread(f, 16384, 'int16');
    adc = complex(adc(1:2:end),adc(2:2:end));
    fclose(f);

    len = length(adc);
    len = len - mod(len,samples);
    
    adc = reshape(adc(1:len),samples,[]);

    zoom = 127;
    figure(101);clf;
    subplot(2,1,1);hold on;grid minor;
    plot(real(adc),'.-');
    title('ADC capture: I-channel,  VP-mode, Rx-ping, several pings overlapped');
    ylabel('ADC output value');
    xlabel('ADC sample (within Rx-ping window)');
    xlim([-20 samples+20]);
    ylim([-1 1]*zoom);
    subplot(2,1,2);hold on;grid minor;
    plot(imag(adc),'.-');
    title('ADC capture: Q-channel,  VP-mode, Rx-ping, several pings overlapped');
    ylabel('ADC output value');
    xlabel('ADC sample (within Rx-ping window)');
    xlim([-20 samples+20]);
    ylim([-1 1]*zoom);
    
    fprintf('Mean:    I =%6.2f,  Q =%6.2f\n', mean(real(adc(:))), mean(imag(adc(:))));

else

    fprintf('File not found: "%s"\n', file);

end
