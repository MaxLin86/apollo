% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

% parameters:
% hist_fname - histogram file name
% data_fname - ego velocity data file name(list of points)
% example - plot_ego_hist('socsim_000000_ego_vel_histogram.bin', 'socsim_000000_ego_vel_data.bin')
function plot_ego_hist(hist_fname, data_fname)

% parse files
f = fopen(hist_fname, 'r');
hist_size = fread(f, 1, 'uint16');
hist = fread(f, hist_size, 'uint16');
fclose(f);

f = fopen(data_fname, 'r');
num_data = fread(f, 1, 'uint16');

ego_vel_angle = zeros(1, num_data);
ego_vel_doppler = zeros(1, num_data);
ego_vel_magnitude = zeros(1, num_data);
for iter=1:num_data
    ego_vel_angle(iter)=fread(f, 1, 'float');
    ego_vel_doppler(iter)=fread(f, 1, 'float');
    ego_vel_magnitude(iter)=fread(f, 1, 'float'); 
end
fclose(f);

% plot
subplot(2,2,1);
plot(hist);
title('histogram');

subplot(2,2,2);
plot(ego_vel_angle); % radian
title('angle');

subplot(2,2,3);
plot(ego_vel_doppler); % m/s
title('doppler');

subplot(2,2,4);
plot(ego_vel_magnitude);
title('magnitude');
