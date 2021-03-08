% Owner     : Uhnder Inc
% Developer : Ganesh Bhokare
% Date      : 14 Mar 2018
% File      : Estimation/interpolation of peak sample bin and magnitude in 
%             Sabine RADAR beamforming and Range FFT algorithms.

%% Algorithm to interpolate data using Sinc interpolation function.
function [x_ip,t] = sinc_ip(x,x_tstart,x_tend,samp_interval,ip_factor)
% Input Parmeters:
    % x             : Input signal
    % x_tstart      : Start time of input signal
    % x_tend        : End time of input signal
    % samp_interval : Interval between two samples of signal x
    % ip_factor     : Interpolation factor
    
% Output Parameters:
    % x_ip          : Interpolated samples
    % t             : Interpolated time

    % Set parmaters for interpolation algo.
    M  = ip_factor;
    Ts = samp_interval;

    % Iteration for all the samples of interpolated output signal.
    x_ip = x;
    for m = 1:M
        % Interpolation of signal by factor 2
        [x_ip,t] = sinc_ip_by2(x_ip,x_tstart,x_tend,Ts);
        Ts = Ts/2;
    end
end

%% Algorithm to interpolate data using Sinc interpolation function.
function [x_ip,t] = sinc_ip_by2(x,x_tstart,x_tend,samp_interval)
% Input Parmeters:
    % x             : Input signal
    % x_tstart      : Start time of input signal
    % x_tend        : End time of input signal
    % samp_interval : Interval between two samples of signal x
    % ip_factor     : Interpolation factor
    
% Output Parameters:
    % x_ip          : Interpolated samples
    % t             : Interpolated time

    % Set parmaters for interpolation algo.
    M  = 2;
    Ts = samp_interval;
    t  = (x_tstart:Ts/M:x_tend)';
    t_offset = x_tstart + (x_tend-x_tstart)/2;
     
    % Iteration for all the samples of interpolated output signal.
    x_ip = zeros(1,length(t));
    for m = 1:length(t)
        % Interpolation of one sample at one time instant t(m).
        x_ip(m) = sinc_ip_sample(x,Ts,t(m)-t_offset);
    end
end