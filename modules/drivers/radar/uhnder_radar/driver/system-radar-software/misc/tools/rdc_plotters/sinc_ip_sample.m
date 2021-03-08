% Owner     : Uhnder Inc
% Developer : Ganesh Bhokare
% Date      : 09 Mar 2018
% File      : Interpolation using Sinc function. 

% Algorithm to interpolate data using Sinc interpolation function.
function [x_ip_sample] = sinc_ip_sample(x,samp_interval,ip_time_index)
% Input parmeters:
    % x             : Input sampled signal
    % samp_interval : Interval between two samples of signal x
    % ip_time_index : Tme index of sample to be interpolated.
% Output parmeters:
    % x_ip_sample   : Interpolated sample
    
    % Initialize interpolation parameter.
    if(mod(length(x),2) == 0)
        N       = (length(x))/2;
        offset  = ((length(x))/2)+1;
    else
        N       = (length(x)-1)/2;
        offset  = ((length(x)+1)/2);
    end   
    Ts      = samp_interval;
    t_m     = ip_time_index;
    
    % Iteration for number of samples of x.
    x_ip_sample = 0;
    for n = -N:N-1
        % Compute Sinc value at specific time instant at t_m
        val = (pi/Ts)*(t_m-n*Ts);
        if(val == 0)
            sinc_val = 1;
        else
            sinc_val = sin(val)/val;
        end
        % Accumulation of input samples using sinc interpolation function.
        x_ip_sample = x_ip_sample + x(n+offset) * sinc_val;
    end
end
