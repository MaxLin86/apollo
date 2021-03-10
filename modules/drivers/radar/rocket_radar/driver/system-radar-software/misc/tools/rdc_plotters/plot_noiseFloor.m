% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

function plot_noiseFloor(filestr)

    % "filestr" should be the pathname of a 'scan_000000_rb_info.bin' file,
    % which contains a vector of the following structure, with one structure for each range bin:

    % struct UhdpRangeBinInfo    /* UHDP_TYPE_RANGE_BINS */
    % {
    %     int16_t  distance_in_bins;        //!< distance of range bin from radar, in units of range_bin_width - negative means invalid
    %     int16_t  reverse_map;             //!< map distance to range bin (indexed by distance in bins) - negative means not represented
    %     FLOAT    noise_floor_max_peak_dB; //!< the max peak of the power histogram in this range
    %     int16_t  exponent;
    %     int16_t  reserved;
    % };

    % The noise_floor_max_peak_dB includes the HW exponent (which is 'exponent'),
    % but it does NOT include the SW exponents
    
	len = 0;
    f = fopen(filestr, 'r');
    if f>0
        while(1)
            dbins = fread(f, 1, 'int16');
            if ~isempty(dbins)
                len = len + 1;
                rev_map = fread(f, 1, 'int16');
                nf = fread(f, 1, 'float32');
                exp = fread(f, 1, 'int16');
                resvd = fread(f, 1, 'int16');

                distance_in_bins(len) = dbins;
                reverse_map(len) = rev_map;
                noise_floor_max_peak_dB(len) = nf;
                exponent(len) =  exp;            
            else
                break;
            end
        end
        
        plot(noise_floor_max_peak_dB,'.-');
        grid;
        xlim([0 len+2]);
        ylabel('dB');
        xlabel('Range Bin');
        title('Noise Floor');
    end
end


