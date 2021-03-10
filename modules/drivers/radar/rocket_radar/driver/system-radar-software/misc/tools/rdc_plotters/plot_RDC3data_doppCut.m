% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [angle_rdc, exponents] = plot_RDC3data_doppCut(RDC3binFile,RDC3expFile,RangeBins,DopplerBins,AngleBins, doppCut, isComplexRDC3)
% eg.
% [rdc3plot] = plot_RDC3data('lrr1-1_rdc3.bin','lrr1-1_rdc3exp.bin',80,32,320,0,3,0)

% set doppCut to zero to disable plots, set to nonzero to plot that 1based Dopplerbin Rng Ang cut 

if(doppCut==0)
   plotson=0; 
else 
    plotson=1;
end

% min is set to the max of this value and the smallest 2^exponent
minval = db2mag(10);

AngExp2d = 1; % angle exponents 2d (range by doppler) or 1d (range only)

% complex RDC3
if ~exist('isComplexRDC3','var')
    isComplexRDC3 = 0;
end

% File load from binary
f1 = fopen(RDC3binFile, 'r');
f2 = fopen(RDC3expFile, 'r');
if f1>0 && f2>0
    
    % Load the RDC3 exponents
    angle_exp = zeros(RangeBins, DopplerBins);
    for rr = 1:RangeBins
        if AngExp2d
            angle_exp(rr, :) = fread(f2, DopplerBins, 'int8').'; % should this be 1d or 2d
        else
            angle_exp(rr, :) = fread(f2, 1, 'int8').'; 
        end
    end
    fclose(f2);
    exponents = max(angle_exp,[],2);
    % Load the RDC3 data
    for rr = 1:RangeBins
        for dd = 1:DopplerBins
            if isComplexRDC3
                ang_vec = fread(f1, 2*AngleBins, 'int16');
                ang_vec = complex(ang_vec(1:2:end),ang_vec(2:2:end));
                angle_rdc(rr, :, dd) = abs(ang_vec) .* (2^angle_exp(rr, dd)); % just take abs of complex data for now
            else
                ang_vec = fread(f1, AngleBins, 'uint16');
                angle_rdc(rr, :, dd) = ang_vec .* (2^angle_exp(rr, dd));
            end
        end
    end
    fclose(f1);
    
    %figure;
    %mesh(squeeze(angle_rdc(:,1,:)));
    
    %figure;
    %mesh(angle_exp);
    
    % ------------
    if plotson>0
        a=figure;
        set(a,'Position',[50 400 1200 400])
        
        histdata=mag2db(angle_rdc(angle_rdc>0));
        
        if 0
            subplot(1,2,1)
            hist(histdata,100)
            title('Histogram of nonzero RDC3 magnitude (dB)')
            ylabel 'Hits'
            xlabel 'Magnitude (dB)'
            
            %minval = max(minval, 2^min(max_angle_exp)); % set minval to min exponent
            angle_rdc(abs(angle_rdc)<minval) = minval;
            
            subplot(1,2,2)
        end
        
        tooMuchdB = 150;
        deltadB = max(histdata)-min(histdata) ;
        
        rdc3p = mag2db(squeeze(angle_rdc(:,:,doppCut)));
        view([120 20 ])
        
        %surf(rdc3p,'LineWidth',2,'EdgeColor','none');
        mesh(rdc3p);
        
        title(['RDC3 data, 1-based Doppler bin ' num2str(doppCut)])
        xlabel 'Angle Bin'
        
        axis tight;
        if deltadB > tooMuchdB
            zlim([max(histdata)-deltadB max(histdata)+3]);
        end
        ylabel 'Range Bin'
    end
else
    disp(['*** Cant find RDC3 or RDC3 exponent data file ***']);
end
end
