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
