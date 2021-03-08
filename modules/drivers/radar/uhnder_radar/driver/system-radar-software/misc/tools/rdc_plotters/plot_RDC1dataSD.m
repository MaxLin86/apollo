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
function [time_rdc, correlatorExpOut] = plot_RDC1dataSD(RDC1binFile,plotdBFS,plotVrx)%,RDC1expFile,RangeBins,Nvrx,Npris,plotVrx,pulsedMode,combinedLoadRDCPumpDat,dBFScorr_lin.RDC1)

global sp_params

makeRDC1gif=0;

if ~exist('plotVrx','var')
    plotVrx = 1:96;
end
if ~exist('plotdBFS','var')
    plotdBFS = 0;
end

if ~exist('loadjson','file')
    addpath('../jsonlab/');
end

% file parts
[fp1, fp2, fp3] = fileparts(RDC1binFile);
RDC1expFile = [fp1 '/' fp2 'exp' fp3]; % HW exponents per Rbin

% Load Scan Info Json
SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_rdc1')) 'info.json'];
[PMCW, antenna, detection_params, dBFScorr_lin, swExponent, ~] = processSabineScanInfo(SDjsonfilepath);

minval = db2mag(10);

% data size check
a1=dir(RDC1binFile);
assert(a1.bytes>=2*(2*antenna.NVrx*PMCW.R*PMCW.N),'RDC1 Bin file size is not correct') 
a2=dir(RDC1expFile);
assert(a2.bytes == 16*PMCW.R,'RDC1 Exponent file size is not correct') 

% File load from binary
f = fopen(RDC1binFile, 'r');
if f>0
    time_rdc = zeros(PMCW.R,antenna.NVrx,PMCW.N);
    
    disp('Loading RDC1 data and exponents')
    nri = fread(f, 2*antenna.NVrx*PMCW.R*PMCW.N, 'int16');
    time_rdc = permute(reshape(complex(nri(1:2:end), nri(2:2:end)),[antenna.NVrx PMCW.R PMCW.N]),[2 1 3]);
    fclose(f);
    a=ver;
    if any(strcmp({a.Name},'MATLAB'))
        [time_rdc, correlatorExponentsMx] = calcCorrExpFromMx(RDC1expFile,time_rdc);
    else
        [time_rdc, correlatorExponentsMx] = calcCorrExpFromMxOctave(RDC1expFile,time_rdc);
    end
    
    % Reorder Range bins
    validrbinmask = detection_params.rangeGatesMid>=0;
    % also apply SW exp
    time_rdc = time_rdc(detection_params.rangeGatesOrdering(validrbinmask),:,:)*2^swExponent.RDC1;
    
    correlatorExpOut = log2(max(correlatorExponentsMx,[],2));
    correlatorExpOut = correlatorExpOut(detection_params.rangeGatesOrdering);
    correlatorExpOut = correlatorExpOut(validrbinmask);
    dBFScorr_lin.RDC1 = dBFScorr_lin.RDC1(validrbinmask);
    
%     if sp_params.pulsedMode
%         [time_rdc, ~, ~, ~] = reorderPulsedTimeRdc(PMCW.R,sp_params,time_rdc);
%     end
%%%% Plot RDC1
if(plotVrx>0)
    Fig1=figure;
    dcm_obj = datacursormode(Fig1);
    set(dcm_obj,'UpdateFcn',@RDC1updatefcn)
    maxall=max(mag2db(abs(double(time_rdc(:)))));
    for itplot=1:numel(plotVrx)
        if plotVrx(itplot)<= antenna.NVrx
            view([90 6])
            RDC1plot = abs(double(time_rdc(:,plotVrx(itplot),:)));
            RDC1plot(abs(RDC1plot)<minval) = minval;
            if plotdBFS
                toplot=mag2db(squeeze(RDC1plot)./repmat(dBFScorr_lin.RDC1,1,size(RDC1plot,3)));
                mesh(toplot);
                %figure,surf(mag2db(squeeze(RDC1plot)),'LineStyle','none')
                title(['Magnitude dBFS RDC1 data, VRx:' num2str(plotVrx(itplot))])
                zlabel 'Magnitude dBFS'
            else
                toplot=mag2db(squeeze(RDC1plot));
                mesh(toplot)
                %figure,surf(mag2db(squeeze(RDC1plot)),'LineStyle','none')
                title(['Magnitude RDC1 data, VRx:' num2str(plotVrx(itplot))])
                zlabel 'Magnitude dB'
            end
            try
                caxis([median(toplot(:)) max(toplot(:))-3]);
            end
            colormap jet;
            xlabel 'Pulses'
            ylabel 'Range Bin'
            view([45 35]);
            axis tight;
            %             zlim([50 maxall]);
            %             caxis([50 maxall]);
            pause(.1)
            if makeRDC1gif
                frame  = getframe(Fig1);
                im = frame2im(frame);
                drawnow
                [imind,cm] = rgb2ind(im,256);
                
                if  itplot == 1;
                    imwrite(imind,cm,[fp1 '/' fp2 '_RDC1Vrxs.gif'],'gif', 'Loopcount',inf);
                else
                    imwrite(imind,cm,[fp1 '/' fp2 '_RDC1Vrxs.gif'],'gif','WriteMode','append');
                end
                drawnow
                pause(.05)
            end
        end
        end
    end
    
else
    disp(['*** Cant find RDC1 or exp data file ***']);
end
end
