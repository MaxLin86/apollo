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
function [ interpBinRng, interpMagRng, interpMagAng, interpBinAng ] = makeRbincutPlot(maxRbinInputData,...
    rBinCutData,plotRbinCut,rbin_offset,rbin_width,...
    PMCW,detection_params,plotdBFS,dBFScorr_lin,swExponent,binAxes,noNewFigs,plot2DBscope,polarplot,polarplotrng,imageUnderlay,...
    doPrintInterpPeaks,dBpeakThresh,peakCensorRngAzEl)

global lastRbinmax
global meanbins

if(~noNewFigs), figure; end
if numel(plotRbinCut)>0
    % overwrite rbincut
    if ~isempty(lastRbinmax) % lastRbinmax==-1 is max search mode
        yblank=rBinCutData.yblank; % meters ignore maxes outside of this y
        tmaxRbinInputData = maxRbinInputData;
%         [~, minbin]=(min(abs(detection_params.rangeGatesMid-105))); % hardcode max start bin, ignore for  (lastRbinmax==-1  && itr < minbin)
        for itr=1:size(maxRbinInputData,2)
            angmask = abs(detection_params.rangeGatesMid(itr)*sind(detection_params.angleGatesMid(:,1)))>yblank;
            tmaxRbinInputData(angmask,itr)=0;
            if lastRbinmax==-1 || rBinCutData.trackMax && lastRbinmax>0 && (itr>lastRbinmax+rBinCutData.trackMaxdeltam || itr<lastRbinmax-rBinCutData.trackMaxdeltam)
                tmaxRbinInputData(:,itr)=0;
            end
        end
        if numel(plotRbinCut) == 1
            rmax=ones(size(maxRbinInputData,1),1);
            toplot = maxRbinInputData(:,plotRbinCut);
            
        else
            [toplot,rmax]=max(tmaxRbinInputData(:,plotRbinCut),[],2);
        end
        clear tmaxStSlice;
        [tmpval,maxangle] = max(toplot);
        midRbin = rmax(maxangle);
        if midRbin<lastRbinmax || lastRbinmax<0
            if isempty(meanbins)
                meanbins =4;
            else
                meanbins = ceil((meanbins + abs(lastRbinmax - midRbin))/2);
            end
            lastRbinmax = midRbin;
            
        else
            lastRbinmax = lastRbinmax-meanbins;
        end
        binA = midRbin + rbin_offset - floor(rbin_width/2);
        binB = binA + rbin_width - 1;
        binA = max(binA,1);
        binA = min(binA,size(maxRbinInputData,2));
        binB = max(binB,1);
        binB = min(binB,size(maxRbinInputData,2));
        plotRbinCut = [ binA : binB ];
    else
        binA = plotRbinCut(1); % lastRbinmax==[] is user defined mode
        binB = plotRbinCut(end);
    end
    [toplot,rmax]=max(maxRbinInputData(:,plotRbinCut),[],2);
else
    rmax=ones(size(maxRbinInputData,1),1);
    toplot = maxRbinInputData(:,plotRbinCut);
end

if plotdBFS
    foo = dBFScorr_lin.RDC3(plotRbinCut(rmax));
    toplot=toplot./foo.*2^0;% dont trust CI exp (swExponent.CI);
end

if PMCW.MIMO2D
    allAz= unique(detection_params.angleGatesMid(:,1));
    allEl= unique(detection_params.angleGatesMid(:,2));
    azLen = length(allAz);
    elLen = length(allEl);
    toplot = reshape(toplot,azLen,elLen);
    if doPrintInterpPeaks
        if binAxes
            % bins are one too big will be subtracted later
            rngMaxes = reshape(plotRbinCut(rmax),16,8);
            [azstr, rngMax, rngMag, interpMagAz, interpBinAz, interpMagEl, interpBinEl] =  printInterpPeaksAzEl(toplot, dBpeakThresh,peakCensorRngAzEl,rngMaxes);
        else
            rngMaxes = detection_params.rangeGatesMid(reshape(plotRbinCut(rmax),azLen,elLen));
            [azstr, rngMax, rngMag, interpMagAz, interpBinAz, interpMagEl, interpBinEl] =  printInterpPeaksAzEl(toplot, dBpeakThresh,peakCensorRngAzEl,rngMaxes,detection_params.angleGatesMid(:,1), detection_params.angleGatesMid(:,2));
        end
        
        
        %                     % get range interp peaks
        %                     rngMaxestemp = reshape(plotRbinCut(rmax),16,8);
        %                     [~, rngtempAE, ~, ~, interpBinAztemp, ~, interpBinEltemp] =  printInterpPeaksAzEl(toplot, dBpeakThresh,peakCensorRngAzEl,rngMaxestemp);
        %                     for itrng = 1:length(rngtempAE)
        %
        %                     try
        %                         ttp = maxStSlice;%./dBFScorr_lin.RDC3(plotRbinCut(rmax)).*2^swExponent.RDC3;
        %                         rBp = rngMax(itrng);
        %                         aBp = round(interpBinAztemp(itrng))-1;
        %                         eBp = round(interpBinEltemp(itrng))-1;
        %                         [inda]=sub2ind(size(toplot),aBp,eBp);
        %
        %                         [ rngMag(itrng), rngMax(itrng) ] = getInterpPeak( [ ttp(inda,rBp-1) ttp(inda,rBp) ttp(inda,rBp+1) ], detection_params.rangeGatesMid([ rBp-1 rBp rBp+1]) );
        %
        %                     catch
        %
        %                     end
        %
        %                     rngMaxes = detection_params.rangeGatesMid(reshape(plotRbinCut(rmax),16,8));
        %                     [azstr, rngMax, rngMag, interpMagAz, interpBinAz, interpMagEl, interpBinEl] =  printInterpPeaksAzEl(toplot, dBpeakThresh,peakCensorRngAzEl,rngMaxes,detection_params.angleGatesMid(:,1), detection_params.angleGatesMid(:,2));
        
        
        %overwrite with 2D interp data
        clear interpMagAng;
        clear interpBinAng;
        interpBinRng = rngMax;
        interpMagRng = rngMag;
        %you must subtract 1 of bins later
        interpMagAng.Az = interpMagAz;
        interpMagAng.El = interpMagEl;
        interpBinAng.Az = interpBinAz;
        interpBinAng.El = interpBinEl;
    else
        interpBinRng = [];
        interpMagRng = [];
        interpMagAng = [];
        interpBinAng =  [];
    end
end
toplot = mag2db(toplot);

if(~noNewFigs), figure; end
if PMCW.MIMO2D
    if doPrintInterpPeaks && (polarplot && plot2DBscope)
        maxsub=6;
    else
        maxsub=5;
    end
    if doPrintInterpPeaks % write peaks to screen
        
        subplot(1,maxsub,maxsub)
        axis off;
        % Create a uicontrol of type "text"
        mTextBox = uicontrol('style','text');
        set(mTextBox,'String',azstr);
        % To move the the Text Box around you can set and get the position of Text
        mTextBoxPosition = get(mTextBox,'Position');
        
        % The array mTextBoxPosition has four elements
        % [x y length height]
        % Something that I find useful is to set the Position Units to Characters,
        set(mTextBox,'Units','normalized')
        set(mTextBox,'Position',[.79 .0 .13 .9]);
        title({'Interpolated Peaks '; [num2str(dBpeakThresh) 'dB Relative to Max']})
        subplot(1,maxsub,1:maxsub-1)
    end
    if (polarplot && plot2DBscope)
        subplot(1,maxsub,1:3)
    end
    
    if binAxes
        if ~isempty(imageUnderlay)
            temp=imread(imageUnderlay);
            imagesc(1:size(toplot,1),1:size(toplot,2),temp), axis equal
            hold
            axis tight
            toplot=toplot+300;
            surf(toplot.','FaceColor','interp','EdgeColor','none','FaceAlpha',0.5);
        else
            surf(toplot,'FaceColor','interp','EdgeColor','none');
        end
    else
        if ~isempty(imageUnderlay)
            temp=imread(imageUnderlay);
            imagesc(allAz,allEl,temp), axis equal
            hold
            axis tight
            toplot=toplot+300;
            surf(allAz,allEl,toplot.','FaceColor','interp','EdgeColor','none','FaceAlpha',0.5);
        else
            surf(allEl,allAz,toplot,'FaceColor','interp','EdgeColor','none');
        end
    end
    axis tight;
    colormap jet;
    shading interp;
    %     try
    caxis([median(toplot(:))-10 max(toplot(:))]);
    %     catch
    %         disp('caxis out of bounds');
    %         caxis([median(toplot(:)) max(toplot(:))]);
    %     end
    %     caxis([145,153]);
    %     ylim(plotRbinMinMax);

    if binAxes
        xlabel('Elevation Bin');
        ylabel('Azimuth Bin');
    else
        xlabel('Elevation (deg)');
        ylabel('Azimuth (deg)');
    end
    if plotdBFS
        zlabel('Mag dBFS');
    else
        zlabel('Mag dB');
    end
    if isempty(imageUnderlay)
        zlim([median(toplot(:))-40 max(toplot(:))+3]);
        set(gca, 'YDir', 'reverse')
        set(gca, 'XDir', 'reverse')
        view([-90,90]); % axis tight;
    end
else
    if binAxes
        plot(toplot)
        xlabel('Angle bin');
    else
        plot(detection_params.angleGatesMid(:,1),toplot);
        xlabel('Azimuth (deg)');
    end
    
    if plotdBFS
        ylabel('Mag dBFS');
    else
        ylabel('Mag dB');
    end
end
if ~isempty(lastRbinmax)
    title({['Scan' rBinCutData.scanNumString ',  ' ...
        't=' num2str(round(str2num(rBinCutData.scanNumString)*.1311131,2)) ' sec  ' ...
        'Rmax= ' num2str(midRbin) ' (' num2str(detection_params.rangeGatesMid(midRbin),'%0.1f') ' m  )    ' ...
        'R= ' num2str(binA) '...' num2str(binB)  ' (' num2str(detection_params.rangeGatesMid(binA),'%0.1f') '...' num2str(detection_params.rangeGatesMid(binB),'%0.1f') ' m  )   '];...
        rBinCutData.maxstr})
else
    title(['Scan' rBinCutData.scanNumString ', 2D Az/El Slice Mag, time: ' num2str(round(str2num(rBinCutData.scanNumString)*.1311131,2)) ' seconds']);
end
axis equal;

ringdat.binA = binA;
ringdat.binB = binB;
if polarplot && (~PMCW.MIMO2D || (PMCW.MIMO2D && plot2DBscope))
    % plsub(1) tot, plsub(2) min, plsub(3) max
    plsub(1) = maxsub;
    if(doPrintInterpPeaks)
        plsub(2) = 4;
        plsub(3) = 5;
    else
        plsub(2) = 4;
        plsub(3) = 5;
    end
    
    makePolarplotFromRA(maxRbinInputData,plotRbinCut,plsub,ringdat,...
        PMCW,detection_params,plotdBFS,dBFScorr_lin,swExponent,noNewFigs,plot2DBscope,polarplot,polarplotrng)
    if (polarplot && plot2DBscope)
        [cax1 cax2] = caxis;
        subplot(1,maxsub,1:3)
        caxis([cax1 cax2]);                  % CAXIS for BSCOPE
    end
    
    
end

end