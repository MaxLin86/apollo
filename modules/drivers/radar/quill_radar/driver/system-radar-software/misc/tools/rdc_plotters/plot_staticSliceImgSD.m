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
function [staticSlicesOut, interpMagRng, interpBinRng, interpMagAng, interpBinAng ] = plot_staticSliceImgSD(...
    SSbinFile, ... % static slice bin file, other files will be read using the root of the filename
    xlimin,... xlim input leave empty for no range censoring
    binAxes,... plots axes in meters and degrees if 0, bins otherwise
    plotdBFS,... enables dBFS scaling to magnitude
    showPlots,... enables plotting, setting to 2 saves a png
    plotRbinCut,...one based Rbin for AngleMagnitude plot
    polarplot,...static image to polar conversion only works with valid scan json ranges and angles
    polarplotrng,...range to plot out in the polar plot version
    doPrintInterpPeaks,... turn on interpolated peak detector
    dBpeakThresh,...  peak print thresh, relative to max
    peakCensorRngAzEl,... min/max, units are in bins if binAxes==1 despite var names [rangemin_m,rangemax_m,azmin_deg,azmax_deg,elmin_deg,elmax_deg]
    rangeAdjustm,... fudge factor to range align plots, subtracted off all rngbins
    dopbin, ... doppler bin
    noNewFigs,... suppress new figs if 1
    plotSliceDopBins,... %Plot the static slice Angle/Doppler plot if 1
    imageUnderlay,... % plot an image under
    plot2DBscope,... % plots the static slice in a cartestian bscope view alongside the rbincut view
    rbin_offset,...
    rbin_width,...
    dbin_offset,...
    dbin_width...
    )
% plots static slice magnitude data for the given
%
% example:
% plot_staticSliceImgSD('X:\software\jpb\data_2017_09_08_outdoors_at_JCCA\_NEW_2D_angle_accuracy\X0_0_Y0_0_Z0_0\scan_000000_stslice.bin',[],0,0,1,24:26,1,30,1,-5);% For complex data the static slice binary data file name stSlData should
% end in "_stslicec.bin"

% example with image overlay
% plot_staticSliceImgSD('X:\software\jonathan\2DwithImg\scan_000143_stslice.bin',[],0,2,1,10:12,0,0,0,0,[],0,[],0,0,'X:\software\jonathan\2DwithImg\Untitled.png');

% ts = 'W:\software\andyz\New test data from SCC\192.168.95.20'; for itsc=0:47, plot_staticSliceImgSD([ts '/scan_' num2str(itsc,'%06d') '_stslice.bin']); end

% setting showPlots = 2 will save a PNG file to the source data directory!

% always load SS exponents
global lastRbinmax

loadStaticSliceExponents = 1;
sigmoidColormap = 0;

%string to number conversion needed for executable input parameters 
if ~exist('showPlots','var')
    showPlots = 1;
end
if ischar(showPlots)
    showPlots=str2num(showPlots);
end
if ~exist('plotdBFS','var')
    plotdBFS = 0;
end
if ischar(plotdBFS)
    plotdBFS=str2num(plotdBFS);
end
if ~exist('binAxes','var')
    binAxes = 1;
end
if ischar(binAxes)
    binAxes=str2num(binAxes);
end
% 2dMIMO Rbin cut supported
if ~exist('plotRbinCut','var')
    plotRbinCut = 0;
end
if ischar(plotRbinCut)
    plotRbinCut=str2num(plotRbinCut);
end
if ~exist('polarplot','var')
    polarplot=0;
end
if ischar(polarplot)
    polarplot=str2num(polarplot);
end
if ~exist('polarplotrng','var')
    polarplotrng = 300;
end
if ischar(polarplotrng)
    polarplotrng=str2num(polarplotrng);
end
if ~exist('xlimin','var')
    xlimin = [];
end
if ~exist('doPrintInterpPeaks','var')
    doPrintInterpPeaks=0;
end
if ischar(doPrintInterpPeaks)
    doPrintInterpPeaks=str2num(doPrintInterpPeaks)
end
if ~exist('dBpeakThresh','var')
    dBpeakThresh = -5;
end
if ischar(dBpeakThresh)
    dBpeakThresh=str2num(dBpeakThresh);
end
if ~exist('peakCensorRngAzEl','var') || isempty(peakCensorRngAzEl)
    peakCensorRngAzEl = [0 999 -180 180 -180 180];
end
if ~exist('rangeAdjustm','var')
    rangeAdjustm = 0;
end
if ischar(rangeAdjustm)
    rangeAdjustm=str2num(rangeAdjustm);
end
if ~exist('noNewFigs','var')
    noNewFigs = 0;
end
if ischar(noNewFigs)
    noNewFigs=str2num(noNewFigs);
end
if ~exist('plotSliceDopBins','var')
    plotSliceDopBins = 0;
end
if ischar(plotSliceDopBins)
    plotSliceDopBins=str2num(plotSliceDopBins);
end
if ~exist('imageUnderlay','var')
    imageUnderlay = [];
end
if ~exist('plot2DBscope','var')
    plot2DBscope = 0;
end
if ischar(plot2DBscope)
    plot2DBscope=str2num(plot2DBscope);
end
if ~exist('rbin_offset','var')
    rbin_offset = 0;
end
if ischar(rbin_offset)
    rbin_offset=str2num(rbin_offset);
end
if ~exist('rbin_width','var')
    rbin_width = 1;
end
if ischar(rbin_width)
     rbin_width=str2num(rbin_width);
end
if ~exist('dbin_offset','var')
    dbin_offset = 0;
end
if ischar(dbin_offset)
    dbin_offset=str2num(dbin_offset);
end
if ~exist('dbin_width','var')
    dbin_width = 1;
end
if ischar(dbin_width)
    dbin_width=str2num(dbin_width);
end
% if plotting the 2D bscope view the enable polarplot
if plot2DBscope
    if ischar(plot2DBscope)
        plot2DBscope=str2num(plot2DBscope);
    end
    polarplot = 1;
end

interpMagRng = [];
interpBinRng = [];
interpMagAng = [];
interpBinAng = [];

heightSurface = 1;
heighSurfThreshdB = dBpeakThresh;

if ~exist('loadjson','file')
    addpath('../jsonlab/');
end

% file parts
[fp1, fp2, fp3] = fileparts(SSbinFile);

stSlData = [fp1 '/' fp2 fp3];
stSlDopBins = [fp1 '/' fp2 '_bins' fp3];


% Load Scan Info Json
SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_stslice')) 'info.json'];
[PMCW, antenna, detection_params, dBFScorr_lin, swExponent, ~] = processSabineScanInfo(SDjsonfilepath);

%CONVERT TO DEGREES UP FRONT
detection_params.angleGatesMid = rad2deg(detection_params.angleGatesMid);

if plotdBFS==2
    dBFScorr_lin.RDC1 = dBFScorr_lin.RDC1max;
    dBFScorr_lin.RDC2 = dBFScorr_lin.RDC2max;
    dBFScorr_lin.RDC3 = dBFScorr_lin.RDC3max;
end

if plotdBFS
    minval = db2mag(-250);
else
    minval = db2mag(10);
end

% complex RDC3 check
isComplexRDC3 = 0;
if strcmp(stSlData(end-12:end-4),'_stslicec')
    isComplexRDC3 = 1;
end


% calculate the SS width from input data sizes. warn if halfwidth is inconsistent
a1=dir(stSlData);
a2=dir(stSlDopBins);
numRbins=PMCW.R;
rbinfo=8;
if isempty(a2)
    warning('No Static Slice Doppler Bin Voxel data provided! Assuming Zero Doppler bin');
else
    tempmod = mod(a2.bytes/2,4);
    if tempmod ==0, tempmod=4; end
    padbytes=2*(4-tempmod);
    SSwidth=(a1.bytes-((rbinfo+padbytes)*numRbins))/(a2.bytes)/numRbins;
    if(isComplexRDC3)
      SSwidth = ceil(SSwidth);
      SSwidth = SSwidth/2;
    end
    if(PMCW.stslice.dbins ~= SSwidth)
        warning(['Static slice width derived from data file sizes (' num2str(SSwidth) ') does not equal input :' num2str(PMCW.stslice.dbins) '!!'])
        warning(['Static slice width derived from data file sizes (' num2str(SSwidth) ') does not equal input :' num2str(PMCW.stslice.dbins) '!!'])
        warning(['Static slice width derived from data file sizes (' num2str(SSwidth) ') does not equal input :' num2str(PMCW.stslice.dbins) '!!'])
        warning(['Static slice width derived from data file sizes (' num2str(SSwidth) ') does not equal input :' num2str(PMCW.stslice.dbins) '!!'])
        warning(['Static slice width derived from data file sizes (' num2str(SSwidth) ') does not equal input :' num2str(PMCW.stslice.dbins) '!!'])
    end
end

f1 = fopen(stSlData, 'r');
f2 = fopen(stSlDopBins, 'r');
if f1>0
    % disp('Loading static slice data')
    staticSlicesOut = zeros(PMCW.stslice.rbins,PMCW.stslice.abins,PMCW.stslice.dbins);
    % internally staticSlicesOut is rngbins x angbins x ss_dbins
    for ii = 1:PMCW.stslice.rbins
        for jj = 1:PMCW.stslice.dbins % 1/13 updated for Sabine
            if isComplexRDC3
                ang_vec = fread(f1, 2*PMCW.stslice.abins, 'int16');
                staticSlicesOut(ii,:,jj) = complex(ang_vec(1:2:end),ang_vec(2:2:end));
            else
                staticSlicesOut(ii,:,jj) = fread(f1, PMCW.stslice.abins, 'uint16');
            end
        end
        if mod(size(staticSlicesOut,2),4)>0 % read extra dont cares if not modulo of 4AB
            for ipad = 1:4-mod(size(staticSlicesOut,2),4)
                fread(f1, 1, 'uint16');
            end
        end
        if loadStaticSliceExponents
            ssRDC2exp(ii) = fread(f1, 1, 'int8'); % exponent for SS rbin, not sure where to use this yet
            fread(f1, 1, 'uint8'); % pad
            checkrange = fread(f1, 1, 'uint16'); % RB index
            %assert(checkrange==ii-1,'Range bin check failed')
            fread(f1, 1, 'uint32'); % pad
        end
    end
    fclose(f1);
else
    disp('Static Slice Data not found');
end

dbin_width = size(staticSlicesOut,3);
%default dopbin to all
if ~exist('dopbin','var') || isempty(dopbin)
    middopbin = floor(size(staticSlicesOut,3) / 2 + 1);
    middopbin = middopbin - dbin_offset;
    middopbin = max(1,middopbin);
    dbinA = middopbin - floor(dbin_width/2);
    dbinB = dbinA + dbin_width - 1;
    dbinA = max(1,dbinA);
    dbinB = min(size(staticSlicesOut,3), dbinB);
    dopbin = dbinA:dbinB;
else
    dbinA = min(dopBin);
    dbinB = max(dopBin);
end

if f2>0
    for jj = 1:PMCW.stslice.abins % 1/13 SEU Voxel update, only read center bin
        tempSScenter = 1 + fread(f2, 1, 'uint16');
        % derive other SS dopp bin indices
        sliceBins=tempSScenter - ((PMCW.stslice.dbins-1)/2):tempSScenter+((PMCW.stslice.dbins-1)/2);
        % correct for edge case
        sliceBins = mod(sliceBins,PMCW.N);
        sliceBins(sliceBins==0) = PMCW.N;
        staticSliceBinIndices(jj,:) = sliceBins ;
    end
    fclose(f2);
else
    disp('Not reading static slice doppler bin voxel info');
end


% apply exponent if available, before reorder!
if(loadStaticSliceExponents)
    staticSlicesOut = staticSlicesOut .* repmat((2.^ssRDC2exp).',[ 1 PMCW.stslice.abins PMCW.stslice.dbins ]);
end
staticSlicesOut = abs(staticSlicesOut);

% % Range Bin reordering for pulsed mode
% Reorder Range bins now easier based on scan data
staticSlicesOut = staticSlicesOut(detection_params.rangeGatesOrdering,:,:);
validrbinmask = detection_params.rangeGatesMid>=0;
staticSlicesOut = staticSlicesOut(validrbinmask,:,:);

% rng correction
detection_params.rangeGatesMid(detection_params.rangeGatesMid>=0) = max(detection_params.rangeGatesMid(detection_params.rangeGatesMid>=0) - rangeAdjustm,0);

% apply SW exp
staticSlicesOut = staticSlicesOut.*2^swExponent.RDC3;

if plotdBFS
    foo = dBFScorr_lin.RDC3(validrbinmask);
    if ndims(staticSlicesOut)==3
        staticSlicesOut=staticSlicesOut./repmat(foo,1,size(staticSlicesOut,2),size(staticSlicesOut,3));
    else
        staticSlicesOut=staticSlicesOut./repmat(foo,1,size(staticSlicesOut,2));
    end
end

% initialize
maxStSlice = zeros(size(staticSlicesOut,2),size(staticSlicesOut,1));
for i = 1:size(staticSlicesOut,2)
    %    [M,I] = max(squeeze(staticSlicesOut(:,i,:)),[],2);
    [M,I] = max(squeeze(staticSlicesOut(:,i,dopbin)),[],2);
    maxStSlice(i,:) = M;      %Max Values Static Slice/ Intensity Image
    mpIndex(i,:) = I;  %Frequency Image
end
% set minval to prevent -Inf plotting
maxStSlice = max(maxStSlice,minval);

if showPlots>0
    if plotSliceDopBins
        if binAxes
            figure, plot(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2))), xlabel('Angle Bin (1b)'), ylabel('Doppler Bin (1b)');
        else
            if PMCW.MIMO2D
                figure, plot3(detection_params.angleGatesMid(:,1),detection_params.angleGatesMid(:,2),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2))),'.-'), xlabel('Az Angle (deg)'),ylabel('El Angle (deg)'), zlabel('Velocity (mps)');
            else
                figure, plot(detection_params.angleGatesMid(:,1),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2)))), xlabel('Az Angle (deg)'), ylabel('Velocity (mps)');
            end
        end
    end
    
    
    
    toplot=maxStSlice;
    if doPrintInterpPeaks
        % handle angle bins on the 2d mimo case
        if PMCW.MIMO2D
            useAngbins = unique(detection_params.angleGatesMid(:,1));
            if ndims(staticSlicesOut)==3
                useSSforInterp = max(abs(staticSlicesOut),[],3);
                useSSforInterp = abs(max(reshape(useSSforInterp,[],16,8),[],3));
            else
                useSSforInterp = abs(max(reshape(staticSlicesOut,[],16,8),[],3));
            end
        else
            useAngbins = detection_params.angleGatesMid(:,1);
            useSSforInterp = abs(staticSlicesOut);
        end
        if binAxes
            [peaksStr, interpMagRng, interpBinRng, interpMagAng, interpBinAng] = printInterpPeaks(useSSforInterp, dBpeakThresh, peakCensorRngAzEl);
        else
            [peaksStr, interpMagRng, interpBinRng, interpMagAng, interpBinAng] = printInterpPeaks(useSSforInterp, dBpeakThresh, peakCensorRngAzEl, detection_params.rangeGatesMid, useAngbins);
        end
    end
    
    toplot = mag2db(toplot);
    
    if any(plotRbinCut)
        titledat.scanNumString= fp2(6:11);
        titledat.dbinA = dbinA;
        titledat.dbinB = dbinB;
        clear interpMagAng interpBinAng; % clear 1d angle out
        [ interpBinRng, interpMagRng, interpMagAng, interpBinAng ] = makeRbincutPlot(maxStSlice,...
            titledat,plotRbinCut,rbin_offset,rbin_width,...
            PMCW,detection_params,plotdBFS,dBFScorr_lin,swExponent,binAxes,noNewFigs,plot2DBscope,polarplot,polarplotrng,imageUnderlay,...
            doPrintInterpPeaks,dBpeakThresh,peakCensorRngAzEl);
        
        
        %         set(gcf,'Position',[362        1062        1192         518]);
    else
        
        
        if(~noNewFigs)
            Fig1=figure;
        else
            Fig1=gcf;
        end
        
        if doPrintInterpPeaks && (polarplot && plot2DBscope)
            maxsub=6;
        else
            maxsub=5;
        end
        if doPrintInterpPeaks % write peaks to screen
            
            subplot(1,maxsub,maxsub)
            axis off;
            % Create a uicontrol of type "text"
            mTextBox = uicontrol('style','text')
            set(mTextBox,'String',peaksStr)
            % To move the the Text Box around you can set and get the position of Text
            mTextBoxPosition = get(mTextBox,'Position')
            
            % The array mTextBoxPosition has four elements
            % [x y length height]
            % Something that I find useful is to set the Position Units to Characters,
            set(mTextBox,'Units','normalized')
            set(mTextBox,'Position',[.75 .4 .25 .5]);
            title({'Interpolated Peaks '; [num2str(dBpeakThresh) 'dB Relative to Max']})
            if (polarplot && plot2DBscope)
                subplot(1,maxsub,1:maxsub-3);
            else
                subplot(1,maxsub,1:maxsub-1);
            end
        end
        % This means a Text Box with 3 lines of text will have a height of 3
        if binAxes
            % RevB has issue with last Range bin. Choose the RB scope here
            % to avoid this
            %surf(toplot(:,1:255),'FaceColor','interp','EdgeColor','none');
            surf(toplot(:,:),'FaceColor','interp','EdgeColor','none');
        else
            surf(detection_params.rangeGatesMid(validrbinmask),detection_params.angleGatesMid(:,1),toplot,'FaceColor','interp','EdgeColor','none');
        end
        dcm_obj = datacursormode(Fig1);
        if binAxes
            set(dcm_obj,'UpdateFcn',@RDC3updatefcn)
        else
            set(dcm_obj,'UpdateFcn',@RDC3updatefcnm)
        end
        axis tight;
        colormap jet;
        %caxis([median(toplot(:)) max(toplot(:))-3]);
        if sigmoidColormap
            cin = colormap;
            steepness = 1;
            midpoint = .5;
            colormap(applySigmoid(cin,steepness,midpoint));
        end
        
        %     ylim(plotRbinMinMax);
        if binAxes
            xlabel('Range Bin');
            ylabel('Angle Bin');
        else
            xlabel('Range (m)');
            ylabel('Angle (deg)');
        end
        if plotdBFS
            zlabel('Mag dBFS');
        else
            zlabel('Mag dB');
        end
        if(~isempty(xlimin))
            currxlim=xlim;
            if min(xlimin)>max(currxlim)
                disp('Requested xlim is out of scope, skipping. Check range units, bins vs meters!');
            else
                xlim(xlimin);
            end
        end
        %zlim([median(toplot(:))-20 max(toplot(:))+3]);
        title(['Scan ' fp2(6:11) ' Static Slice Data Magnitude, Max over ' num2str(PMCW.stslice.dbins) ' Static Slice dopbins']);
        set(gca, 'YDir', 'reverse')
        curpos=get(gcf,'Position');5
        set(gcf,'Position',curpos-[345   425  -760  -385]);
        
        % polar plot
        if polarplot && (~PMCW.MIMO2D || (PMCW.MIMO2D && plot2DBscope))
            ringdat=[];
            makePolarplotFromRA(maxStSlice,plotRbinCut,maxsub,ringdat,...
                PMCW,detection_params,plotdBFS,dBFScorr_lin,swExponent,noNewFigs,plot2DBscope,polarplot,polarplotrng);
        end
        
    end
    if showPlots==2
        saveas(gcf,[fp1 '/' fp2], 'fig');
    end
    
end

end


