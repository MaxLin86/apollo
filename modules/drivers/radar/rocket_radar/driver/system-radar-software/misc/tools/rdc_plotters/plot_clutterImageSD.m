% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [clutImgOut, interpMagRng, interpBinRng, interpMagAng, interpBinAng ] = plot_clutterImageSD(...
    filestr, ...
    binAxes,... plots axes in meters and degrees if 0, bins otherwise
    plotdBFS,... enables dBFS scaling to magnitude
    showPlots,... enables plotting, setting to 2 saves a png
    calcSNR,... calculates CI in SNR units (default) rather than mag
    plotRbinCut,...one based Rbin for AngleMagnitude plot
    polarplot, ... plot polar projection if 1, plot angle vs range if 0
    polarplotrng, ... max plot range
    doPrintInterpPeaks, ... print peak info if above thresh and plotSliceDopBins==0
    dBpeakThresh,... print peaks for all peaks this thresh relative to max
    peakCensorRngAzEl,... min/max, units are in bins if binAxes==1 despite var names [rangemin_m,rangemax_m,azmin_deg,azmax_deg,elmin_deg,elmax_deg]
    rangeAdjustm,... fudge factor to range align plots, subtracted off all rngbins
    noNewFigs,... suppress new figs if 1...
    plotSliceDopBins,...  plot the ego vel slice if 1
    imageUnderlay,... % plot an image under
    plot2DBscope,... % plots the static slice in a cartestian bscope view alongside the rbincut view
    plotStaticDets, ... plot static detection squares if 1
    plotMovingDets, ... plot moving detection circles if 1
    elevationAngleAdjust_deg,... adjustment on elevation angle if not properly aligned
    rbin_offset,...
    rbin_width,...
    dbin_offset,...
    dbin_width...
    )

global lastRbinmax
interpMagRng=[];
interpBinRng=[];
interpMagAng=[];
interpBinAng=[];

% rbin cut example
% plot_clutterImageSD('C:\uhnderOLD\TESTDATA\1d-2d_alternate\scan_000089_clutterimage.bin',0,0,1,[1:50],1,10,1,0,[],0,1,0,[],1,0,0,0);

% From rdcstructs.h
%     // For Azimuth-only scans:
%     CIMG_M8C                = 1,        //                                                                          (Unsupported)
%     CIMG_M8P                = 2,        //                                                                          (Unsupported)
%     CIMG_M16C               = 3,        // 16-bit Magnitude Cartesian 2D image (X,Y)                                (OLD)
%     CIMG_M16P               = 4,        // 16-bit Magnitude Polar     2D image (R,A)
%     CIMG_M16C_D8C           = 5,        // 16-bit Magnitude Cartesian 2D image (X,Y), with Cartesian Doppler image
%     CIMG_M16C_D7P           = 6,        // 16-bit Magnitude Cartesian 2D image (X,Y), with Polar     Doppler image
%     CIMG_M16P_D7P           = 7,        // 16-bit Magnitude Polar     2D image (R,A), with Polar     Doppler image
%
%     // For Azimuth+Elevation scans:
%     CIMG_M16Z               = 21,       // 16-bit Magnitude Cartesian 3D voxel cuboid                               (OLD)
%     CIMG_M16Z_D7P           = 22,       // 16-bit Magnitude Cartesian 3D voxel cuboid, with Polar Doppler image     (Unsupported)
%     CIMG_M16C_H8C_D8C       = 23,       // 16-bit Magnitude Cartesian, with 8-bit Height and Doppler Polar images
%     CIMG_M16C_H7P_D7P       = 24,       // 16-bit Magnitude Cartesian, with 7-bit Height and Doppler Polar images   (NEW)
%     CIMG_M16P_H7P_D7P       = 25,       // 16-bit Magnitude Polar,     with 7-bit Height and Doppler Polar images
%     CIMG_M16PP_D7PP         = 26,       // 16-bit Magnitude Polar Azimuth and Polar Elevation, with 7-bit Doppler Polar/Polar image

% TODO Clean up X(m) vs Y(m) naming issues
f = fopen(filestr, 'r');
if ~exist('binAxes','var')
    binAxes = 0;
end
if ischar(binAxes)
    binAxes=str2num(binAxes);
end
if ~exist('polarplot','var')
    polarplot=1;
end
if ischar(polarplot)
    polarplot=str2num(polarplot);
end
if ~exist('polarplotrng','var')
    polarplotrng=400;
end
if ischar(polarplotrng)
    polarplotrng=str2num(polarplotrng);
end
if ~exist('plotdBFS','var')
    plotdBFS = 2;
end
if ischar(plotdBFS)
    plotdBFS=str2num(plotdBFS);
end
if ~exist('calcSNR','var')
    calcSNR = 1;
end
if ~exist('showPlots','var')
    showPlots = 1;
end
if ischar(showPlots)
    showPlots=str2num(showPlots);
end
% 2dMIMO Rbin cut supported
if ~exist('plotRbinCut','var')
    plotRbinCut = 0;
end
if ischar(plotRbinCut)
    plotRbinCut=str2num(plotRbinCut);
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
if ~exist('plotStaticDets','var')
    plotStaticDets = 1;
end
if ischar(plotStaticDets)
    plotStaticDets=str2num(plotStaticDets);
end
if ~exist('plotMovingDets','var')
    plotMovingDets = 1;
end
if ischar(plotMovingDets)
    plotMovingDets=str2num(plotMovingDets);
end
if ~exist('noNewFigs','var')
    noNewFigs = 1;
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
if ~exist('imageUnderlay','var') | imageUnderlay==0
    imageUnderlay = [];
end
if ~exist('plot2DBscope','var')
    plot2DBscope = 0;
end
if ischar(plot2DBscope)
    plot2DBscope=str2num(plot2DBscope);
end
if ~exist('elevationAngleAdjust_deg','var')
    elevationAngleAdjust_deg = 0;
end
if ischar(elevationAngleAdjust_deg)
    elevationAngleAdjust_deg=str2num(elevationAngleAdjust_deg);
end
if ~exist('rbin_offset','var')
    rbin_offset = 0;
end
if ischar(rbin_offset)
    rbin_offset=str2num(rbin_offset);
end
if ~exist('rbin_width','var')
    rbin_width = 3;
end
if ischar(rbin_width)
     rbin_width=str2num(rbin_width);
end
% if plotting the 2D bscope view the enable polarplot
if plot2DBscope
    if ischar(plot2DBscope)
        plot2DBscope=str2num(plot2DBscope);
    end
    polarplot = 1;
end

if numel(plotRbinCut)>1 && ~noNewFigs
    warning('Recommended setting for noNewFigs is 1 with rBinCut slicer, current settings will look like ass')
end

%disable dBFS scaling if SNR calc is used
if calcSNR
    plotdBFS = 0;
end

sigmoidColormap = 0;

% all sorts of hacky plotting goodness
detCircleMinMaxSize = [ 30 50 ];   % Minimum size of a moving detection indicator circle
bigDetSNR = 99;     % Minimum SNR (in dB) for drawing velocity vectors
slowDopMode = 1;    % Turn on to limit moving detection colormap to +/- 2 m/sec
rangeNorm = 1;  % Apply range normalization to make the floor constant'

combineHgtDop = -1;  % combine height/dop flag plots into 1 figure
% set to -1 to disable height plot

mitigate.SLadjSNRthresh = 9; %only censor if adj RD det is 6dB below currDet
mitigate.swamperDegToStartCensor = 24; % used by primalSidelobeDetFilter
mitigate.plotSuppressedDets = 1; % plots suppressed dets in black
mitigate.angleSL =  0; % takes out most angle sidelobe dets
mitigate.MIMOdets =  0; % mitigation for radial ride sidelobe detections
mitigate.MIMOdetsCntSat =  [5 10]; % num dets to trigger an angle bin set search, num dets to trigger set removal
mitigate.MIMOsideBinsSet = 8; % set consists on plus minus from trigger bin, 6 means 13 bins about trigger bin
mitigate.MIMOmedSNRadddB = 6; % additional dB above set's SNR median needed to keep the detection
mitigate.DCdets =  0;
mitigate.extraSSdets =  0;
CaxisRngDb = 8;

enableHeightSurface = 1; % Enable Gray sheet 2DMIMO Height plotting
heighSurfThreshdB = 15; % threshold above median for using the height data on plot
surfheight = -.5; % in 2dMIMO height image with overlay, this is the threshold for bright, elevated targets
transparentCmap = 1; %add transparency based on magnitude, must enableHeightSurface

% subplot distribution, higher number, bigger bscope
maxsubp=6;
firstsubp=5;
htImgOut = 0; %place holder in case there is no height data

rBinCutData.dbinA = 1;
rBinCutData.dbinB = 1;
rBinCutData.yblank = 10; % meters ignore maxes outside of this y
rBinCutData.trackMax = 0; % enable scan to scan max tracking
rBinCutData.trackMaxdeltam = 20; %allow max to move +/- this many range bins per scan

%% Loading
if f>0
    % file parts
    [fp1, fp2, fp3] = fileparts(filestr);
    SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'info.json'];
    [PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(SDjsonfilepath);


    %CONVERT TO DEGREES UP FRONT
    detection_params.angleGatesMid = rad2deg(detection_params.angleGatesMid);

    % rng correction
    detection_params.rangeGatesMid = max(detection_params.rangeGatesMid - rangeAdjustm,0);

    if PMCW.MIMO2D
        useSVAcolormap = 0; % 2DMIMO has no SVA, do not use tweaked color map
    else
        useSVAcolormap = 1; % 1d has SVA use tweaked color map
    end


    if plotdBFS==2
        dBFScorr_lin.RDC1(:) = max(dBFScorr_lin.RDC1);
        dBFScorr_lin.RDC2(:) = max(dBFScorr_lin.RDC2);
        dBFScorr_lin.RDC3(:) = max(dBFScorr_lin.RDC3);
    end
    %% Read Mag
    %common header
    sliceNum = fread(f, 1, 'uint16');
    ysize = fread(f, 1, 'uint16');
    xsize = fread(f, 1, 'uint16');
    depthBytes = fread(f, 1, 'uint16');
    exponent = fread(f, 1, 'int16');
    fmt = fread(f, 1, 'uint16');
    pixel_meters = fread(f, 1, 'float');

    if depthBytes==1
        datastr = 'uint8';
    elseif depthBytes==2
        datastr = 'uint16';
    else
        error('Unknown depth')
    end
    clutImg = zeros(xsize*ysize,1);

    temp = fread(f, xsize*ysize, datastr);
    clutImg(1:length(temp))= temp;
    if length(temp)~=xsize*ysize
        warning(['Clutter image file ' fp2 ' is corrupt, working with bad data']);
    end
    clutImg = ((reshape(clutImg,xsize,ysize)));

    clutImg = clutImg * 2^exponent;
    %    clutImg = clutImg(:,detection_params.rangeGatesOrdering);    % Not needed

    clutImgOut = clutImg(:,detection_params.rangeGatesMid>=0)+1;
    if calcSNR
        clutImgOut = clutImgOut./(2^swExponent.RDC3*db2mag(repmat(detection_params.rbininfo.nf_max_peak(detection_params.rbininfo.rangeorder(detection_params.rbininfo.rangeorder>=0)+1),size(clutImgOut,1),1)));
    end
    %fixme sra output CI bins dont match the map, bad bins are already shifted to end
    % prev line should be     clutImgOut = clutImg(:,detection_params.rbininfo.rangeorder>=0)+1;
    dbImg = mag2db(clutImgOut);

    fclose(f);
    if showPlots

    % formatting shorthand
    is2DMIMO = fmt>=21;
    hasHeight = fmt == 23 || fmt == 24 || fmt == 25;
    hasDoppler = fmt == 5 || fmt == 6 || fmt == 7 || fmt == 23 || fmt == 24 || fmt == 25 || fmt == 26;

    % currently only support transparent map in fmt 25, height map
    if fmt ~= 25
        transparentCmap = 0;
    end

    %% Read Height Image
    if hasHeight
        %CIMG_M16C_D7P CIMG_M16P_D7P CIMG_M16C_H7P_D7P, CIMG_M16P_H7P_D7P
        %         hdImgX = fread(f, 1, 'int32');
        %         hdImgY = fread(f, 1, 'int32');
        %         heightDopplerImg = fread(f, hdImgX*hdImgY, datastr);
        %
        f=fopen([fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'ci_height.bin'],'r');
        %common header
        sliceNumH = fread(f, 1, 'uint16');
        hImgY = fread(f, 1, 'uint16');
        hImgX = fread(f, 1, 'uint16');
        depthBytesH = fread(f, 1, 'uint16');
        exponentH = fread(f, 1, 'uint16');
        fmtH = fread(f, 1, 'uint16');
        pixel_metersH = fread(f, 1, 'float');
        if depthBytesH==1
            datastr = 'uint8';
        elseif depthBytesH==2
            datastr = 'uint16';
        else
            error('Unknown depth')
        end
        heightImg = fread(f, hImgX*hImgY, datastr);
        heightImg = reshape(heightImg,hImgX,hImgY);
        heightImg = heightImg(:,detection_params.rbininfo.rangeorder>=0);
        hImgY = hImgY - sum(detection_params.rbininfo.rangeorder<0);
        fclose(f);
    end
    if hasDoppler
        %% Read Doppler Image
        % TODO Fix doppler image
        f=fopen([fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'ci_doppler.bin'],'r');
        %common header
        sliceNumD = fread(f, 1, 'uint16');
        dImgY = fread(f, 1, 'uint16');
        dImgX = fread(f, 1, 'uint16');
        depthBytesD = fread(f, 1, 'uint16');
        exponentD = fread(f, 1, 'uint16');
        fmtD = fread(f, 1, 'uint16');
        pixel_metersD = fread(f, 1, 'float');
        if depthBytesD==1
            datastr = 'uint8';
        elseif depthBytesD==2
            datastr = 'uint16';
        else
            error('Unknown depth')
        end
        dopplerImg = fread(f, dImgX*dImgY, datastr);
        dopplerImg = reshape(dopplerImg,dImgX,dImgY);
        dopplerImg = dopplerImg(:,detection_params.rbininfo.rangeorder>=0);
        dImgY = dImgY - sum(detection_params.rbininfo.rangeorder<0);
        fclose(f);


    end

    if is2DMIMO % ToDo check which formats we have a hdpeaks file with new CI scheme
        %separate dopplerhd peaks file
        f=fopen([fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'ci_hdpeaks.bin'],'r');
        %common header
        sliceNumD = fread(f, 1, 'uint16');
        dImgY = fread(f, 1, 'uint16');
        dImgX = fread(f, 1, 'uint16');
        depthBytesD = fread(f, 1, 'uint16');
        exponentD = fread(f, 1, 'uint16');
        fmtD = fread(f, 1, 'uint16');
        pixel_metersD = fread(f, 1, 'float');
        if depthBytesD==1
            datastr = 'uint8';
        elseif depthBytesD==2
            datastr = 'uint16';
        else
            error('Unknown depth')
        end
        hdpeaks = fread(f, dImgX*dImgY, datastr);
        hdpeaks = reshape(hdpeaks,dImgX,dImgY);
        hdpeaks = hdpeaks(:,detection_params.rbininfo.rangeorder>=0);
        dImgY = dImgY - sum(detection_params.rbininfo.rangeorder<0);
        fclose(f);

    end

    %% Plotting

    % Doppler plot
    if hasDoppler  % just skip this plot if we are in no new figs mode

        dopplerImg = reshape(dopplerImg,dImgX,dImgY);
        dopplerImg = dopplerImg / 4;

        if combineHgtDop > -1 && ~noNewFigs
            if ~combineHgtDop
                if(~noNewFigs), figure; end
            else
                if(~noNewFigs), figure; end
                subplot(1,4,2);
            end
            surf(hdpeaks);
            view([90 90 ]);
            title('Multi Height/Doppler Flag Image');
            ylabel('Azimuth Bin');
            xlabel('Range Bin');
            axis tight;

            if ~combineHgtDop
                if(~noNewFigs), figure; end
            else
                subplot(1,4,1);
            end
            surf(dopplerImg,'LineStyle','none');
            %axis equal;
            view([90 90 ]);
            title('Doppler Image');
            ylabel('Azimuth Bin');
            xlabel('Range Bin');
            axis tight;
        end
    else
        disp('No doppler plot')
    end

    % Height plot
    if hasHeight

        heightImg = reshape(heightImg,hImgX,hImgY); %reshape height bin array
        heightImg = heightImg / 4; % bins have fractional part of 2 bits
        elBinsDeg = unique(detection_params.angleGatesMid(:,2)); % get elevation bins
        adjhbin = min(heightImg ,length(elBinsDeg));
        angleElevDeg = interp1(elBinsDeg,max(adjhbin,0)+1)+ elevationAngleAdjust_deg;
        rngbinsm = detection_params.rangeGatesMid;
        azbinsdeg = unique(detection_params.angleGatesMid(:,1));
        [rngbingrid,azbingrid] = meshgrid(rngbinsm,azbinsdeg);
        htImgOut = rngbingrid.*sind(angleElevDeg);
        height = -1*rngbingrid.*sind(angleElevDeg); % negate height for plotting
        %         figure,             surf(rngbinsm,azbinsdeg,height,'LineStyle','none');

        if combineHgtDop > -1 &&~noNewFigs
            if ~combineHgtDop
                if(~noNewFigs), figure; end
            else
                subplot(1,4,4);
            end
            surf(hdpeaks);
            view([90 90 ]);
            title('Multi Height/Doppler Flag Image');
            ylabel('Azimuth Bin');
            xlabel('Range Bin');

            axis tight;

            if ~combineHgtDop
                if(~noNewFigs), figure; end
            else
                subplot(1,4,3);
            end

            surf(rngbinsm,azbinsdeg,height,'LineStyle','none');
            %axis equal;
            view([90 90 ]);
            title('Height Image');
            ylabel('Azimuth (deg)');
            xlabel('Range (m)');
            axis tight;
        end
    else
        disp('No height plot')
    end

    if fmt == 21 % M16Z
        if(~noNewFigs), figure; end
        for z = 1:8
            subplot(2,4,z);
            surf(1:256,1:8:256,dbImg((z-1)*32+1:(z*32),:),'LineStyle','none');
            %surf(1:384,1:6:384,dbImg((z-1)*64+1:(z*64),:),'LineStyle','none');
            title(num2str(z));
            axis equal;
            axis tight;
            view([90 90 ]);
            %caxis([thresh thresh+45]);  % was 25
            caxis([29.3 40]);
        end
        xlabel('X(m)');
        ylabel('Y(m)');
        axis equal;
        axis tight;
        view([-90 90 ])
        title('X/Y Magnitude Clutter Image');


    else %all other formats
        ym=detection_params.rangeGatesMid; %(0:ysize-1)*pixel_meters;

        if(~noNewFigs),
            figure;
            curpos=get(gcf,'Position');
            set(gcf,'Position',curpos-[345   425  -760  -385])
        end
        toplot2=mag2db(clutImgOut);%(minrbin:maxrbin,:);
        if rangeNorm && ~calcSNR
            toplot2 = range_norm(toplot2);
        end
        normedClutImg=db2mag(toplot2);
        % Calculate color axis
        method = 1;
        vars.rangeNorm = rangeNorm;
        vars.useSVAcolormap = useSVAcolormap;
        vars.CaxisRngDb = CaxisRngDb;
        ctemp = getCaxisFunc(method,toplot2,vars);
        c1 = ctemp(1);
        c2 = ctemp(2);

        if fmt == 26 && any(plotRbinCut)  % 2d range/angle plot

            %             lastRbinmax=-1; % for tracking the max peak on JPs visualizer

            if max(plotRbinCut)> size(normedClutImg,2)
                warning(['RbinCut out of limits, reducing to ' num2str(size(normedClutImg,2)) ])
                plotRbinCut = min(plotRbinCut): size(normedClutImg,2);
            end
            rBinCutData.scanNumString= fp2(6:11);
            clear interpMagAng interpBinAng; % clear 1d angle out
            detfile = [fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'detectreport.bin'];
            if lastRbinmax>0
                det = loadDetectionData(detfile,detection_params.num_detections);
                [errm, foundmax] = min(abs(det.range-detection_params.rangeGatesMid(lastRbinmax)));
                if ~isempty(foundmax) && errm<3
                    rBinCutData.maxstr = ['Max Target Peak is at Z = ' num2str(det.xyzmx(3,foundmax)) ' meters']
                else
                    rBinCutData.maxstr = 'Searching for Max Target Peak';
                end
            else
                rBinCutData.maxstr = 'Searching for Max Target Peak';
            end
            [ interpBinRng, interpMagRng, interpMagAng, interpBinAng ] = makeRbincutPlot(normedClutImg,...
                rBinCutData,plotRbinCut,rbin_offset,rbin_width,...
                PMCW,detection_params,plotdBFS,dBFScorr_lin,swExponent,binAxes,noNewFigs,plot2DBscope,polarplot,polarplotrng,imageUnderlay,...
                doPrintInterpPeaks,dBpeakThresh,peakCensorRngAzEl);

            if showPlots==2
                saveas(gcf,[fp1 '/' fp2], 'fig');
            end
        elseif fmt == 4 || fmt == 7 || fmt == 25 || (fmt == 26 && ~any(plotRbinCut)) % range/angle plot
            if any(plotRbinCut)
                error('CI must be 2D data for Rbincut. If fmt=25 use SS data');
            end

            % check and correct crossrange for 2d
            if  fmt == 25
                assert(xsize==size(unique(detection_params.angleGatesMid(:,1)),1), ['Expected ' num2str(xsize) ' Az/El Bins'])
                xm=unique(detection_params.angleGatesMid(:,1));
            elseif fmt == 26
                assert(xsize==size(detection_params.angleGatesMid,1), ['Expected ' num2str(xsize) ' Azimuth Bins'])
                xm=unique(detection_params.angleGatesMid(:,1));
            else
                assert(xsize==size(detection_params.angleGatesMid,1), ['Expected ' num2str(xsize) ' Azimuth Bins'])
                xm=detection_params.angleGatesMid(:,1);
                if xm(1)==xm(2)
                    warning('identical first angle bin')
                    xm(2)=xm(2)+.001;
                end
            end

            % max of elevations for 2d
            if fmt==26
                allAz= unique(detection_params.angleGatesMid(:,1));
                allEl= unique(detection_params.angleGatesMid(:,2));
                azLen = length(allAz);
                elLen = length(allEl);
                useCIforInterp = abs(squeeze(max(reshape(db2mag(toplot2),azLen,elLen,[]),[],2)));
            else
                useCIforInterp = db2mag(toplot2);
            end

            validrbinmask = detection_params.rangeGatesMid>=0;
            % apply dBFS correction and convert to dB for plot
            if plotdBFS
                foo = transpose(dBFScorr_lin.RDC3(validrbinmask));
                useCIforInterp = useCIforInterp./repmat(foo,size(useCIforInterp,1),1).*2^(1);
            end
            useCItoplot = mag2db(useCIforInterp);

            % TODO generate height image for new fmt 26
            if enableHeightSurface && is2DMIMO && hasHeight
                % filter height to only show elevation on large objs,
                % all small objs are set to min height
                filtheight = height;
                filtmask=(median(useCItoplot(:))+heighSurfThreshdB)>useCItoplot;
                filtheight(filtmask)= surfheight;
            end

            if ~polarplot

                curpos=get(gcf,'Position');
                set(gcf,'Position',curpos-[320   120  -760  -120]);

                if doPrintInterpPeaks
                    peaksStr = printInterpPeaksCI(useCIforInterp, dBpeakThresh, peakCensorRngAzEl, xm, ym);
                    subplot(1,5,5)
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
                    subplot(1,5,1:4)
                elseif plotSliceDopBins
                    subplot(1,5,4:5)
                    stSlDopBins = [fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'stslice_bins.bin'];

                    f2 = fopen(stSlDopBins, 'r');
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
                    else
                        disp('Could not find static slice doppler bin voxel info');
                    end
                    if f2>0
                        if PMCW.MIMO2D
                            %                             plot3(rad2deg(detection_params.angleGatesMid(:,1)),detection_params.angleGatesMid(:,2),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2))),'.-'), xlabel('Az Angle (deg)'),ylabel('El Angle (deg)'), zlabel('Velocity (mps)');
                            centerEl=(5*16+1):6*16; plot(detection_params.angleGatesMid(centerEl,1),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(centerEl,ceil(size(staticSliceBinIndices,2)/2))),'.-'), xlabel('Az Angle (deg)'), ylabel('Velocity (mps)');
                        else
                            plot(detection_params.angleGatesMid(:,1),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2)))), xlabel('Az Angle (deg)'), ylabel('Velocity (mps)');
                        end
                        fclose(f2);
                    end
                    xlabel('Angle(deg)');
                    ylim([min(detection_params.dopplerGatesMidFFT) max(detection_params.dopplerGatesMidFFT)]);
                    if abs(egoMotion.velocity.est_x)<10000 || ~PMCW.MIMO2D
                        title(['Static Slice Doppler, CAN Ego Vel: ' num2str(round(egoMotion.velocity.x,1)) ' mps, Est.Ego Vel: ' num2str(round(egoMotion.velocity.est_x,1)) ' mps']);
                    else
                        title(['Static Slice Doppler, CAN Ego Vel: ' num2str(round(egoMotion.velocity.x,1)) ' mps, Est.Ego Vel: N/A           ']);
                    end
                    hold on, plot(0,egoMotion.velocity.x,'rx'), plot(0,egoMotion.velocity.est_x,'r+');
                    legend('SS dop bins', 'CANbus Ego Vel', 'SRS Est. Ego Vel', 'Location','southwest')
                    subplot(1,5,1:3)
                end

                if enableHeightSurface && is2DMIMO && hasHeight
                    surface( ym, xm, filtheight, toplot2);
                    zlabel('Height(m)');
                    grid on;
                else
                    surf( ym, xm, toplot2,'LineStyle','none');
                end

                set(gca, 'YDir', 'reverse')
                xlabel('Range(m)');
                ylabel('Angle(deg)');
                view([-90 90 ])
                title('Range/Angle Magnitude Clutter Image, Color is Mag dB');
                caxis([c1 c2]);

                colorbar;
                shading interp;
                axis tight;
            else
                plotrng=min(polarplotrng, ym(end));
                if plotrng >199
                    angticksp = 20;
                    xticksp = 20;
                elseif plotrng >99
                    angticksp = 10;
                    xticksp = 10;
                else
                    angticksp = 5;
                    xticksp = 5;
                end


                if doPrintInterpPeaks

                    peaksStr = printInterpPeaksCI(useCIforInterp, dBpeakThresh, peakCensorRngAzEl, xm, ym);
                    subplot(1,5,5)
                    axis off;
                    % Create a uicontrol of type "text"
                    mTextBox = uicontrol('style','text');
                    set(mTextBox,'String',peaksStr);
                    % To move the the Text Box around you can set and get the position of Text
                    mTextBoxPosition = get(mTextBox,'Position')

                    % The array mTextBoxPosition has four elements
                    % [x y length height]
                    % Something that I find useful is to set the Position Units to Characters,
                    set(mTextBox,'Units','normalized')
                    set(mTextBox,'Position',[.75 .4 .25 .5]);
                    title({'Interpolated Peaks '; [num2str(dBpeakThresh) 'dB Relative to Max']})
                    subplot(1,5,1:4)

                elseif plotSliceDopBins
                    subplot(1,maxsubp,firstsubp+1:maxsubp); % subplot(1,5,4:5)
                    stSlDopBins = [fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'stslice_bins.bin'];

                    f2 = fopen(stSlDopBins, 'r');
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
                    else
                        disp('Could not find static slice doppler bin voxel info');
                    end

                    if f2>0
                        if PMCW.MIMO2D
                            %                             plot3(rad2deg(detection_params.angleGatesMid(:,1)),detection_params.angleGatesMid(:,2),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2))),'.-'), xlabel('Az Angle (deg)'),ylabel('El Angle (deg)'), zlabel('Velocity (mps)');
                            centerEl=(5*16+1):6*16; plot(detection_params.angleGatesMid(centerEl,1),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(centerEl,ceil(size(staticSliceBinIndices,2)/2))),'.-'), xlabel('Az Angle (deg)'), ylabel('Velocity (mps)');
                        else
                            plot(detection_params.angleGatesMid(:,1),detection_params.dopplerGatesMidFFT(staticSliceBinIndices(:,ceil(size(staticSliceBinIndices,2)/2)))), xlabel('Az Angle (deg)'), ylabel('Velocity (mps)');
                        end
                        fclose(f2);
                    end
                    ylabel('Velocity (mps)');
                    xlabel('Angle(deg)');
                    ylim([min(detection_params.dopplerGatesMidFFT) max(detection_params.dopplerGatesMidFFT)]);
                    if abs(egoMotion.velocity.est_x)<10000 && ~PMCW.MIMO2D
                        title(['Static Slice Doppler, CAN Ego Vel: ' num2str(round(egoMotion.velocity.x,1)) ' mps, Est.Ego Vel: ' num2str(round(egoMotion.velocity.est_x,1)) ' mps']);
                    else
                        title(['Static Slice Doppler, CAN Ego Vel: ' num2str(round(egoMotion.velocity.x,1)) ' mps, Est.Ego Vel: N/A           ']);
                    end
                    hold on, plot(0,egoMotion.velocity.x,'rx'), plot(0,egoMotion.velocity.est_x,'r+');
                    legend('SS dop bins', 'CANbus Ego Vel', 'SRS Est. Ego Vel', 'Location','southwest')

                    testa=get(gca,'Position');
                    set(gca,'Position',[testa(1) testa(2)-.015 testa(3)+.06 testa(4)+.05 ]);

                    subplot(1,maxsubp,1:firstsubp); %subplot(1,5,1:3)
                end
                if 0
                    allranges=detection_params.rangeGatesMid(validrbinmask);
                    minrbin = 1;
                    maxrbin = 15;
                    minrng = allranges(minrbin);
                    maxrng = allranges(maxrbin);

                else
                    minrng= ym(1);
                    maxrng= ym(end);
                end
                useCItoplot(useCItoplot<median(useCItoplot(:))-30)=median(useCItoplot(:))-30;
                [t,r] = meshgrid(xm,ym);
                [x,y] = pol2cart(t,r);

                % Define some angular and radial range vectors for example plots
                anglims = [min(xm) max(xm) ];
                rnglims = [minrng min(plotrng,maxrng)];
                if is2DMIMO
                    ylim2 = floor(rnglims(2)*sind(max(abs(detection_params.angleGatesMid(:,1)))-3));
                else
                    ylim2 = floor(rnglims(2)*sind(max(abs(detection_params.angleGatesMid(:,1)))-3));
                end

                % Axis property cell array
                axprop = {'DataAspectRatio',[1 1 1],'View', [-90 90],...
                    'Xlim', [0 maxrng],       'Ylim', [-ylim2 ylim2],...
                    'XTick',[rnglims(1):xticksp:rnglims(2)],    'YTick',[fliplr(0:-angticksp:-ylim2) angticksp:angticksp:ylim2]};

                if (enableHeightSurface || transparentCmap) && is2DMIMO && hasHeight
                    if transparentCmap
                        polarplot3d(filtheight.','plottype','alphasurf','radialrange',[minrng maxrng],...
                            'angularrange',[deg2rad(xm)],'tickspacing',25,'colordata',useCItoplot.');
                    else
                        polarplot3d(filtheight.','plottype','surfn','radialrange',[minrng maxrng],...
                            'angularrange',[deg2rad(xm)],'tickspacing',25,'colordata',useCItoplot.');
                    end
                else
                    polarplot3d(useCItoplot.','plottype','surfn','radialrange',[minrng maxrng],...
                        'angularrange',[deg2rad(xm)],'tickspacing',25);
                end

                set(gca,axprop{:});

                if plotrng>99
                    temp=get(gca);
                    set(gca,'XTickLabel',num2str(round(str2double(temp.XTickLabel))));
                end

                method = 1;
                vars.rangeNorm = rangeNorm;
                vars.useSVAcolormap = useSVAcolormap;
                vars.CaxisRngDb = CaxisRngDb;
                caxis(getCaxisFunc(method,useCItoplot,vars));
                xlabel 'X (m)';
                ylabel 'Y (m)';
                xlim([0 plotrng]);
                colorbar off;
                if ~transparentCmap || ~is2DMIMO
                    shading interp;
                end

                set(gca, 'YDir', 'reverse')

                testa=get(gca,'Position');
                if plotSliceDopBins
                    set(gca,'Position',[testa(1)-.105 testa(2)-.025 testa(3)+.1 testa(4)+.06 ]);
                end

                %title('Range/Angle Polar Magnitude Clutter Image');
                scanstr=findstr(fp2,'_');
                title(['Clutter Image Top View, Scan: ' fp2(scanstr(1)+1:scanstr(2)-1) ', Host Vel: ' num2str(round(-3.6*egoMotion.velocity.x,1),'%10.1f\n') ' kph']);

                if plotStaticDets || plotMovingDets
                    detfile = [fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'detectreport.bin'];
                    ssfile= [fp1 '/' fp2(1:findstr(fp2,'_clutterimage')) 'stslice.bin'];
                    maxmag = mag2db(max(useCIforInterp(:)));
                    det = makePolarDetsOverlay(plotStaticDets,plotMovingDets,PMCW,detection_params, ...
                        detfile,ssfile,mitigate,egoMotion,bigDetSNR,detCircleMinMaxSize,plotrng,slowDopMode,maxmag,elevationAngleAdjust_deg);
                end

                if transparentCmap && is2DMIMO && hasHeight
                    grid on;
                    zlim([-12 12]);
                    view([-90 10])
                    %                     view([-81 70])
                    view([-14 10])
                elseif enableHeightSurface && is2DMIMO && hasHeight
                    zlabel('Height (m) dB');
                    hold on;
                    surf([0:80],[-40:40],surfheight*ones([81 81]),'FaceColor',[.04 .04 .04],'LineStyle','none','FaceAlpha',0.5);
                    hold off;
                else
                    if plotdBFS
                        zlabel('Mag dBFS');
                    else
                        zlabel('Mag dB');
                    end
                    if exist('det','var') && ~isempty(det) && ~isempty(det.xyzmx)
                        zlim([min(min(det.xyzmx(3,:)), median(useCItoplot(:))-20) max(max(det.xyzmx(3,:)),max(useCItoplot(:))+3)]);
                    end
                end


                if showPlots==2
                    saveas(gcf,[fp1 '/' fp2], 'fig');
                end
            end

        else
            %unknown format
            if(~noNewFigs), figure; end
            xm=(-xsize/2:xsize/2-1)*pixel_meters;
            surf(ym, xm,(mag2db(clutImgOut)),'LineStyle','none');
            set(gca, 'YDir', 'reverse')
            xlabel('X(m)');
            ylabel('Y(m)');
        end
    end
end
else
    disp(['*** Cant find ClutterImage data file ***']);
end

end

function [det]  = makePolarDetsOverlay(plotStaticDets,plotMovingDets,PMCW,detection_params, ...
    detfile,ssfile,mitigate,egoMotion,bigDetSNR,detCircleMinMaxSize,plotrng,slowDopMode,maxmag,elevationAngleAdjust_deg)

%% polar plot detections overlay
c = jet(300);
cc = [ c(60:130,:); repmat(c(130,:),40,1) ; c(130:240,:) ];
allAz= unique(detection_params.angleGatesMid(:,1));
allEl= unique(detection_params.angleGatesMid(:,2));
azLen = length(allAz);
elLen = length(allEl);

rangebin_m = median(diff(detection_params.rangeGatesMid));
minrng = min(detection_params.rangeGatesMid);
dopplerbin_mps = median(diff(detection_params.dopplerGatesMidFFT));
mindopp = min(detection_params.dopplerGatesMidFFT);

if plotStaticDets || plotMovingDets
    hold on;
    
    det = loadDetectionData(detfile,detection_params.num_detections);
    if ~isempty(det)
        censorDetMask = zeros(size(det.snr));
        dynFlag=logical(logical(~det.staticFlag) .* logical(det.range > 0.4));
        bigFlag = logical(det.snr > bigDetSNR);
        bigDet = logical(dynFlag .* bigFlag);
        if mitigate.angleSL

            dbins = length(detection_params.dopplerGatesMidFFT); % for wrap below
            rbins = length(detection_params.rangeGatesMid); % for wrap below

            %all range bin calcs are in distance bins
            detrangebin = round((det.range-minrng)/rangebin_m)+1; %one based
            detdopplerbin = round((det.dopp-mindopp)/dopplerbin_mps)+1; % one based
            %                         detrangemid=detection_params.rangeGatesMid(detrangebin+1);
            %                         detdoppmid=detection_params.dopplerGatesMidFFT(detdopplerbin+1);

            % Fixme fix this correctly
            detrangebin(detrangebin<2)=2;
            detrangebin(detrangebin>max(detection_params.rangeBins))=max(detection_params.rangeBins);

            if ~isempty(detrangebin)
            RDind = sub2ind([length(detection_params.rangeGatesMid) length(detection_params.dopplerGatesMidFFT)],detrangebin, detdopplerbin);
            RDindA(1,:) = sub2ind([length(detection_params.rangeGatesMid) length(detection_params.dopplerGatesMidFFT)],min(detrangebin+1,rbins), detdopplerbin);
            RDindA(2,:) = sub2ind([length(detection_params.rangeGatesMid) length(detection_params.dopplerGatesMidFFT)],max(detrangebin-1,1), detdopplerbin);
            RDindA(3,:) = sub2ind([length(detection_params.rangeGatesMid) length(detection_params.dopplerGatesMidFFT)],detrangebin, mod((detdopplerbin+1)-1,dbins)+1);
            RDindA(4,:) = sub2ind([length(detection_params.rangeGatesMid) length(detection_params.dopplerGatesMidFFT)],detrangebin, mod((detdopplerbin-1)-1,dbins)+1);

            [RDinduni, ia, ic]=unique(RDind);
            for RDit=1:length(RDinduni) %cycle through unique RD detections
                currDetMask = RDind == RDinduni(RDit); % detections in same RD
                tempaz = rad2deg(det.az(currDetMask)); % az of dets in same RD
                tempind = find(currDetMask);
                %mask dets in same RD outsise of +/-swamperDegToStartCensor
                [snrMax,indmax]=max(det.snr(currDetMask)); %max detection
                censorDetMask(tempind(tempaz>tempaz(indmax)+mitigate.swamperDegToStartCensor | tempaz<tempaz(indmax)-mitigate.swamperDegToStartCensor))=1;

                AdjacDetMask = ismember(RDind,RDindA(:,ia(RDit)).') & snrMax>(det.snr+mitigate.SLadjSNRthresh); % in adjacent RDbin
                censorDetMask = censorDetMask | AdjacDetMask;
            end
            end
        end
        if mitigate.MIMOdets % MIMO Range sidelobe detection
            if(0)
                [staticSlicesOut, ~, ~, ~, ~] = plot_staticSliceImgSD(ssfile,[],0,2,0);
                %
                if azLen~=size(staticSlicesOut,2)
                    prefilteredSS=sort(abs(max(reshape(staticSlicesOut,[],azLen,elLen),[],3)));
                else
                    prefilteredSS=sort(abs(staticSlicesOut(:,:)));
                end
                evalVec = mag2db(prefilteredSS(round(0.9*size(prefilteredSS,1)),:)).';
                interppts = interp1(allAz,evalVec,rad2deg(det.az));
                badDetMask = logical(det.mag<=interppts & det.snr<(max(det.snr)-40));
                censorDetMask = censorDetMask | badDetMask;
            else
                for itd=1:length(det.snr)
                    [~, det.azbin(itd) ] = min(abs(rad2deg(det.az(itd))-allAz));
                end
                ahist = hist(det.azbin,[1:azLen]);
                % initial bad angle
                % find bad angle sets
                % todo: merge adjacent sets
                badDetMask = logical(zeros(size(censorDetMask)));
                minbad = find(ahist>=mitigate.MIMOdetsCntSat(1))-mitigate.MIMOsideBinsSet;
                maxbad = find(ahist>=mitigate.MIMOdetsCntSat(1))+mitigate.MIMOsideBinsSet;
                %iter through sets
                for itb=1:length(minbad)
                    badIdx = minbad(itb):maxbad(itb);
                    badIdx = badIdx(badIdx>0 & badIdx<=azLen);
                    % check num of hits in the set
                    if sum(ahist(badIdx))>=mitigate.MIMOdetsCntSat
                        allBadDets = find(ismember(det.azbin,badIdx)); % get all hits in the set
                        tempMedSnr=median(det.snr(allBadDets)); % get their median SNR
                        allBadDets = allBadDets(det.snr(allBadDets) < tempMedSnr + mitigate.MIMOmedSNRadddB); % keep dets mitigate.MIMOmedSNRadddB bigger
                        censorDetMask(allBadDets) = 1;
                    end
                end
            end

        end

        if mitigate.DCdets && abs(egoMotion.velocity.x)>0
            % remove DC bias induced detections
            censorDetMask = censorDetMask | det.dopp==0;
        end
        if mitigate.extraSSdets && abs(egoMotion.velocity.x)>0
            % remove anything less than 0.5 mps, hax
            % for SRS bug on SS/moving boundary
            censorDetMask = censorDetMask | (~det.staticFlag & abs(cos(det.az)*egoMotion.velocity.x-det.dopp)<.5);
        end


        % dont plot detections past plot range
        censorDetMask = censorDetMask | det.range>plotrng;

%         censorDetMask = censorDetMask | det.range<40 ;%| (det.xyzmx(2,:)<-3 & det.xyzmx(1,:)<7);
        det.adjVel = det.dopp./cos(det.az) - egoMotion.velocity.x; % make det vel cos corrected, ground relative

        mask_pos = det.dopp > (egoMotion.velocity.x + max(detection_params.dopplerGatesMidFFT));
        mask_neg = det.dopp < (egoMotion.velocity.x + min(detection_params.dopplerGatesMidFFT));

        det.adjVel(mask_pos) = det.adjVel(mask_pos) + (2 * min(detection_params.dopplerGatesMidFFT));
        det.adjVel(mask_neg) = det.adjVel(mask_neg) - (2 * min(detection_params.dopplerGatesMidFFT));

        colortemp= zeros(1,length(det.staticFlag)); % create temp vector
        % detection_params.dopplerGatesMidFFT is derived in processSabineScanInfo.m
        if slowDopMode
            colortemp(det.adjVel>0) = det.adjVel(det.adjVel>0)/2;
            colortemp(det.adjVel<0) = det.adjVel(det.adjVel<0)/2;
        else
            colortemp(det.adjVel>0) = det.adjVel(det.adjVel>0)/max(detection_params.dopplerGatesMidFFT); % scale max vel
            colortemp(det.adjVel<0) = det.adjVel(det.adjVel<0)/-min(detection_params.dopplerGatesMidFFT); % scale min vel
        end
        colortemp(colortemp>1)=1; %limit max vel
        colortemp(colortemp<-1)=-1; % limit min vel
        colortemp=round((colortemp/2+.5)*size(cc,1)); % calc det colormap bins
        colortemp(colortemp==0)=1;
        colormatrix=ones(length(det.staticFlag),3); % set all white
        colormatrix(~det.staticFlag,:)=cc(colortemp(~det.staticFlag),:); % set colors on moving dets

        if plotStaticDets && any(det.staticFlag)
            if PMCW.MIMO2D
                bluesqflag = det.staticFlag & det.xyzmx(3,:)>=2;
                redsqflag = det.staticFlag & det.xyzmx(3,:)<2;
                if any(bluesqflag)
                    det.xyzmx(3,bluesqflag) = -1*(det.xyzmx(3,bluesqflag) + det.range(bluesqflag)*sind(elevationAngleAdjust_deg)); % add correction and invert for plotting
                    scatter3(...
                        det.xyzmx(1,bluesqflag), ...det.range(redsqflag).*cos(det.az(redsqflag)), ...
                        det.xyzmx(2,bluesqflag), ...det.range(redsqflag).*sin(det.az(redsqflag)), ...
                        det.xyzmx(3,bluesqflag), ... ones(1,sum(redsqflag==1)), ...
                        min(1000,max(4, (2 .* (det.snr(bluesqflag) - 20)) .^ 2)), ...
                        [1 1 1], ...
                        'LineWidth',1, ...
                        'Marker','s');
                end
                if any(redsqflag)
                    det.xyzmx(3,redsqflag) = -1*(det.xyzmx(3,redsqflag) + det.range(redsqflag)*sind(elevationAngleAdjust_deg)); % add correction and invert for plotting
                    scatter3(...
                        det.xyzmx(1,redsqflag), ...det.range(greensqflag).*cos(det.az(greensqflag)), ...
                        det.xyzmx(2,redsqflag), ...det.range(greensqflag).*sin(det.az(greensqflag)), ...
                        det.xyzmx(3,redsqflag), ...ones(1,sum(greensqflag==1)), ...
                        min(1000,max(4, (2 .* (det.snr(redsqflag) - 20)) .^ 2)), ...
                        [1 1 1], ...
                        'LineWidth',1, ...
                        'Marker','s');
                end
            else
                plotflag = det.staticFlag & ~censorDetMask;
                scatter3(...
                    det.range(plotflag).*cos(det.az(plotflag)), ...
                    det.range(plotflag).*sin(det.az(plotflag)), ...
                    ones(1,sum(plotflag==1)).*maxmag, ...
                    min(1000,max(4, (2 .* (det.snr(plotflag) - 20)) .^ 2)), ...
                    colormatrix(plotflag,:), ...
                    'LineWidth',1, ...
                    'Marker','s');
                if mitigate.plotSuppressedDets
                    aplotflag = det.staticFlag & censorDetMask;

                    scatter3(...
                        det.range(aplotflag).*cos(det.az(aplotflag)), ...
                        det.range(aplotflag).*sin(det.az(aplotflag)), ...
                        ones(1,sum(aplotflag==1)).*maxmag, ...
                        min(1000,max(4, (2 .* (det.snr(aplotflag) - 20)) .^ 2)), ...
                        [.25,.25,.25], ...
                        'LineWidth',1, ...
                        'Marker','s');

                end
            end
        end
        if  plotMovingDets && any(dynFlag)
            plotflag = dynFlag & ~censorDetMask;

            if PMCW.MIMO2D
                det.xyzmx(3,plotflag) = -1*(det.xyzmx(3,plotflag) + det.range(plotflag)*sind(elevationAngleAdjust_deg)); % add correction and invert for plotting
                scatter3(...
                    det.xyzmx(1,plotflag), ...det.range(redsqflag).*cos(det.az(redsqflag)), ...
                    det.xyzmx(2,plotflag), ...det.range(redsqflag).*sin(det.az(redsqflag)), ...
                    det.xyzmx(3,plotflag), ... ones(1,sum(redsqflag==1)), ...
                    min(detCircleMinMaxSize(2),max(detCircleMinMaxSize(1), (1 .* (det.snr(plotflag) - 20)) .^ 2)), ...
                    colormatrix(plotflag,:), ...
                    'LineWidth',2, ...
                    'Marker','o');
            else
                scatter3(...
                    det.range(plotflag) .* cos(det.az(plotflag)), ...
                    det.range(plotflag) .* sin(det.az(plotflag)), ...
                    ones(1,sum(plotflag==1)) .* maxmag, ...
                    min(detCircleMinMaxSize(2),max(detCircleMinMaxSize(1), (1 .* (det.snr(plotflag) - 20)) .^ 2)), ...
                    colormatrix(plotflag,:), ...
                    'LineWidth',2, ...
                    'Marker','o');
            end
            if mitigate.plotSuppressedDets
                aplotflag = dynFlag & censorDetMask;
                scatter3(...
                    det.range(aplotflag) .* cos(det.az(aplotflag)), ...
                    det.range(aplotflag) .* sin(det.az(aplotflag)), ...
                    ones(1,sum(aplotflag==1)) .* maxmag, ...
                    min(detCircleMinMaxSize(2),max(detCircleMinMaxSize(1), (1 .* (det.snr(aplotflag) - 20)) .^ 2)), ...
                    [.25,.25,.25], ...
                    'LineWidth',1.5, ...
                    'Marker','o');
            end
            plotbigflag = bigDet & ~censorDetMask;
            if any(plotbigflag)
                nn = complex(...
                    det.range(plotbigflag) .* cos(det.az(plotbigflag)), ...
                    det.range(plotbigflag) .* sin(det.az(plotbigflag)));
                nn = nn ./ abs(nn);
                nn = nn .* 0.3 .* (det.adjVel(plotbigflag) - (sign(det.adjVel(plotbigflag)) * 0.4));

                quiver3(...
                    det.range(plotbigflag) .* cos(det.az(plotbigflag)), ...
                    det.range(plotbigflag) .* sin(det.az(plotbigflag)), ...
                    ones(1,sum(plotbigflag==1)) .* maxmag, ...
                    real(nn), ...
                    imag(nn), ...
                    zeros(1,sum(plotbigflag==1)),...
                    'AutoScale', 'off',...
                    'LineWidth', 1,...
                    'MaxHeadSize', 1.0,...
                    'Color', [1,1,1]);
            end
        end
    end
end
end

function [b] =  range_norm(a)
new_floor_bins = min(a(:,[end-10:end-3]));
new_floor = mean(new_floor_bins);
current_floor = min(a(:,:));
offset = new_floor - current_floor;
offset(find(current_floor == 0)) = 0;
offset = repmat(offset,size(a,1),1);

b = a + offset;
b(:,find(current_floor == 0)) = new_floor;      % nuke invalid bins

end

