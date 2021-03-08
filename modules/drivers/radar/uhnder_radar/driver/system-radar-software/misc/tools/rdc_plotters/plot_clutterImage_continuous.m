function plot_clutterImage_continuous(path, max_range, live, startscan, room_x, room_y)

polarplot=1;
plotdBFS = 2;
savePolarPlot = 0;
doPrintInterpPeaks=0;
dBpeakThresh = -15;
peakCensorRngAzEl = [0 999 -180 180 -180 180];
rangeAdjustm = 0;
plotStaticDets = 0;
plotMovingDets = 1;
noNewFigs = 1;
plotSliceDopBins = 1;
elevationAngleAdjust_deg = 0;%-1.3125;
binAxes=0;
plotdBFS=2;
showPlots=1;
plotRbinCut=[];
polarplot = 1;
polarplotrng = 400;
imageUnderlay=[];
plot2DBscope=0;

makeMovie = 1;
cnt = 0;

if ~exist('path','var')
    error('Usage:  plot_clutterImage_continuous(path, max_range)');
end
if ~exist('max_range','var')
    max_range = 38;
end
if ~exist('live','var')
    live = 1;
end
if ~exist('startscan','var')
    startscan = 0;
end

i = startscan;
startedRunning=0;
Fig1 = figure;
if makeMovie
    set(gcf,'MenuBar','none');
end
curpos=get(gcf,'Position');
set(gcf,'Units','pixels','outerposition',[ 0 0 1920 1080])
set(gcf,'Color',[1 1 1]);
keepWaiting = 10; %
gooddetcnt=0;
while 1 && keepWaiting
    % Wait for next scan output files to appear
    while 1 && keepWaiting
	    file_ci = sprintf('%s/scan_%06d_clutterimage.bin', path, i);
        file_ss = sprintf('%s/scan_%06d_stslice.bin', path, i);
		file_dets = sprintf('%s/scan_%06d_detectreport.bin', path, i);
		file_this = sprintf('%s/scan_%06d_info.json', path, i);
        file_next = sprintf('%s/scan_%06d_info.json', path, i+1);
        if live


            while exist(file_next,'file')
                i = i + 1;
                file_next = sprintf('%s/scan_%06d_info.json', path, i+1);
                disp(['Live mode on, Skipping TO SCAN ' num2str(i)]);
                startscan = i;
            end
        else
            while ~exist(file_this,'file') && ~startedRunning
                i = i + 1;
                disp(['Live mode off, Skipping TO SCAN ' num2str(i)]);
                file_this = sprintf('%s/scan_%06d_info.json', path, i);
                file_dets = sprintf('%s/scan_%06d_detectreport.bin', path, i);
                startscan = i;
            end
        end
        if exist(file_ci,'file') % && exist(file_dets,'file')
            keepWaiting = 10;
            startedRunning = 1;
            break;
        else
            if ~live
                keepWaiting = keepWaiting-1;
            end
        end
        if ~exist(sprintf('%s/scan_%06d_info.json', path, startscan))
            if (i > startscan)
                disp('RESETTING TO SCAN 0');
                i = 0;
                startscan = 0;
                clf;
                continue;
            end
            i = startscan;
        end
        if ~live
            i = i+1;
        end
        pause(0.01);
    end
    clf;
    
    
    %     SDjsonfilepath = [path '/scan_' num2str(i,'%06d'),'_info.json'];
    file_curr = sprintf('%s/scan_%06d_info.json', path, i);
    disp(['Processing scan_' num2str(i,'%06d') '_info.json']);
    if ~exist(file_curr,'file')
        disp('Complete'); close;
        fc=fopen([path '/complete.txt'],'w');
        fwrite(fc,0);
        fclose(fc);
    else
        [~, ~, ~, ~, ~, egoMotion] = processSabineScanInfo(file_curr);
        
        try
            %         plot_staticSliceImgSD(file_ss,[],0,0,1,80:126,0,0,0,0,[],0,[],1,0,[]);
            %         colorbar;
            if exist(file_ci,'file')
            plot_clutterImageSD(file_ci, binAxes, plotdBFS, showPlots, plotRbinCut, polarplot, max_range, ...
                doPrintInterpPeaks, dBpeakThresh, peakCensorRngAzEl, rangeAdjustm, noNewFigs, plotSliceDopBins, imageUnderlay, plot2DBscope, plotStaticDets, plotMovingDets, elevationAngleAdjust_deg );
            end
        catch
            disp('Caught error in clutter image plotter');
        end
        title(num2str(i));
        if exist('room_x','var') && exist('room_y','var')
            % [room_x,room_y] = ginput(4);
            zl = zlim;
            line(room_x, room_y, (zl(2)-1)*ones(length(room_x),1), 'LineWidth',2, 'Color',[0,1,0]);
        end
        try
            drawnow;
        catch
        end
        if makeMovie
            if ~exist('starttime','var')
                starttime= egoMotion.scan_timestamp;
                vidObj = VideoWriter([path '/clutimg'], 'MPEG-4');
                open(vidObj);
                %             set(gca,'nextplot','replacechildren');
            end
            while cnt<=round((egoMotion.scan_timestamp-starttime)/1e6/(1/30))
                cnt=cnt+1;
                
                % Write each frame to the file.
                drawnow
                currFrame = getframe(gcf);%a, [pos(1)+10 pos(2)+10 pos2(1)+pos2(3)-10 pos2(2)+pos2(4)-10 ]);% [210 80 1550 975]);
                writeVideo(vidObj,currFrame);
                drawnow
            end
        end
        i = i + 1;
    end
end
if makeMovie
    close(vidObj);
end
end
