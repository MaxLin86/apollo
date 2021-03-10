function plot_RDC2zd_continuous(path, max_range, live, startscan,makeMovie)
% e.g. plot_RDC2zd_continuous('X:\software\jonathan\RDC2zerodopAgain\init_192.168.95.1',8,0,257)
persistNumScans = 5;
interpFact = 4; %2^4 interp factor
useRDC1=0;
%cmxin=[path '/OldCal/cal_az_only'];

cnt = 0;

if ~exist('path','var')
    error('Usage:  plot_RDC2zd_continuous(path, max_range)');
end
if ~exist('max_range','var')
    max_range = 400;
end
if ~exist('live','var')
    live = 1;
end
if ~exist('startscan','var')
    startscan = 0;
end

i = startscan;
Fig1 = figure;
if makeMovie
    set(gcf,'MenuBar','none');
end
curpos=get(gcf,'Position');
set(gcf,'Units','pixels','outerposition',[ 0 0 1920 1080])
set(gcf,'Color',[1 1 1]);

if makeMovie
    set(gcf,'MenuBar','none');
end
curpos=get(gcf,'Position');
set(gcf,'Units','pixels','outerposition',[ 0 0 1920 1080])
set(gcf,'Color',[1 1 1]);

keepWaiting = 500;
%gooddetcnt=0;
persistPhaseData = [];
persistRangeData = [];
scanIDs = [];
while 1 && keepWaiting
    % Wait for next scan output files to appear
    while 1 && keepWaiting
        % keepWaiting = keepWaiting - 1;
        file_zd = sprintf('%s/scan_%06d_rdc2zdop.bin', path, i);
        file_rdc1 = sprintf('%s/scan_%06d_rdc1.bin', path, i);
        file_next = sprintf('%s/scan_%06d_info.json', path, i+1);
        if live
            while exist(file_next,'file')
                i = i + 1;
                file_next = sprintf('%s/scan_%06d_info.json', path, i+1);
                disp(['Skipping TO SCAN ' num2str(i)]);
                startscan = i;
            end
        end
        if useRDC1
            if exist(file_rdc1,'file') % && exist(file_dets,'file')
                break;
            end
        else
            if exist(file_zd,'file') % && exist(file_dets,'file')
                break;
            end
        end
        if ~exist(sprintf('%s/scan_%06d_info.json', path, startscan),'file')
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

    file_curr = sprintf('%s/scan_%06d_info.json', path, i);
    disp(['Processing scan_' num2str(i,'%06d') '_info.json']);
    if ~exist(file_curr,'file')
        disp('Complete'); close;
        fc=fopen([path '/complete.txt'],'w');
        fwrite(fc,0);
        fclose(fc);
    else
        keepWaiting = 500;
        [PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(file_curr);
            
        if useRDC1
            file_in = sprintf('%s/scan_%06d_info.json', path, i);
            [~, ~, RDC2data, ~] = procRDC(file_in,cmxin);
            tempRDC2 = squeeze(permute(RDC2data(:,:,floor(size(RDC2data,3)/2)+1),[2 1 3]));
        else
            tempRDC2 = plot_rdc2_zerod(file_zd, length(detection_params.rangeBins),0); %binAxes, plotdBFS, showPlots, plotRbinCut, polarplot, max_range, ...
        end
        % Reorder Range bins, mask out invalid bins
        validrbinmask = detection_params.rangeGatesMid>=0;
        if ~useRDC1
            tempRDC2 = tempRDC2(:,detection_params.rangeGatesOrdering(validrbinmask));
        end
        % Mask out undesired ranges
        desiredRangeBins = detection_params.rangeGatesMid(validrbinmask);
        desiredRangeMask = desiredRangeBins<max_range;
        desiredRangeBins = desiredRangeBins(desiredRangeMask);
        % apply mask
        tempRDC2 = tempRDC2(:,desiredRangeMask);
        numVrxs = size(tempRDC2,1);
        % initialize the results matrices if first run
        if isempty(persistPhaseData)
            persistPhaseData = zeros(numVrxs,persistNumScans);
            persistRangeData = persistPhaseData;
            scanIDs = zeros(1,persistNumScans);
        end
        
        % roll over rover, shift old results
        persistPhaseData(:,2:end) = persistPhaseData(:,1:end-1);
        persistRangeData(:,2:end) = persistRangeData(:,1:end-1);
        scanIDs(2:end) = scanIDs(1:end-1);
        
        %% Initialize sinc filter.
        Ts  = 0.5;              % Sampling interval
        filter_len = 11;        % Number of coeffients
        gauss_win_var = 0.44;    % Variance of Gaussian window function.
        %sinc_coeff = init_sinc_filter(Ts,filter_len,gauss_win_var);
        sinc_coeff = sinc_ip_sample(Ts,filter_len,gauss_win_var);
        for itVrx = 1:numVrxs
            
            % Input Parmeters:
            % x             : Input signal
            % x_tstart      : Start time of input signal
            % x_tend        : End time of input signal
            % samp_interval : Interval between two samples of signal x
            % ip_factor     : Interpolation factor
            r_tstart = min(desiredRangeBins); % really shouldnt need this, bad code
            r_tend = max(desiredRangeBins);
            r_s = detection_params.rangeGatesMid(2)-detection_params.rangeGatesMid(1);
            %RDC2interp(itVrx,:) = sinc_ip_poly(tempRDC2(itVrx,:),interpFact,sinc_coeff);
            RDC2interp(itVrx,:) = sinc_ip_sample(tempRDC2(itVrx,:),interpFact,sinc_coeff);
            interpRanges = 0:r_s/(2^interpFact):r_tend+r_s-r_s/(2^interpFact);

            %fixme
%             fixmeInterpRanges = 0:r_s/(2^interpFact):r_tend;
%             fixmeRDC2interp(itVrx,:)=interp1(desiredRangeBins,tempRDC2(itVrx,:),fixmeInterpRanges,'pchip');
%             figure, plot(InterpRangesNew, abs(RDC2interp(itVrx,:)),desiredRangeBins,abs(tempRDC2(itVrx,:)),0:r_s/(2^interpFact):r_tend,abs(fixmeRDC2interp(itVrx,:)))
            [~,rangeidx] = max(abs(RDC2interp(itVrx,:)));
            persistPhaseData(itVrx,1) = RDC2interp(itVrx,rangeidx);
            persistRangeData(itVrx,1) = interpRanges(rangeidx);
            % sinc_ip(x,x_tstart,x_tend,samp_interval,ip_factor)
            
        end
        scanIDs(1) = i;
        
        %surf(detection_params.rangeGatesMid(desiredRangeBins),1:size(tempRDC2,1),mag2db(abs(tempRDC2(:,desiredRangeBins))))
            
            maxValidIdx = min(find(sum(persistPhaseData)==0))-1;
            if isempty(maxValidIdx)
                maxValidIdx = persistNumScans;
            end
            lin=lines;
            temp=[lin(1:7,:); .5 .5 .5];
            cmap=[repelem(temp(:,1),8), repelem(temp(:,2),8), repelem(temp(:,3),8)];
            %             scatter(real(max(tempRDC2(:,desiredRangeMask),[],2)),imag(max(tempRDC2(:,desiredRangeMask),[],2)),20,cmap,'filled')

            figure(Fig1);
            subplot(1,2,1)
            % account for matlab being a little picky
            if maxValidIdx==1
                fpd = flatten(persistPhaseData(:,1:maxValidIdx));
                scatter(real(fpd),imag(fpd),20,squeeze(reshape(permute(repmat(cmap,1,1,maxValidIdx),[1 3 2]),1,[],3)),'filled')
            else
                plot(persistPhaseData(:,1:maxValidIdx).','.-','MarkerSize',10)
            end
            title(['RDC2 (FFT pass) Phase for the past ' num2str(persistNumScans) ' scans, last scan ' num2str(i) ', temp: ' num2str(round(detection_params.chip_temp_C,2)) 'degC'])
            xlabel('I');
            ylabel('Q');
            axis equal;
            
%             figure(Fig2);
            subplot(1,2,2)
            if maxValidIdx==1
                frd = flatten(persistRangeData(:,1:maxValidIdx));
                scatter(repmat(scanIDs(1:maxValidIdx),1,numVrxs),frd,20,squeeze(reshape(permute(repmat(cmap,1,1,maxValidIdx),[1 3 2]),1,[],3)),'filled')
            else
                plot(scanIDs(1:maxValidIdx),persistRangeData(:,1:maxValidIdx).','.-','MarkerSize',10)
            end
            title(['RDC2 (FFT pass) Rangebin for the past ' num2str(persistNumScans) ' scans, last scan ' num2str(i) ', temp: ' num2str(round(detection_params.chip_temp_C,2)) 'degC'])
            xlabel('Scan Num');
            ylabel('Range Bin');
        
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
