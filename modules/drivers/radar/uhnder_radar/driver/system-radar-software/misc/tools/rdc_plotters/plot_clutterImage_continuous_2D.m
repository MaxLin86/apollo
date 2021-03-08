function plot_clutterImage_continuous_2D(fpath, range_range, live, startscan, polarMaxRng, makeMovie)

global lastRbinmax

lastRbinmax=-1; % for tracking the max peak on JPs visualizer
% range_range    - [RB_min:RB_max], in range bins, not meters

if ~exist('fpath','var')
    error('Usage:  plot_clutterImage_continuous_2D(path, max_range)');
end
if ~exist('range_range','var')
    range_range = [2:8];
end
if ~exist('live','var') %  0 replay with kb, 1 live mode
    live = 0;
end
if ~exist('startscan','var')
    startscan = 0;
end

if ~exist('polarMaxRng','var')
    polarMaxRng = 320;
end

if ~exist('makeMovie','var')
    makeMovie=0; % turn movie maker on for "hands free" version
end

i = startscan;
xdone = 0;
figure;
curpos=get(gcf,'Position');
% set(gcf,'Position',curpos-[545   520  -960  -550]);
set(gcf,'Units','pixels','outerposition',[ 0 0 1920 1080])
% set(gcf,'Color',[1 1 1]);
rbin_offset = 0;
rbin_width = 5;
cntRow=0;
dbin_offset=0;
dbin_width=1;
plotStaticDets= 1;
plotMovingDets= 1;
elevationAngleAdjust_deg=0;
cnt = 0;
endrun = 0;

while 1
    % Wait for next scan output files to appear
    while 1
        
        if live
            file_next = sprintf('%s/scan_%06d_info.json', fpath, i+1);
            while exist(file_next,'file')
                i = i + 1;
                file_next = sprintf('%s/scan_%06d_info.json', fpath, i+1);
                disp('SKIPPING');
            end
        end
        file_ss = sprintf('%s/scan_%06d_stslice.bin', fpath, i);
        file_dets = sprintf('%s/scan_%06d_detectreport.bin', fpath, i);
        file_ci = sprintf('%s/scan_%06d_clutterimage.bin', fpath, i);
        file_sd = sprintf('%s/scan_%06d_info.json', fpath, i);

        if exist(file_ci,'file')
            disp('FOUND CI');
            break;
        end
        if ~exist(sprintf('%s/scan_000000_info.json', fpath))
            if (i > 0)
                disp('RESETTING TO SCAN 0');
                clf;
            end
            i = 0;
        end
        if ~live
            i = i+1;
        end
        pause(0.01);
    end
    clf;
    
                disp('PLOTTING...');

        try
            [~, ~, detection_params, ~, ~, egoMotion] = processSabineScanInfo(file_sd);
        catch
            if ~live
                disp([file_sd ' not found']);
                endrun = 1;
                break;
            else
                pause(.01);
                continue;
            end
        end

    %check for end movie flag
    if ~(endrun && makeMovie)
        %load Static slice OR CI data
            %useCIdata
            [clutImgOut, ~, interpRng, interpMagAng, interpAng ] = plot_clutterImageSD(...
                file_ci, 0,0,1,range_range,1,polarMaxRng,1,0,[],0,1,0,[],1,plotStaticDets,plotMovingDets,  ...
                elevationAngleAdjust_deg,rbin_offset,rbin_width,dbin_offset,dbin_width);
        % load SRS Detection report
        det = loadDetectionData(file_dets,detection_params.num_detections);
        
        interpdet.xyz(1) = interpRng(1)*cosd(interpAng.Az(1));
        interpdet.xyz(2) = interpRng(1)*sind(interpAng.Az(1));
        interpdet.xyz(3) = interpRng(1)*sind(interpAng.El(1));
        
        if isempty(det)
            det.xyzmx = [0; 0; 0];
            det.mag = 1;
        end
        deltaxyz=det.xyzmx - repmat(interpdet.xyz.',[1 size(det.xyzmx,2)]);
        errxyz=(deltaxyz(1,:).^2+deltaxyz(2,:).^2+deltaxyz(1,:).^2).^.5;
        [closestError, closestdet] = min(errxyz);
        
        if(0) % old height map code
            % load Clutter image
            [ clutImgOut, htImgOut ] = plot_clutterImage(file_ci, 0, 0, 1, 80, 0, ...
                1, -15, [], 0, 0, 1, 0 );
            [~, ~, detection_params, ~, ~, ~] = processSabineScanInfo(file_sd);
            close, close; % todo add option to disable plots
            
            [~, indRbin]=min(abs(detection_params.rangeGatesMid-interpRng));
            azbins_deg = rad2deg(unique(detection_params.angleGatesMid(:,1)));
            [~, indAzbin]=min(abs(azbins_deg-interpAng.Az));
            
            %mag2db(clutImgOut(indAzbin,indRbin))
            CIheight = htImgOut(indAzbin,indRbin);
            
            cntRow = cntRow+1;
            tableout(cntRow,:) = [i closestError interpdet.xyz(1:3) mag2db(mean([interpMagAng.Az interpMagAng.El])) det.xyzmx(:,closestdet).' mag2db(det.mag(closestdet)) CIheight];
        end
        drawnow;
        
        if ~live
            if makeMovie
                if ~exist('starttime','var')
                    starttime= egoMotion.scan_timestamp;
                    vidObj = VideoWriter([fpath '/clutimg'], 'MPEG-4');
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
                
                i=i+1;
            else
                while 1
                    [a, b, getch] = ginput(1);
                    if ~isempty(getch)
                        switch getch
                            case 'q'
                                xdone = 1;
                                break;
                            case 'd'
                                dbin_offset = dbin_offset + 1;
                                break;
                            case 'D'
                                dbin_offset = dbin_offset - 1;
                                break;
                            case 'f'
                                dbin_width = dbin_width + 1;
                                break;
                            case 'F'
                                dbin_width = max(dbin_width - 1,1);
                                break;
                            case 'w'
                                rbin_width = rbin_width + 1;
                                break;
                            case 'W'
                                rbin_width = max(rbin_width - 1, 1);
                                break;
                            case 'r'
                                rbin_offset = rbin_offset + 1;
                                break;
                            case 'R'
                                rbin_offset = rbin_offset - 1;
                                break;
                            case 31   %Down arrow key
                            case 30   %Up arrow key
                            case 29   %Right arrow key
                                i = i + 1;
                                break;
                            case 28   %Left arrow key
                                i = i - 1;
                                break;
                            case '0'
                                i = 0;
                                break;
                            otherwise
                                %Do nothing and keep waiting
                        end
                    end
                end
            end
        else
            i = i + 1;
        end
        if xdone
            close;
            break;
        end
    else
        disp('Complete'); close;
        fc=fopen([fpath '/complete.txt'],'w');
        fwrite(fc,0);
        fclose(fc);
        close(vidObj);
        break;
    end
end
