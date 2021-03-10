function dets= loadAllDets(path, live, startscan)

cnt = 0;
detcntidx=1;

i = startscan;
startedRunning=0;
keepWaiting = 10; %
gooddetcnt=0;
    while 1 && keepWaiting
    % Wait for next scan output files to appear
    while 1 && keepWaiting
        if live
            file_dets = sprintf('%s/scan_%06d_detectreport.bin', path, i);
            file_next = sprintf('%s/scan_%06d_info.json', path, i+1);

            while exist(file_next,'file')
                i = i + 1;
                file_next = sprintf('%s/scan_%06d_info.json', path, i+1);
                disp(['Live mode on, Skipping TO SCAN ' num2str(i)]);
                startscan = i;
            end
        else
            file_this = sprintf('%s/scan_%06d_info.json', path, i);
            file_dets = sprintf('%s/scan_%06d_detectreport.bin', path, i);
            while ~exist(file_this,'file') && ~startedRunning
                i = i + 1;
                disp(['Live mode off, Skipping TO SCAN ' num2str(i)]);
                file_this = sprintf('%s/scan_%06d_info.json', path, i);
                file_dets = sprintf('%s/scan_%06d_detectreport.bin', path, i);
                startscan = i;
            end
        end
        if exist(file_dets,'file') % && exist(file_dets,'file')
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
        [~, ~, detection_params, ~, ~, egoMotion] = processSabineScanInfo(file_curr);
        try
            times(detcntidx) = egoMotion.scan_timestamp;
            dets{detcntidx} = loadDetectionData(file_dets,detection_params.num_detections);
            detcntidx = detcntidx+1;
            startedRunning = 1;
        catch
            disp('Caught error in detloader');
        end

        i = i + 1;
    end
end

end
