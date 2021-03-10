function plot_pointcloud(fpath, live, initialseq, makeMovie)

if ~exist('live','var')
    live=0;
end
if ~exist('initialseq','var')
    initialseq=0;
end
if ~exist('makeMovie','var')
    makeMovie=0;
end

close all;

setup = 0;
i = initialseq;
while 1
    if live
        file_next = sprintf('%s/scan_%06d_pointcloud.bin', fpath, i + 1);
        while exist(file_next, 'file')
            i = i + 1;
            file_next = sprintf('%s/scan_%06d_pointcloud.bin', fpath, i + 1);
        end
    end
    s = sprintf('%s/scan_%06d_pointcloud.bin', fpath, i);
    if exist(s, 'file')
        if ~setup
            figure;
            set(gcf, 'Units', 'pixels', 'outerposition', [0 0 1920 1080])
            c = zeros(501, 3);
            c_autumn = colormap(autumn(400));
            c_cool = colormap(cool(400));
            c(1:250,:) = c_autumn(1:250,:);
            c(251,:) = 0;
            c(252:end,:) = c_cool(end-249:end,:);
            colormap(c);
            if makeMovie
                set(gcf,'MenuBar','none');
            end
            setup = 1;
        end
        try
            f1 = fopen(s,'r');
            s1 = dir(s);
            tempdat = fread(f1,s1.bytes/2,'uint16');
            tempdat = reshape(tempdat,6,[]);
            d.range = tempdat(1,:);
            d.azimuth = double(tempdat(2,:))/2^8;
            d.elevation = double(tempdat(3,:))/2^8;
            d.doppler = tempdat(4,:);
            d.snr = double(tempdat(5,:))/2^8;
            d.flags = bitand(tempdat(6,:),0000000011111111);
            d.reserved = bitand(tempdat(6,:),1111111100000000);
            fclose(f1);
            if ~size(d, 1)
                % skip empty frames (probably caused by CIU overflow)
                i = i + 1;
                continue
            end
            try
                % reading timestamps requires the jsonlab toolbox
                jsonpath = sprintf('%s/scan_%06d_info.json', fpath, i);
                scanData = loadjson(jsonpath);
                timestamp = scanData.scan_timestamp;
                [PMCW, antenna, detection_params, dBFScorr_lin, swExponent, egoMotion] = processSabineScanInfo(jsonpath);
                az_bins = unique(detection_params.angleGatesMid(:,1).');
                el_bins = unique(detection_params.angleGatesMid(:,2).');
                d.azimuth = interp1(1:length(az_bins),az_bins,d.azimuth+1,'linear','extrap');
                d.elevation = interp1(1:length(el_bins),el_bins,d.elevation+1,'linear','extrap');
                d.range = detection_params.rangeGatesMid(d.range+1).';
                d.doppler = detection_params.dopplerGatesMidFFT(d.doppler+1);
            catch
                timestamp = 0;
            end
            egovel_kph = round(scanData.ego_velocity_X*-3600/1000);
            scatterplot(d, i, egovel_kph);
            %             if initialseq
            %                 return
            %             end
            if makeMovie

                if ~exist('vidObj','var')
                    vidObj = VideoWriter([fpath '/points'], 'MPEG-4');
                    open(vidObj);
                    starttime = timestamp;
                    cnt = 0;
                end
                writeVideo(vidObj, getframe(gcf));
                if timestamp
                    % add more frames to simulate recorded scan cadence
                    % assumes writeVideo defaults to 30FPS and scan
                    % timestamps are in microsecond units
                    while cnt <= round((timestamp-starttime)/1e6/(1/26.25))
                        cnt = cnt + 1;
                        drawnow
                        currFrame = getframe(gcf);
                        writeVideo(vidObj, currFrame);
                        drawnow
                    end
                end
            else
                if ~live
                    pause(0.25); % pause for simulated dwell
                end
            end
        catch
            disp([s ' has no points']);
        end
        i = i + 1;
    else
        if live
            pause(0.1); % pause before checking again
        else
            pause(2);   % leave last scan on the screen briefly
            close all;
            if exist('vidObj', 'var')
                close(vidObj);
            end
            return
        end
    end
end

function scatterplot(data, scan_no, egovel_kph)

% plot objects with SNR less than 22 as tiny dots
S = data.snr - 15;
S(S<=0) = 1;

doprange = 30;
C = data.doppler;

if all(data.elevation == 0)
    x = data.range .* sin(data.azimuth);
    y = data.range .* cos(data.azimuth);
    scatter(x(S>0), y(S>0), S(S>0) .^ 2, C(S>0),'LineWidth',1.5);
%     axis([-60, 60, 0, 100]);
    caxis([-doprange doprange]);
    xlabel('Y left-/right+ [m]');
    ylabel('X backward-/forward+ [m]');
    title(sprintf('Scan %d, Host Vel %d kph', scan_no, egovel_kph));
else
    subplot(1,2,1)
    x = data.range .* sin(data.azimuth) .* cos(data.elevation);
    y = data.range .* cos(data.azimuth) .* cos(data.elevation);
    scatter(x, y, S .^ 2, C);
%     axis([-20, 20, 0, 50]);
    caxis([-doprange doprange]);
    xlabel('Y left-/right+ [m]');
    ylabel('X backward-/forward+ [m]');
    title(sprintf('Scan %d Range By Azimuth', scan_no));

    subplot(1,2,2)
    x = data.range .* sin(data.elevation) .* cos(data.elevation);
    z = data.range .* cos(data.elevation);
    scatter(z, x, S .^ 2, C);
%     axis([0, 50, -10, 10]);
    caxis([-doprange doprange]);
    set(gca,'Ydir', 'reverse')
    ylabel('Z up-/down+ [m]');
    xlabel('X backward-/forward+ [m]');
    title(sprintf('Scan %d Elevation By Range', scan_no));
end

colorbar;
colorbar('Ticks', [-doprange, doprange],...
         'TickLabels', {sprintf('%d mps', -doprange),...
                        sprintf('%d mps', doprange)})
drawnow

%[x, y, z] = sph2cart(data.azimuth, data.elevation, data.range);
%scatter3(x, y, z, (S .* 2) .^ 2, C);
%set(gca,'XLim',[0, 50],'YLim',[-25, 25],'ZLim',[-10, 10]);
%xlabel('X backward-/forward+ [m]');
%ylabel('Y left-/right+ [m]');
%zlabel('Z up-/down+ [m]');
