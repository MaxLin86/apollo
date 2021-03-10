% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
% sam=loadDetectionData('C:\Uhnder\BitBucket\sam-sim\testVectors\verifVectors\OEMscan1case2_BPSK_Lc4096_N360_forSoC\scan_000001_detectreport.bin')
%scannums=1:207;
%socpath='~/Documents/Uhnder/Bitbucket/sabine-radar-sw/build-x86';

function plot_dets_movie(socpath,scannums)

    if ~exist('loadjson','file')
        addpath('../jsonlab/');
    end
    
    cm = colormap('jet');
    
    files = dir(fullfile(socpath, '*_detectreport.bin'));

    for itscan=scannums
        fig=figure;
        set(fig,'Position',[5 40 1520 850]);
        %soc=loadDetectionData([socpath '/socsim_' num2str(itscan,'%06d') '_detectreport.bin']);
        soc_detectreport = [files(itscan).folder '/' files(itscan).name];
        [fp1, fp2, fp3] = fileparts(soc_detectreport);
        socjsonfilepath = [fp1 '/' fp2(1:strfind(fp2,'_detectreport')) 'info.json'];
        scanData = loadjson(socjsonfilepath);
        soc_num_detections = scanData.num_detections;
        soc = loadDetectionData(soc_detectreport,soc_num_detections);

        soc.mag = soc.mag - min(soc.mag) + 1;
        soc.mag = soc.mag ./ max(soc.mag);
        soc.mag = ((soc.mag .* 4) .^ 4);

        cm = colormap('jet');
        cx = soc.dopp;
        cx = min(cx, 26);
        cx = max(cx, -26);
        cx = cx + 27;
        cx = cx ./ 53 .* size(cm,1);
        c = cm(floor(cx),:);

        subplot(1,3,1)
        scatter(soc.dopp,soc.range,soc.mag,c,'LineWidth',2 );
        xlabel('Doppler [m/s]');
        ylabel('Range [m]');
        title(['Scan: ' num2str(itscan) ', Doppler vs Range']);
        xlim([-30 30]);
        ylim([0 50]);

        subplot(1,3,2)
        scatter(rad2deg(soc.az),soc.range,soc.mag,c,'LineWidth',2 );
        xlabel('Azimuth [deg]');
        ylabel('Range [m]');
        title(['Scan: ' num2str(itscan) ', Azimuth vs Range']);
        xlim([-45 45]);
        ylim([0 50]);

        subplot(1,3,3)
        scatter(rad2deg(soc.az),soc.dopp,soc.mag,c,'LineWidth',2 );
        ylabel('Doppler [m/s]');
        xlabel('Azimuth [deg]');
        title(['Scan: ' num2str(itscan) ', Azimuth vs Doppler']);
        ylim([-30 30]);
        xlim([-45 45]);  

        colorbar;
        frame  = getframe(fig);
        im = frame2im(frame);
        drawnow
        [imind,cm] = rgb2ind(im,256);

        if itscan == scannums(1)
            imwrite(imind,cm,[socpath '/detections_mov.gif'],'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,[socpath '/detections_mov.gif'],'gif','WriteMode','append');
        end
        
        drawnow
        pause(.25)
        close all
    end
end
% figure, scatter3(soc.range,rad2deg(soc.az),soc.dopp,20,soc.range,'filled'); colormap jet;
