function plot_clutImg_movie(socpath,scannums)
%scannums=1:207;
%socpath='~/Documents/Uhnder/Bitbucket/sabine-radar-sw/build-x86';
    if ~exist('loadjson','file')
         addpath('../jsonlab/');
    end
    for itscan=scannums
        fig=figure;
        set(fig,'Position',[5 40 900 850]);
        %plot_clutterImage([socpath '/socsim_' num2str(itscan,'%06d') '_clutterimage.bin'])

        plot_clutterImageSD([socpath '/scan_' num2str(itscan,'%06d') '_clutterimage.bin']);

        xlabel('X [pixels]');
        ylabel('Y [pixels]');
        title(['Scan: ' num2str(itscan) ', Clutter Image']);
        axis equal;

        frame  = getframe(fig);
        im = frame2im(frame);
        drawnow
        [imind,cm] = rgb2ind(im,256);

        if itscan == scannums(1)
            imwrite(imind,cm,[socpath '/clutterImage_mov.gif'],'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,[socpath '/clutterImage_mov.gif'],'gif','WriteMode','append');
        end
        drawnow
        pause(.25)
        close all
    end
end