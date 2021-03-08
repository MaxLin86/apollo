function batch_contCI(adir)
    %adir = 'X:\systems\SabineBringUp\2018\Mar8\Waymo\DataCollection\Set1_HorizontalSensitivityResolvabilityTest\'
    %fdir = dir(adir);
    all_files = dir(adir);
    all_dir = all_files([all_files(:).isdir]);

    for itf = 3:length(all_dir)
        %plot_clutterImage_continuous([adir fdir(itf).name],1,0,1)
        plot_clutterImage_continuous([all_dir(itf).folder '/' all_dir(itf).name],1,0)
        close all
    end
end