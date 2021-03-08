adir = 'X:\systems\SabineBringUp\2018\Mar8\Waymo\DataCollection\Set1_HorizontalSensitivityResolvabilityTest\'

fdir = dir(adir)

for itf = 3:length(fdir)
    
%     try
        plot_clutterImage_continuous([adir fdir(itf).name],315,0)
%     catch
%         disp(['error on ' fdir(itf).name ]);
%     end
    fclose all
    close all
    
end