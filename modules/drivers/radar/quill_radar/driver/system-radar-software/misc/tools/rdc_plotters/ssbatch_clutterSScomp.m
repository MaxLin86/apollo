function [clutImgOut] = ssbatch_clutterSScomp(datapath)
    if ~exist('loadjson','file')
        addpath('../jsonlab/');
    end
    %datapath = 'C:\uhnder\TESTDATA\SVAdata';
    ad=dir(datapath);
    outTable_name_rng_ang=[];
    for it=3:length(ad)
        ts=[datapath '/' ad(it).name ];
        for its=0
                if ~isempty(strfind(ad(it).name,'stslice.bin'))
                    %plot_staticSliceImgSD(ts, xlimin, binAxes, plotdBFS, showPlots, plotRbinCut, polarplot,...
                        %polarplotrng, doPrintInterpPeaks, dBpeakThresh, peakCensorRngAzEl, rangeAdjustm);
                    plot_staticSliceImgSD(ts);
                    fig=gcf;
                    pause(.5)
                end

                if ~isempty(strfind(ad(it).name,'clutterimage.bin'))
                    %clutImgOut = plot_clutterImage(ts, plotStaticDetsc, plotMovingDetsc, ...
                        %polarplotc, polarplotrngc, savePolarPlotc, doPrintInterpPeaksc, dBpeakThreshc, peakCensorRngAzElc,rangeAdjustmc);
                    clutImgOut = plot_clutterImageSD(ts);
                    fig=gcf;
                    num_graphic_arr = numel(fig.Children.Children);
                    maxclut = max(fig.Children(end).Children(num_graphic_arr).ZData(:));
                    caxis([maxclut-35 maxclut]);
                    pause(.5)
                end
        end
    end
    save([datapath '/InterpData.mat'],'outTable_name_rng_ang')
end
