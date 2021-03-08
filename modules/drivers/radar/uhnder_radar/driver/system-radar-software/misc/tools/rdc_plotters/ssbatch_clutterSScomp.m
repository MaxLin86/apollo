clear all;
close all;
datapath = 'C:\uhnder\TESTDATA\SVAdata';
ad=dir(datapath)

if ~exist('showPlots','var')
    showPlots = 1;
end
if ~exist('plotdBFS','var')
    plotdBFS = 0;
end
if ~exist('binAxes','var')
    binAxes = 0;
end
% 2dMIMO Rbin cut supported
if ~exist('plotRbinCut','var')
    plotRbinCut = 0;
end

if ~exist('polarplot','var')
    polarplot=1;
end
if ~exist('polarplotrng','var')
    polarplotrng = 38.6;
end
if ~exist('xlimin','var')
    xlimin = [];
end
if ~exist('doPrintInterpPeaks','var')
    doPrintInterpPeaks=0;
end
if ~exist('dBpeakThresh','var')
    dBpeakThresh = -5;
end
if ~exist('peakCensorRngAzEl','var')
    peakCensorRngAzEl = [0 999 -180 180 -180 180];
end
if ~exist('rangeAdjustm','var')
    rangeAdjustm = 0;
end


if ~exist('polarplotc','var')
    polarplotc=1;
end
if ~exist('polarplotrngc','var')
    polarplotrngc=100;
end
if ~exist('plotdBFS','var')
    plotdBFSc = 2;
end
if ~exist('savePolarPlotc','var')
    savePolarPlotc = 1;
end
if ~exist('doPrintInterpPeaksc','var')
    doPrintInterpPeaksc=0;
end
if ~exist('dBpeakThreshc','var')
    dBpeakThreshc = -5;
end
if ~exist('peakCensorRngAzElc','var')
    peakCensorRngAzElc = [0 999 -180 180 -180 180];
end
if ~exist('rangeAdjustmc','var')
    rangeAdjustmc = 0;
end
if ~exist('plotStaticDetsc','var')
    plotStaticDetsc = 0;
end
if ~exist('plotMovingDetsc','var')
    plotMovingDetsc = 0;
end


count = 0;
outTable_name_rng_ang=[];
for it=3:length(ad)
    ts=[datapath '/' ad(it).name ]
    %     if isdir(ts)
    for its=0
%         try
            
            if ~isempty(strfind(ad(it).name,'stslice.bin'))
             plot_staticSliceImgSD(ts, xlimin, binAxes, plotdBFS, showPlots, plotRbinCut, polarplot,...
                    polarplotrng, doPrintInterpPeaks, dBpeakThresh, peakCensorRngAzEl, rangeAdjustm);
                
                fig=gcf;
                maxstat=max(fig.Children(end).Children(10).ZData(:))
                caxis([maxstat-35 maxstat]);
                % [~, interpMagRng, interpBinRng, interpMagAng, interpBinAng] = plot_staticSliceImgSD([ts '/scan_' num2str(its,'%06d') '_stslice.bin'],[],0,0,2,[40:42],1,30,1,-5);
                % [~,ind]=find(interpBinRng);
                % % [~,ind]=min(abs(interpBinAng));
                % % [~,indA]=max(interpBinAng);
                %
                % for itind=1:length(ind)
                %     count = count + 1;
                %     outTable_name_rng_ang{count,1} = ad(it).name;
                %     outTable_name_rng_ang{count,2} = interpBinRng(ind(itind));
                %     if isfield(interpBinAng,'Az')
                %         outTable_name_rng_ang{count,3} = interpBinAng.Az(ind(itind));
                %         outTable_name_rng_ang{count,4} = interpBinAng.El(ind(itind));
                %     else
                %         outTable_name_rng_ang{count,3} = interpBinAng(ind(itind));
                %     end
                % end
                
                pause(.5)
                % plot_clutterImage([ts '/scan_' num2str(its,'%06d') '_clutterimage.bin']);
                
                %         end
                %         [fp1, fp2, fp3] = fileparts(ad(it).name);
                
            end
            
            if ~isempty(strfind(ad(it).name,'clutterimage.bin'))
                clutImgOut = plot_clutterImage(ts, plotStaticDetsc, plotMovingDetsc, ...
                    polarplotc, polarplotrngc, savePolarPlotc, doPrintInterpPeaksc, dBpeakThreshc, peakCensorRngAzElc,rangeAdjustmc);
                fig=gcf;
                maxclut = max(fig.Children(end).Children(10).ZData(:));
                caxis([maxclut-35 maxclut]);
                pause(.5)
            end
            
            
            % SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_stslice')) 'info.json'];
            %         saveas(gcf,[datapath '/' ad(it).name '/stslice.png'])
%         catch
%             
%             disp('oops');
%         end
    end
end
save([datapath '/InterpData.mat'],'outTable_name_rng_ang')