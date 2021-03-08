datapath = 'X:\software\jpb\data_2017_09_11_outdoors_at_airfield\_FIELD_2D_resolution_vertical_250mcps_X25m';
ad=dir(datapath)

count = 0;
outTable_name_rng_ang=[];
for it=3:length(ad)
    ts=[datapath '/' ad(it).name ]
    if isdir(ts)
    for its=0
%     try
%         if ~isempty(strfind(ad(it).name,'stslice.bin'))
%             plot_staticSliceImgSD([ts '/scan_000000_stslice.bin']);
[~, interpMagRng, interpBinRng, interpMagAng, interpBinAng] = plot_staticSliceImgSD([ts '/scan_' num2str(its,'%06d') '_stslice.bin'],[],0,0,2,[40:42],1,30,1,-5);
[~,ind]=find(interpBinRng);
% [~,ind]=min(abs(interpBinAng));
% [~,indA]=max(interpBinAng);

for itind=1:length(ind)
    count = count + 1;
    outTable_name_rng_ang{count,1} = ad(it).name;
    outTable_name_rng_ang{count,2} = interpBinRng(ind(itind));
    if isfield(interpBinAng,'Az')
        outTable_name_rng_ang{count,3} = interpBinAng.Az(ind(itind));
        outTable_name_rng_ang{count,4} = interpBinAng.El(ind(itind));
    else
        outTable_name_rng_ang{count,3} = interpBinAng(ind(itind));
    end
end

pause(.5)
close all;         
% plot_clutterImage([ts '/scan_' num2str(its,'%06d') '_clutterimage.bin']);

%         end
%         [fp1, fp2, fp3] = fileparts(ad(it).name);

    end
% SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_stslice')) 'info.json'];
%         saveas(gcf,[datapath '/' ad(it).name '/stslice.png'])
%     catch
        
%         disp('oops');
%     end
end
end
save([datapath '/InterpData.mat'],'outTable_name_rng_ang')