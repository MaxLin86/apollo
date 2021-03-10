%datapath = 'X:\software\jpb\data_2017_09_11_outdoors_at_airfield\_FIELD_2D_resolution_vertical_250mcps_X25m';
function ssbatch(datapath)

    if ~exist('loadjson','file')
        addpath('../jsonlab/');
    end

    ad = dir(fullfile(datapath, '*_stslice.bin'));

    count = 0;
    outTable_name_rng_ang=[];
    for it=1:numel(ad)
        ts=[datapath '/' ad(it).name ];
            for its=0
                [~, interpMagRng, interpBinRng, interpMagAng, interpBinAng] = plot_staticSliceImgSD(ts);
                [~,ind]=find(interpBinRng);
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
            end
    end
    save([datapath '/InterpData.mat'],'outTable_name_rng_ang')
end
