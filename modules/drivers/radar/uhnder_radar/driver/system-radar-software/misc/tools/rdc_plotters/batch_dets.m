adir = 'C:\uhnderOLD\TESTDATA\elevTest\';
fdir = dir(adir)

idxf = 1;
snrmaxlist = [];
snrminlist = [];
snrmeanlist = [];
snrstdlist = [];
snrnumdets = [];

cnt=1;
for itf = 3:length(fdir)
    allranges = [];
allsnrs = [];
allstat = [];
    if isdir([adir fdir(itf).name])
        tdets = loadAllDets([adir fdir(itf).name],0,0);
        idxf = idxf+1;
        for itsc = 1:numel(tdets)
            if(~isempty(tdets{itsc}))
                snrmaxlist = [snrmaxlist max(tdets{itsc}.snr)];
                snrminlist = [snrminlist min(tdets{itsc}.snr)];
                snrmeanlist = [snrmeanlist mag2db(mean(db2mag(tdets{itsc}.snr)))];
                snrstdlist = [snrstdlist std(tdets{itsc}.snr)];
                snrnumdets = [snrnumdets numel(tdets{itsc}.range)];
                allranges = [allranges tdets{itsc}.range];
                allsnrs = [allsnrs tdets{itsc}.snr];
                allstat = [allstat tdets{itsc}.staticFlag];
                c_allranges{cnt}=allranges;
                c_allsnrs{cnt} = allsnrs;
                c_allstat{cnt} = allstat;
            end
        end
            cnt = cnt+1;

    end
    snrmaxlist = [snrmaxlist -1000];
    snrminlist = [snrminlist -1000];
    snrmeanlist = [snrmeanlist -1000];
    snrstdlist = [snrstdlist 0];
    snrnumdets = [snrnumdets -1000];

    fclose all
    close all
    
end
figure, hold,
plot(snrmaxlist,'r-');
plot(snrnumdets,'k-');
plot(snrmeanlist,'g-');
plot(snrmeanlist-snrstdlist,'b');
plot(snrmeanlist+snrstdlist,'b');
plot(snrminlist,'r-');
