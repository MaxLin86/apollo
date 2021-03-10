% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [det] = loadDetectionData(filestr,numdets)
a = dir(filestr);

if (size(a,1) > 0)
    if numdets>=0
        oldDetFormat = a.bytes/64 == numdets;
    else
        oldDetFormat=1;
    end
    if oldDetFormat
        numDets = a.bytes/64;
        assert(mod(a.bytes,64)==0,'unexpected file size');
    else
        numDets = a.bytes/44;
        assert(mod(a.bytes,44)==0,'unexpected file size');
    end
    f = fopen(filestr, 'r');

    det.range = [];
    det.az = [];
    det.el = [];
    det.dopp = [];
    det.mag = [];
    det.rcs = [];
    det.xyz = [];
    det.snr = [];
    det.flags = [];

    if f>0
        for itd = 1:numDets
            det.range(itd) = fread(f, 1, 'float');
            det.az(itd) = fread(f, 1, 'float'); %az
            det.el(itd) = fread(f, 1, 'float'); %el
            det.dopp(itd) = fread(f, 1, 'float');
            det.mag(itd) = fread(f, 1,'float');
            det.snr(itd) = fread(f, 1, 'float');
            det.rcs(itd) = fread(f, 1, 'float'); %TODO RCS calculation
            det.xyz(3*(itd-1)+1:3*(itd-1)+3) = fread(f, 3, 'float'); %TODO XYZ
            det.flags(itd) = fread(f, 1, 'uint32');
            if oldDetFormat
                fread(f, 5, 'uint32');
            end
        end
        det.staticFlag=logical(bitget(int32(det.flags),1));
        det.xyzmx = reshape(det.xyz,3,[]);

    else
        disp(['*** Cant find Dets data file ***']);
    end
    fclose(f);
else
    det = [];
end
end


