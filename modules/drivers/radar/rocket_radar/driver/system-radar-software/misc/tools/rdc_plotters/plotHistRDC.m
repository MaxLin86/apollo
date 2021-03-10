% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [histdata] = plotHistRDC(dataIn,redhist)
figure;
histdata=mag2db(abs(dataIn(dataIn>0)));
if redhist
    [linhist bincenters]=hist(histdata,50);
    if(max(linhist<500))
        % dont bother with redhist if there are less that 500 hits
        hist(histdata,50);
        ylabel 'Hits'
    else
        bar(bincenters,linhist)
        redthresh= 10^sum(10.^(0:10) < max (linhist))/1000;
        linhistred = zeros(size(linhist)); linhistred(linhist>0 & linhist<max(linhist)/redthresh) = linhist(linhist>0 & linhist<max(linhist)/redthresh)*redthresh;
        hold; bar(bincenters,linhistred,'r')
        ylabel(['Hits (Red = hits x ' num2str(redthresh) ') ']);
    end
else
    hist(histdata,50);
    ylabel 'Hits'
end