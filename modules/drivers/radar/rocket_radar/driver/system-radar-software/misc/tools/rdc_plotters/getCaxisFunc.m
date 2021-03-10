% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

function [ cax ] = getCaxisFunc(method,Caxdata,vars)

switch method
    case 1
        %clutterImageSD
        c = colormap(jet(400));
        
        if vars.rangeNorm && vars.useSVAcolormap
            c(1,:) = [0 0 .4];
            colormap([c(1,:);c(25:end,:)]);
            % caxis([c1 c2]);
            % used for DeNoising plots for CES and Ford
            cmin = median(Caxdata(:))+7 ;
            cmax = max(max(Caxdata(:))-0,median(Caxdata(:))+vars.CaxisRngDb);
            cmax = max(cmax,cmin+30);
        else
            cmin = median(Caxdata(:))-4 ;
            cmax = max(max(Caxdata(:)),median(Caxdata(:))+vars.CaxisRngDb);
        end
    case 2
        %clutterImageSD old code path        
        if (vars.rangeNorm)
            cmin = min(Caxdata(:));
        else
            cmin = min(min(Caxdata(:,3:end)));
        end
        cmax = max(Caxdata(:));
        if cmin>=cmax,
            cmax=cmin+.1;
        end % make sure min<max
    case 3
        % makePolarplotFromRA.m
        if vars.plotrng <= 80 % JJP hax skip agressive caxis on long range, peaks disappear
            cmin = min(vars.cax1+30,vars.cax2-11) 
            cmax = vars.cax2;                  % CAXIS for BSCOPE
        else
            cmin = vars.cax1;
            cmax = vars.cax2;
        end
    case 4
        % makePolarplotFromRA.m no SVA
        cmin = median(Caxdata(:))+1;
        cmax =  max(Caxdata(:))-3;
    otherwise
        cmin = min(Caxdata(:));
        cmax = max(Caxdata(:));
        
end

cax = [cmin cmax];