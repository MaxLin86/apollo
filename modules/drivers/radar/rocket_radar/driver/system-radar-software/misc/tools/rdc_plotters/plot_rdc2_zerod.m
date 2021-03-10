% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function zerod_mat = plot_rdc2_zerod(filename, num_rbins, showPlots)

if ~exist('filename','var')
    error('Usage:  plot_rdc2_zerod(filename, num_rbins)');
end
if ~exist('num_rbins','var')
    num_rbins = 128;
end
if ~exist('showPlots','var')
    showPlots = 1;
end

%string to number conversion needed for executable input parameters 
if ischar(num_rbins)
     num_rbins=str2num(num_rbins);
end
if ischar(showPlots)
     showPlots=str2num(showPlots);
end

num_vrx = 64;
%zerod_mat = zeros(num_vrx,num_rbins);


fp = fopen(filename, 'rb');

raw_zerod = fread(fp, num_rbins*num_vrx*2, 'int64');
czerod = complex(raw_zerod(1:2:end), raw_zerod(2:2:end));

zerod_mat = reshape(czerod, num_vrx , num_rbins);
if showPlots
    surf(mag2db(abs(zerod_mat)));
end
fclose(fp);
