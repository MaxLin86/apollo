% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [clutImg, exponent] = plot_clutterImage2D(filestr)

f = fopen(filestr, 'r');
if f>0
    sliceNum = fread(f, 1, 'uint16');
    xsize = fread(f, 1, 'uint16');
    ysize = fread(f, 1, 'uint16');
    depthBytes = fread(f, 1, 'uint16');
    exponent = fread(f, 1, 'uint16');
    reserved = fread(f, 1, 'uint16');
    pixel_meters = fread(f, 1, 'float');
    if depthBytes==1
        datastr = 'uint8';
    elseif depthBytes==2
        datastr = 'uint16';
    else
        error('Unknown depth')
    end
    clutImg = fread(f, xsize*ysize, datastr);
    clutImg = (fliplr(reshape(clutImg,xsize,ysize)));
    fclose(f);
    dbImg = mag2db(1+clutImg);
    % Calculate the lower threshold for color axis
    H = histcounts(dbImg,1:100);      % Calculate histogram of the image pixels
    H = H(2:end);               % Ignore zero bin of the histogram
    thresh = find(cumsum(H,'reverse')/sum(H) < 0.15, 1)+4;  % Take the brightest 10% of pixels

    figure;
    surf(1:xsize,1:ysize,dbImg);
    axis equal;
    axis tight;
    view([90 90 ])
    caxis([29.3 40])
    
else
    disp('*** Cant find ClutterImage data file ***');
end
end

