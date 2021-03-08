% START_SOFTWARE_LICENSE_NOTICE
% -------------------------------------------------------------------------------------------------------------------
% Copyright (C) 2016-2017 Uhnder, Inc. All rights reserved.
% This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
% under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
% develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
% strictly prohibited.
% Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
% Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
% THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
% BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
% REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
% TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
% PROGRAM IS STRICTLY PROHIBITED.
% THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
% WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
% THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
% IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
% PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
% SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
% WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
% -------------------------------------------------------------------------------------------------------------------
% END_SOFTWARE_LICENSE_NOTICE
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

