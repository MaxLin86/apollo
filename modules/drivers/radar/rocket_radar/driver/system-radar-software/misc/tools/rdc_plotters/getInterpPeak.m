% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2016 Uhnder Inc
% Author: J. Preussner
% This function performs quadratic interpolation max 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage
%       getInterpPeak
% Option:
%       Input:
%           peaksIn
%       Output:
%           peaksOut
% Calls: 
%       getInterpPeak
%
% Description: 
%       Calculate log-based quadratic interpolation of peak in range doppler and angle
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ ye, xe ] = getInterpPeak( mags, bins )

yl = mag2db(mags(1));
yc = mag2db(mags(2));
yu = mag2db(mags(3));
xl = bins(1);
xc = bins(2);
xu = bins(3);

  d2 = 2*((yu-yc)/(xu-xc)-(yl-yc)/(xl-xc))/(xu-xl); 
  d1 = (yu-yc)/(xu-xc) - 0.5*d2*(xu-xc); 
  if(isnan(d2))
      %degenerate if NaN
      d2=0;
  end
  if (d2)  
    xe = xc - d1/d2; 
    ye = db2mag(yc + 0.5*d1*(xe-xc)); 
  else  % Degenerate d2 
    xe = xc; 
    ye = db2mag(yc);  
    warning('Degenerate peak interpolation, returning center bin'); 
  end
  if ((xe>xu)||(xe<xl)) 
      warning('Peak bin outside of input bins'); % Reliability test 
  end  
end

