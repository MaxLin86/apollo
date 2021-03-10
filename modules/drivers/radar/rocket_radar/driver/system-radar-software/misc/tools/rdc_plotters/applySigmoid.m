% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [cout] = applySigmoid(cin,steepness,midpoint)
% steepness = steepness*10;
midpoint = max(0,min(10,round(midpoint*10)));
x=0:.1:10;
tempmap=1+round((size(cin,1)-1)./(1+exp(-steepness.*(x-midpoint)))); %figure, plot(x,tempmap)
cout=cin(tempmap,:);
