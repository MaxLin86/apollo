% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function ydb = mag2db(y) %#codegen
%MAG2DB  Magnitude to dB conversion.
%
%   YDB = MAG2DB(Y) converts magnitude data Y into dB values.
%   Negative values of Y are mapped to NaN.
%
%   See also DB2MAG.

%   Copyright 1986-2011 The MathWorks, Inc.
y(y<0) = NaN;
ydb = 20*log10(y);
