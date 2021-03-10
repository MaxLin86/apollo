% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function txt = RDC1updatefcn(empt,event_obj)
% Customizes text of data tips
decpts=2;
pos = get(event_obj,'Position');
txt = {['PRI(1b): ',num2str(round(pos(1), decpts))],...
    ['RangeBin(1b): ',num2str(round(pos(2), decpts))],...
    ['Mag(dB): ',num2str(round(pos(3), decpts))]};
