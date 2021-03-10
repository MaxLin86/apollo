% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function txt = myupdatefcn(empt,event_obj)
% Customizes text of data tips
decpts=2;
pos = get(event_obj,'Position');
txt = {['Rng_m: ',num2str(round(pos(2), decpts))],...
    ['Dop_mps: ',num2str(round(pos(1), decpts))],...
    ['Amp_dB: ',num2str(round(pos(3), decpts))]};
