% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
%rx_num = 8 - Plots all RX
%rx_num = 0-7 plots RX(0-7) specified
function rx = plot_ADCRawMode(ADCbinFile,Lc,Nrx,Npris,sampsPerChip,Fs,rx_num)

rx = plot_ADCdata(ADCbinFile,Lc,Nrx,Npris,sampsPerChip,0,1,Fs);

figure
if rx_num == 8
plot(real(rx),'*-')
hold on
plot(imag(rx),'.-')
legend( { 'I0' ,'I1','I2','I3','I4','I5','I6','I7','Q0', 'Q1','Q2','Q3','Q4','Q5','Q6','Q7' } )
xlim([0 Lc])
else
% plot specific RX
plot(real(rx(:,rx_num+1)),'*-')
hold on
plot(imag(rx(:,rx_num+1)),'.-')
legend( { 'I' ,'Q' } )
xlim([0 Lc])

end








