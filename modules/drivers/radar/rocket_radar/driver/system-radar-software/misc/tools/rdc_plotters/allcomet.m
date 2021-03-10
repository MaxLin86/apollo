% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
load('C:\uhnder\TESTDATA\Sep6\vrx_az_only_70deg0p1perstep_TX9.mat')


antenna.map = [1,17,2,33,18,3,49,34,19,4,57,50,35,20,9,58,51,25,36,10,59,41,52,26,11,60,42,27,12,5,43,28,21,6,44,37,22,7,53,38,23,8,61,54,39,24,13,62,55,29,40,14,63,45,56,30,15,64,46,31,16,47,32,48];
RxMajVrx(:,antenna.map) = vrx;
figure;
for itVrx = 49:56
subplot(2,4,itVrx-48)
scatter(real(RxMajVrx(:,itVrx)),imag(RxMajVrx(:,itVrx)),30,1:70,'filled')
axis equal;
title(['RxmVrx ' num2str(itVrx) ' Tx' num2str(ceil(itVrx/8)) ' Rx' num2str(mod((itVrx)-1,8)+1)]);
end
colormap jet

load('C:\uhnder\TESTDATA\Sep6\vrx_az_only_70deg0p1perstep_TX11.mat')

antenna.map = [1,17,2,33,18,3,49,34,19,4,57,50,35,20,9,58,51,25,36,10,59,41,52,26,11,60,42,27,12,5,43,28,21,6,44,37,22,7,53,38,23,8,61,54,39,24,13,62,55,29,40,14,63,45,56,30,15,64,46,31,16,47,32,48];
RxMajVrx(:,antenna.map) = vrx;
figure;
for itVrx = 57:64
subplot(2,4,itVrx-56)
scatter(real(RxMajVrx(:,itVrx)),imag(RxMajVrx(:,itVrx)),30,1:70,'filled')
axis equal;
title(['RxmVrx ' num2str(itVrx) ' Tx' num2str(ceil(itVrx/8)) ' Rx' num2str(mod((itVrx)-1,8)+1)]);
end
colormap jet

load('C:\uhnder\TESTDATA\Sep6\vrx_az_only_70deg0p1perstep.mat')
antenna.map = [1,17,2,33,18,3,49,34,19,4,57,50,35,20,9,58,51,25,36,10,59,41,52,26,11,60,42,27,12,5,43,28,21,6,44,37,22,7,53,38,23,8,61,54,39,24,13,62,55,29,40,14,63,45,56,30,15,64,46,31,16,47,32,48];
RxMajVrx(:,antenna.map) = vrx;
figure;
for itVrx = 1:64
subplot(8,8,itVrx)
scatter(real(RxMajVrx(:,itVrx)),imag(RxMajVrx(:,itVrx)),30,1:70,'filled')
axis equal;
%title(['RxmVrx ' num2str(itVrx) ' Tx' num2str(ceil(itVrx/8)) ' Rx' num2str(mod((itVrx)-1,8)+1)]);
end
colormap jet
colorbar