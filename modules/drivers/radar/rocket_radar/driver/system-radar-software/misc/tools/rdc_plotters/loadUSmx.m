% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [ SVDout ] = loadUSmx(filestr)
a = dir(filestr);
debugPlots = 0;
f = fopen(filestr, 'r');
rtrash = fread(f,1 , 'uint16');
dtrash = fread(f, 1 , 'uint16');
Lstart = fread(f, 1 , 'uint16');

fclose(f);
f = fopen(filestr, 'r');

numskewers = a.bytes/(4*2+(2*4*Lstart^2)+(4*2*Lstart));
for itsk = 1:numskewers
    %         [UUoutBin, Uexp]=complexDouble2int32(transpose(squeeze(UUout(itsk,:,:))),24); %known transpose for HW
    %         [SSoutBin, Sexp]=complexDouble2int32(squeeze(SSout(itsk,:)),24);
    SVDout.Rbin(itsk) = fread(f,1,'uint16')+1; %ONE BASED, MaTlAB RuLeZ
    SVDout.Dbin(itsk) = fread(f,1, 'uint16')+1; %ONE BASED, MaTlAB RuLeZ
    SVDout.L(itsk) = fread(f, 1,'uint16');
    SVDout.skIndex(itsk) = fread(f, 1, 'uint16');
    tempin = fread(f,2*SVDout.L(itsk)^2,'int32');
    tempin2=complex(tempin(1:2:end),tempin(2:2:end));
    SVDout.U(itsk,:,:)=reshape(tempin2,SVDout.L(itsk),SVDout.L(itsk)).';
    tempin = fread(f,2*SVDout.L(itsk),'int32');
    SVDout.S(itsk,:) =complex(tempin(1:2:end),tempin(2:2:end));
    if debugPlots
        figure, imagesc(squeeze(abs(SVDout.U(itsk,:,:))));
        figure, plot(squeeze(abs(SVDout.S(itsk,:))));
    end
end

fclose(f);


end


