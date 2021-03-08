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


