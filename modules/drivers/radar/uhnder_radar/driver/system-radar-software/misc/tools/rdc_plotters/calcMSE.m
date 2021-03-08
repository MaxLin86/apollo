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
 function [ MSEAll, MSETarg ] = calcMSE(set1RDC, set2RDC, targbin )
% return MSE over the designated target bin, targbin
% targbin is one based, and is of length ndims of the data
% if a targbin value is 0 then all indices in that dimesion are processed
% this will work on RDC1, RDC2 or RDC3 data

% exponents have already been applied to data, by the plot function, all
% that is needed is applied is the fudge factor scalebits

normalizeMSEtoData = 0; % if 1 then MSE is normalized to the sim value
                        % if 0 then MSE is normalized only by the number of elements

% some hardcoded baddies
antenna_vrxPos = [-0.0578713289177215,-0.0521790670569620,-0.0502816464367089,-0.0464868051962025,-0.0445893845759494,-0.0426919639556962,-0.0407945433354430,-0.0388971227151899,-0.0369997020949367,-0.0351022814746835,-0.0332048608544304,-0.0313074402341772,-0.0294100196139240,-0.0275125989936709,-0.0256151783734177,-0.0237177577531646,-0.0218203371329114,-0.0199229165126582,-0.0180254958924051,-0.0161280752721519,-0.0142306546518987,-0.0123332340316456,-0.0104358134113924,-0.00853839279113924,-0.00664097217088607,-0.00474355155063291,-0.00284613093037974,-0.000948710310126576,0.000948710310126583,0.00284613093037975,0.00474355155063291,0.00664097217088608,0.00853839279113924,0.0104358134113924,0.0123332340316456,0.0142306546518987,0.0161280752721519,0.0180254958924051,0.0199229165126582,0.0218203371329114,0.0237177577531646,0.0256151783734177,0.0275125989936709,0.0294100196139241,0.0313074402341772,0.0332048608544304,0.0351022814746835,0.0369997020949367,0.0388971227151899,0.0407945433354431,0.0426919639556962,0.0445893845759494,0.0464868051962025,0.0502816464367089,0.0521790670569620,0.0578713289177215,0.0787429557405063,0.0844352176012658,0.0863326382215190,0.0901274794620253,0.0920249000822785,0.0958197413227848,0.0977171619430380,0.103409423803797];

dim1bins = targbin(1);
dim2bins = targbin(2);
dim3bins = targbin(3);
if targbin(1)==0
    dim1bins = 1:size(set1RDC,1);
end
if targbin(2)==0
    dim2bins = 1:size(set1RDC,2);
end
if targbin(3)==0
    dim3bins = 1:size(set1RDC,3);
end


figure,
plotComplex = (any(imag(set1RDC)~=0) | any(imag(set2RDC)~=0));
% If the data is complex then do phase analysis and take abs for remaining magnitude analyis
if plotComplex
    
    
    % take target bins
    phaseRDCset1 = (angle(set1RDC(dim1bins,dim2bins,dim3bins)));
    phaseRDCset2 = (angle(set2RDC(dim1bins,dim2bins,dim3bins)));
    rawPhaseError_rad = abs(phaseRDCset1-phaseRDCset2);
    
    % calc phase
    diffAngRDCset1 = diff(phaseRDCset1);
    diffAngRDCset2= diff(phaseRDCset2);
    
    % RDCset1 force wrap down
    subph = 0;
    for itph = 2:length(phaseRDCset1)
        if diffAngRDCset1(itph-1) >0
            subph = subph -2*pi;
        end
        phaseRDCset1(itph) = phaseRDCset1(itph)+subph;
    end
    % RDCset2 force wrap down
    subph = 0;
    for itph = 2:length(phaseRDCset2)
        if diffAngRDCset2(itph-1) >0
            subph = subph -2*pi;
        end
        phaseRDCset2(itph) = phaseRDCset2(itph)+subph;
    end
%     % RDCset1 force wrap up
%     subph = 0;
%     for itph = 2:length(phaseRDCset1)
%         if diffRDCset1(itph-1)<0
%             subph = subph+2*pi;
%         end
%         phaseRDCset1(itph) = phaseRDCset1(itph)+subph;
%     end
%     % RDCset2 force wrap up
%     subph = 0;
%     for itph = 2:length(phaseRDCset2)
%         if diffRDCset2(itph-1)<0
%             subph = subph+2*pi;
%         end
%         phaseRDCset2(itph) = phaseRDCset2(itph)+subph;
%     end
    subplot(2,2,2), plot(antenna_vrxPos,180/pi*((phaseRDCset1)),'.-', antenna_vrxPos,180/pi*(phaseRDCset2),'o-'),
    title(['Phase of target bin, mean error: ' num2str(180/pi*(mean(rawPhaseError_rad))) 'deg, max error: ' num2str(num2str(180/pi*(max(rawPhaseError_rad)))) 'deg']);
    xlabel('Element position'), ylabel('Phase (deg)')
    legend('RDCset1 RDC','RDCset2 RDC');
    %convert to mag for analysis
    set1RDC = abs(set1RDC);
    set2RDC = abs(set2RDC);
    
end

rawdelta = set2RDC-set1RDC; % subtract from test data
censor = (isinf(rawdelta) | set1RDC==0);
if sum(censor(:))
   disp(['Censored out ' num2str(sum(censor(:))) ' bins, ' num2str(100*sum(censor(:))/numel(censor(:))) '% of data']) 
end
% censored out any infs
rawdeltacens=rawdelta(~censor);
reference = set1RDC(~censor);
if ~normalizeMSEtoData
    reference = numel(set1RDC(~censor));
end
% MSEAll=sum(rawdeltacens(:).^2)/sum(reference.^2);

% Calc MSE based on normalized reference data
MSEAll=sum((rawdeltacens(:)./reference(:)).^2);

rawdeltatarg = rawdelta(dim1bins,dim2bins,dim3bins);
censor = (isinf(rawdeltatarg) | set1RDC(dim1bins,dim2bins,dim3bins)==0);
if  sum(censor(:))
   disp(['Censored out ' num2str(sum(censor(:))) ' bins, ' num2str(100*sum(censor(:))/numel(censor(:))) '% of data']) 
end
rawdeltacenstarg=rawdeltatarg(~censor);
temp=set1RDC(dim1bins,dim2bins,dim3bins);
reference = temp(~censor);
if ~normalizeMSEtoData
    reference = numel(temp(~censor));
end
% MSETarg=sum(rawdeltacenstarg(:).^2)/sum(reference(:).^2);

% Calc MSE based on normalized reference data
MSETarg=sum((rawdeltacenstarg(:)./reference(:)).^2);

if sum(targbin==0)<2 % if we have 1d targ make this plot
 if plotComplex, subplot(2,2,[1 3]), else subplot(2,1,1); end
 plot(mag2db(abs(squeeze(set1RDC(dim1bins,dim2bins,dim3bins)))),'.-')
    hold, plot(mag2db(abs(squeeze(set2RDC(dim1bins,dim2bins,dim3bins)))),'o-');
title(['Range Bin ' num2str(dim1bins) ' RDC comparison, Skewer MSE: ' num2str(MSETarg) ]);
    legend('RDCset1','RDCset2');
    ylabel('Scaled Magnitude (dB)')
    xlabel('Vrx, Spatial ordering')
    if plotComplex, subplot(2,2,4), else subplot(2,1,2); end
    plot((abs((rawdelta(dim1bins,dim2bins,dim3bins))))); % no norm./angle_rdc_RDCset1(Rbin,:,Dbin))));%,...
ylabel('RDCset2-RDCset1, Linear'), title('Delta Magnitude, Linear')

grid on;
xlabel('Vrx, Spatial ordering')
end
figure, mesh((squeeze(mean(abs(rawdelta),1)))), title('Linear RDC Error, Mean Across Range'), xlabel('Doppler Bin'), ylabel('Angle or Vrx Bin'), zlabel('Mean Linear Error'), colorbar;
figure, mesh((squeeze(mean(abs(rawdelta),2)))), title('Linear RDC Error, Mean Across Angle or Vrx'), xlabel('Doppler Bin'), ylabel('Range Bin'), zlabel('Mean Linear Error'), colorbar;
figure, mesh((squeeze(mean(abs(rawdelta),3)))), title('Linear RDC Error, Mean Across Doppler'), ylabel('Range Bin'), xlabel('Angle or Vrx Bin'), zlabel('Mean Linear Error'), colorbar;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
