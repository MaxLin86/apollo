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
 function [ SNRpow, SigPow ] = calcPallSNR(palRDC, targbin, scaleBits )
% return SNR over the designated target bins, targbin
% targbin is one based, and is of length ndims of the data
% if a targbin value is 0 then all indices in that dimesion are processed
% this will work on RDC1, RDC2 or RDC3 data
% Scale bits are a temp field while we tune the Sabine Range doppler processing
%   it is used to scale down the samSim data to palladium levels

% exponents have already been applied to data, by the plot function, all
% that is needed is applied is the fudge factor scalebits


% some hardcoded baddies
antenna_map = [1,9,2,17,10,3,25,18,11,4,26,19,12,5,27,20,13,6,28,21,14,33,29,22,41,34,30,49,42,35,57,50,43,36,58,51,44,37,59,52,45,38,60,53,46,7,61,54,15,8,62,23,16,31,24,32,39,47,40,55,48,63,56,64];
% antenna_vrxPos = [-0.0578713289177215,-0.0521790670569620,-0.0502816464367089,-0.0464868051962025,-0.0445893845759494,-0.0426919639556962,-0.0407945433354430,-0.0388971227151899,-0.0369997020949367,-0.0351022814746835,-0.0332048608544304,-0.0313074402341772,-0.0294100196139240,-0.0275125989936709,-0.0256151783734177,-0.0237177577531646,-0.0218203371329114,-0.0199229165126582,-0.0180254958924051,-0.0161280752721519,-0.0142306546518987,-0.0123332340316456,-0.0104358134113924,-0.00853839279113924,-0.00664097217088607,-0.00474355155063291,-0.00284613093037974,-0.000948710310126576,0.000948710310126583,0.00284613093037975,0.00474355155063291,0.00664097217088608,0.00853839279113924,0.0104358134113924,0.0123332340316456,0.0142306546518987,0.0161280752721519,0.0180254958924051,0.0199229165126582,0.0218203371329114,0.0237177577531646,0.0256151783734177,0.0275125989936709,0.0294100196139241,0.0313074402341772,0.0332048608544304,0.0351022814746835,0.0369997020949367,0.0388971227151899,0.0407945433354431,0.0426919639556962,0.0445893845759494,0.0464868051962025,0.0502816464367089,0.0521790670569620,0.0578713289177215,0.0787429557405063,0.0844352176012658,0.0863326382215190,0.0901274794620253,0.0920249000822785,0.0958197413227848,0.0977171619430380,0.103409423803797];

if ~exist('scaleBits','var')
    scaleBits=0;
end

% % scaling factors to get palladium normalized with SamSim
% downbits = transpose(scaleBits)-4;
% allbits=18; % fudge factor to equalize data with palladium 
% downbits = ones(128,1)*5; % bits to scale down SamSim back to zero exponent
% downbits(41)=14; downbits(40)=8; downbits(81)=6;
% allbits=13; % fudge factor to equalize data with palladium
% scaled=allbits-downbits;

dim1bins = targbin(1);
dim2bins = targbin(2);
dim3bins = targbin(3);
if targbin(1)==0
    dim1bins = 1:size(palRDC,1);
end
if targbin(2)==0
    dim2bins = 1:size(palRDC,2);
end
if targbin(3)==0
    dim3bins = 1:size(palRDC,3);
end
sigmask = logical(zeros(size(palRDC)));
sigmask(dim1bins,dim2bins,dim3bins) = 1;

noisemask = logical(ones(size(palRDC)));
buffbins=4;
noisemask(dim1bins,:,max(1,dim3bins-buffbins):min(size(palRDC,3),dim3bins+buffbins)) = 0;

% scale samsim data down to pal level and floor it to make it cheezier  
% samRDC = floor(samRDC/(2^scaleBits));

isComplex = any(imag(palRDC(:))~=0);
% If the data is complex then do phase analysis and take abs for remaining magnitude analyis
if isComplex
    
    % Convert Palladium data to Spatial ordering
    if length(antenna_map)==size(palRDC,2)
        palRDC = palRDC(:,antenna_map,:);
    end
    
    %convert to mag for analysis
    palRDC = abs(palRDC);
    
end

% Calc MSE based on normalized reference data
% sMx = palRDC(dim1bins,dim2bins,dim3bins);
% nMx = palRDC(setxor(1:size(palRDC,1),dim1bins),dim2bins,dim3bins);
sMx = palRDC(sigmask&(palRDC>0));
nMx = palRDC(noisemask&(palRDC>0));

SigPow=(sum(sMx(:))/numel(sMx)).^2;
NoisePow= (sum(nMx(:))/numel(nMx)).^2;
SNRpow = SigPow/NoisePow;
