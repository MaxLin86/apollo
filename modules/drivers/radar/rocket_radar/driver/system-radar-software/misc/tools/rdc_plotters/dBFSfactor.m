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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (C) 2016 Uhnder Inc
% Author:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: dBFSfactor()
%
% Option:
%
% Input:     None
%
% Output:    Many global structs
%
% Calls:
%
%
% Description: This function is called from samSim.m, calculates dependent
%               parameters used elsewhere in the sim
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dBFSfac1, dBFSfac2, dBFSfac3] = dBFSfactor(pulsedMode,PMCW, antenna, digital_BE, scanData)

if exist('scanData','var')
    RSUbits = 3;
    PreCorrelatorScaling = (2^(digital_BE.adcOutputBitDepth+RSUbits-1)-1);
    CodeScaling = 2^(11-1)-1;
else
    RSUbits = 0;
    PreCorrelatorScaling = (2^(digital_BE.adcOutputBitDepth+RSUbits-1)-1);
    CodeScaling = 2^0;
end
    
%RDC 1 factor
if pulsedMode
    %fixme for RDC1 load
    cpp = (PMCW.Lc/2)/ PMCW.txPumpWidth;
    dBFSfac1 = [cpp:cpp:PMCW.R*cpp].' * PMCW.M * PMCW.G * PMCW.K * PreCorrelatorScaling * CodeScaling;
else
    % dBFS adjustment calculation for RDC1 RDC2 RDC3
    dBFSfac1 = ones(PMCW.R,1).* PMCW.Lc * PMCW.M * PMCW.G * PMCW.K * PreCorrelatorScaling * CodeScaling;
end
dBFSfac1 = dBFSfac1;

%RDC 2 factor
dBFSfac2 = dBFSfac1 * PMCW.N;

%RDC 3 factor
dBFSfac3 = dBFSfac2 * antenna.fullyFilled; %fixme use fully filled

dBFSfac1 = mag2db(dBFSfac1);
dBFSfac2 = mag2db(dBFSfac2);
dBFSfac3 = mag2db(dBFSfac3);