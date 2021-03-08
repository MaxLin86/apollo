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
% Author: J. Preussner
% This script creates and tests the very simple data checker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage
%
% Input:
%   testvector (struct):
% Description:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Pass] = simpleHALchecker(inputStr,testDataPath,goldenDataPath,fileNamePrefix)
% [Pass] = simpleHALchecker('create','X:/software/jonathan/goldedOutputs/','X:/software/jonathan/goldedOutputs/')

noiseDeltaThresh_dB = 0.1;
sigDeltaThresh_lin = 1000;

if nargin==0
    disp('Simple HAL Data Checker')
    disp('Input arguments: testString (required), testDataPath (optional),');
    disp('goldenDataPath (optional),fileNamePrefix(optional)');
    disp('Available testString inputs: rdc1, rdc2, sprdc2, sprdc2ch, sprdc3, stslice, hist')
    disp('Return Code will be 0 if the test passes and 1 if it failed')
    Pass = false; 
    return;
elseif nargin==1
    goldenDataPath='./';
    testDataPath ='./';
    fileNamePrefix = 'scan_000000_';
elseif nargin==2
    goldenDataPath ='./';
    fileNamePrefix = 'scan_000000_';
elseif nargin==3
    fileNamePrefix = 'scan_000000_';
end


switch inputStr
    case 'create'
        [targData_lin.RDC1, noisemean_dB.RDC1] = checkHALdata([goldenDataPath 'scan_000000_rdc1.bin']);
        %[targData_lin.RDC2, noisemean_dB.RDC2] = checkHALdata([goldenDataPath 'scan_000000_rdc2.bin']);
        [targData_lin.spRDC2ch, noisemean_dB.spRDC2ch] = checkHALdata([goldenDataPath 'scan_000000_spsumchu.bin']);
        % [targData_lin.spRDC2, noisemean_dB.spRDC2] = checkHALdata([goldenDataPath 'scan_000000_sprdc2.bin']);
        [targData_lin.spRDC3, noisemean_dB.spRDC3] = checkHALdata([goldenDataPath 'scan_000000_spsumu.bin']);
        [targData_lin.stslice, noisemean_dB.stslice] = checkHALdata([goldenDataPath 'scan_000000_stslice.bin']);
        [targData_lin.hist, noisemean_dB.hist] = checkHALdata([goldenDataPath 'scan_000000_hist.bin']);
        save([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
    case 'rdc1'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.RDC1, noisemean_dB.RDC1] = checkHALdata([testDataPath fileNamePrefix 'rdc1.bin']);
        if(all(abs(gold.noisemean_dB.RDC1 - noisemean_dB.RDC1)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.RDC1 - targData_lin.RDC1),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
        
    case 'rdc2'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.RDC2, noisemean_dB.RDC2] = checkHALdata([testDataPath fileNamePrefix 'rdc2.bin']);
        if(all(abs(gold.noisemean_dB.RDC2 - noisemean_dB.RDC2)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.RDC2 - targData_lin.RDC2),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
        
    case 'sprdc2ch'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.spRDC2ch, noisemean_dB.spRDC2ch] = checkHALdata([testDataPath fileNamePrefix 'spsumchu.bin']);
        if(all(abs(gold.noisemean_dB.spRDC2ch - noisemean_dB.spRDC2ch)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.spRDC2ch - targData_lin.spRDC2ch),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
        
    case 'sprdc2'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.spRDC2, noisemean_dB.spRDC2] = checkHALdata([testDataPath fileNamePrefix 'sprdc2.bin']);
        if(all(abs(gold.noisemean_dB.spRDC2 - noisemean_dB.spRDC2)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.spRDC2 - targData_lin.spRDC2),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
    case 'sprdc3'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.spRDC3, noisemean_dB.spRDC3] = checkHALdata([testDataPath fileNamePrefix 'spsumu.bin']);
        if(all(abs(gold.noisemean_dB.spRDC3 - noisemean_dB.spRDC3)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.spRDC3 - targData_lin.spRDC3),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
    case 'stslice'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.stslice, noisemean_dB.stslice] = checkHALdata([testDataPath fileNamePrefix 'stslice.bin']);
        if(all(abs(gold.noisemean_dB.stslice - noisemean_dB.stslice)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.stslice - targData_lin.stslice),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
    case 'hist'
        gold = load([goldenDataPath 'GoldenData.mat'],'targData_lin','noisemean_dB');
        [targData_lin.hist, noisemean_dB.hist] = checkHALdata([testDataPath fileNamePrefix 'hist.bin']);
        if(all(abs(gold.noisemean_dB.hist - noisemean_dB.hist)<=noiseDeltaThresh_dB) ...
                & all(reshape(abs(gold.targData_lin.hist - targData_lin.hist),1,[])<=sigDeltaThresh_lin))
            Pass = true;
        else
            Pass = false;
        end
    otherwise
        Pass = False;
        error('Unknown case');
end

% use exit code 1 for passing result and zero for a fail
if isdeployed
    if Pass
        exit(0);
    else
        exit(1);
    end
end
