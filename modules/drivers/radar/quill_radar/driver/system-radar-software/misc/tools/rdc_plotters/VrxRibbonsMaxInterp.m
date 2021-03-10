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
% (C) 2017 Uhnder Inc
% Author:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: plotSimpleRDC1()
%
% Option:
%
% Input:     None
%
% Output:
%
% Calls:
%
%
% Description: Plots coherent RDC1 sum over all Vrx.  Also will perform
%              Correlation and/or Decorrelation on ADC/RDC data from bin file
% WORKS FOR BPSK ONLY !!!!!!!!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [interpPerVrx] = ...
    plotSimpleRDC1(databinfile)
global antenna
global sp_params
global cref
global PMCW
global digital_BE
global scenario
global detection_params

interpPerVrx=[];

sp_params.decorrelationIters = 0;

HAX_numPulses = 360; % for when the scan data doesnt have the correct Npulses
rangeAdjustm = 0;
targRbin = 76;
targDbin = 505;
showAllVRx=1;
spatialOrdering = 0; % set to zero for HW Tx/Rx Major (not Spatial) VRx
plotRDC1vrx = 1;
plotRbinMinMax = [1 256];
pulsedMode=1;
rbinStartInterp = 1; 
rbinStopInterp = 20;

if ~exist('activeHWTX','var')
    activeHWTX=[ 3 4 5 6 7 8 9 11 ];
    activeHWRX=[ 0:7 ];
end


if strfind(pwd,'support_scripts')
   cd ..
   addPaths
end

addpath('../jsonlab');

if ~exist('databinfile','var')
    databinfile = 'X:\software\jpb\data_2017_09_12\s2_rdc1_adi_00000000\scan_000000_rdc1.bin';
end
if ~exist('plotrxMajVrx','var')
    plotrxMajVrx = 1:64;
end
plotdBFS = 0;

disp('Loading params from scaninfo json... ');
[fp1 fp2 fp3] = fileparts(databinfile);
RDC1expFile = [fp1 '/' fp2 'exp' fp3];
%     scanData=loadjson([fp1 '/' fp2(1: max([findstr(fp2,'_rdc1'),findstr(fp2,'_adc')])) 'info.json']);
%Batchmode: scanData=loadjson('C:\uhnder\TESTDATA\Jul28\data\ADB101\scan_000162_adc_info.json');

% Load Scan Info Json
SDjsonfilepath = [fp1 '/' fp2(1:findstr(fp2,'_rdc1')) 'info.json'];
[PMCW, antenna, detection_params, dBFScorr_lin, swExponent, ~] = processSabineScanInfo(SDjsonfilepath);

if ~exist('HAX_numPulses','var')
    PMCW.N=HAX_numPulses
end

nrx = 8;
ntx = antenna.NVrx/nrx;

% hardcoded baddies 1-D antenna
antenna.realhwTxMap=[ 3 4 5 6 7 8 9 11 ]; % Rx/Tx major ordering
%  antenna.hwTxMap8 = [1 3 5 7 8 2 4 6];
%  antenna.realhwTxMap = antenna.realhwTxMap(antenna.hwTxMap8);
% antenna.realhwTxMap=[ 3 5 7 9 11 4 6 8 ];
antenna.realhwRxMap=[ 0 1 2 3 4 5 6 7 ]; % Rx/Tx major ordering
antenna.map = [1,17,2,33,18,3,49,34,19,4,57,50,35,20,9,58,51,25,36,10,59,41,52,26,11,60,42,27,12,5,43,28,21,6,44,37,22,7,53,38,23,8,61,54,39,24,13,62,55,29,40,14,63,45,56,30,15,64,46,31,16,47,32,48];
antenna.mapToVrx = [1,3,6,10,30,34,38,42,15,20,25,29,47,52,57,61,2,5,9,14,33,37,41,46,18,24,28,32,50,56,60,63,4,8,13,19,36,40,45,51,22,27,31,35,54,59,62,64,7,12,17,23,39,44,49,55,11,16,21,26,43,48,53,58];
antenna.filledIndices = [4,5,6,7,8,9,11,12,13,15,16,17,19,20,21,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,43,44,45,47,48,49,51,52,53,55,56,57,58,59,60,61];
antenna.txposoffs = [0,0,0,0,0,0,0,0;-0.0204543465784095,0.00894565342164539,-0.0145743465783985,0.0148256534216564,-0.00869434657838757,0.0207056534216674,-0.00281434657837658,0.00306565342163440;0.0399020000000000,0.0399020000000000,0.0399020000000000,0.0399020000000000,0.0399020000000000,0.0399020000000000,0.0399020000000000,0.0399020000000000];
antenna.rxposoffs = [0,0,0,0,0,0,0,0;-0.0393143465784315,-0.0314743465784169,-0.0236343465784022,-0.0157943465783876,0.0155756534216124,0.0234156534216271,0.0312556534216417,0.0390956534216564;-0.0399020000000000,-0.0399020000000000,-0.0399020000000000,-0.0399020000000000,-0.0399020000000000,-0.0399020000000000,-0.0399020000000000,-0.0399020000000000];
antenna.scan_alpha_deg = [-57.8509475739755	-56.4426902380793	-55.0847937525558	-53.7715225618896	-52.4981170132557	-51.2605754021444	-50.0554948101851	-48.8799524467641	-47.7314155704275	-46.6076720042680	-45.5067757692332	-44.4270040008057	-43.3668224132226	-42.3248573261885	-41.2998727917059	-40.2907517287020	-39.2964802391875	-38.3161344736657	-37.3488695567813	-36.3939101912697	-35.4505426391754	-34.5181078410613	-33.5959954815160	-32.6836388462580	-31.7805103451237	-30.8861175981283	-30.0000000000000	-29.1217256931816	-28.2508888910513	-27.3871075026539	-26.5300210180112	-25.6792886194569	-24.8345874897016	-23.9956112916832	-23.1620687988820	-22.3336826578053	-21.5101882668875	-20.6913327581876	-19.8768740700788	-19.0665801006559	-18.2602279328959	-17.4576031237221	-16.6584990500775	-15.8627163059362	-15.0700621448888	-14.2803499635482	-13.4933988215517	-12.7090329943954	-11.9270815557326	-11.1473779861174	-10.3697598054774	-9.59406822686047	-8.82014782923351	-8.04784624731152	-7.27701387656959	-6.50750359174475	-5.73917047726679	-4.97187156817286	-4.20546560015999	-3.43981276751520	-2.67477448773541	-1.91021317170994	-1.14599199838859	-0.381974692898352	0.381974692898352	1.14599199838859	1.91021317170994	2.67477448773541	3.43981276751520	4.20546560015999	4.97187156817286	5.73917047726679	6.50750359174475	7.27701387656959	8.04784624731152	8.82014782923351	9.59406822686047	10.3697598054774	11.1473779861174	11.9270815557326	12.7090329943954	13.4933988215517	14.2803499635482	15.0700621448888	15.8627163059362	16.6584990500775	17.4576031237221	18.2602279328959	19.0665801006559	19.8768740700788	20.6913327581876	21.5101882668875	22.3336826578053	23.1620687988820	23.9956112916832	24.8345874897016	25.6792886194569	26.5300210180112	27.3871075026539	28.2508888910513	29.1217256931816	30.0000000000000	30.8861175981283	31.7805103451237	32.6836388462580	33.5959954815160	34.5181078410613	35.4505426391754	36.3939101912697	37.3488695567813	38.3161344736657	39.2964802391875	40.2907517287020	41.2998727917059	42.3248573261885	43.3668224132226	44.4270040008057	45.5067757692332	46.6076720042680	47.7314155704275	48.8799524467641	50.0554948101851	51.2605754021444	52.4981170132557	53.7715225618896	55.0847937525558	56.4426902380793	57.8509475739755];antenna.fullyFilled = length(antenna.filledIndices);

%CONVERT TO DEGREES UP FRONT
detection_params.angleGatesMid = rad2deg(detection_params.angleGatesMid);

if plotdBFS==2
    dBFScorr_lin.RDC1 = dBFScorr_lin.RDC1max;
    dBFScorr_lin.RDC2 = dBFScorr_lin.RDC2max;
    dBFScorr_lin.RDC3 = dBFScorr_lin.RDC3max;
end

samples = PMCW.Lc * PMCW.N * PMCW.RxOSFnum/PMCW.RxOSFden; % Lc  * OSF *N

% if(~(all(scanData.vrx_map+1 == antenna.mapToVrx)))
%     warning('Vrx map mismatch warning!!!')
% end

% if sp_params.pulsedMode
%    disp('HAX: PMCW.R=512');disp('HAX: PMCW.R=512');disp('HAX: PMCW.R=512');disp('HAX: PMCW.R=512')
%     PMCW.R=512;
% end


%% Begin Processing

% RDC1 data binary is in spatial order
RDC1data=plot_RDC1dataSD([fp1 '/' fp2 '.bin']);
ylim(plotRbinMinMax);
% Range Bin reordering for pulsed mode
% Reorder Range bins now easier based on scan data

% rng correction
detection_params.rangeGatesMid(detection_params.rangeGatesMid>=0) = max(detection_params.rangeGatesMid(detection_params.rangeGatesMid>=0) - rangeAdjustm,0);

% Plot RDC1
if spatialOrdering
    
    HWtxmap = repelem(antenna.realhwTxMap, 8);
    HWrxmap = repmat(antenna.realhwRxMap,[1 8]);
    if showAllVRx
        plotmask=ismember(HWtxmap,antenna.realhwTxMap) & ismember(HWrxmap,antenna.realhwRxMap);
    else
        plotmask=zeros(1,64);
        plotmask(plotrxMajVrx)=1;
    end
    plotRDC1=RDC1data;
    
    %masking off hidden antennas
    plotRDC1(:,~plotmask(antenna.map),:)=0;
    
    % plot Tx colored
    lin=lines;
    temp=[lin(1:7,:); .5 .5 .5];
    cmap=[repelem(temp(:,1),8), repelem(temp(:,2),8), repelem(temp(:,3),8)];
%     if numel(activeHWRX)==1
%         figure(activeHWRX+1);
%     else
%         figure(1);
%     end
    figure;
    vrxCohSum = mag2db(abs(squeeze(sum(plotRDC1,3))));
    if plotdBFS
        hplot = ribbon(vrxCohSum-repmat(mag2db(dBFScorr_lin.RDC2),1,size(plotRDC1,2)));
    else
        hplot = ribbon(vrxCohSum);
    end
    ylim(plotRbinMinMax);
    
    
    if sp_params.decorrelationIters
        if length(activeHWTX)==0
            set( hplot, 'EdgeColor', 'k' );
        elseif length(activeHWTX) == 1
            set( hplot, 'EdgeColor', 'm' );
        else
            set( hplot, 'EdgeColor', 'c' );
        end
    else
        if length(activeHWTX)==0
            set( hplot, 'EdgeColor', 'k' );
        elseif length(activeHWTX) == 1
            set( hplot, 'EdgeColor', 'r' );
        else
            set( hplot, 'EdgeColor', 'k' );
        end
    end
    

%     if sp_params.decorrelationIters
%         set( hplot, 'EdgeColor', 'k' );
%     else
%         set( hplot, 'EdgeColor', 'r' );
%     end
%         set( hplot, 'LineStyle', 'none' );
    colormap(cmap((antenna.map),:));
    xlabel 'Spatial VRx Ordering', ylabel Rangebin, zlabel Magnitude(dB);
    title 'RDC1 Coherent Sum, Spatial Order';
    title 'Simulated-RDC1 Coherent Sum, Spatial Order';
else
    %masking off hidden antennas
    %RDC1data(:, antenna.map, :) = RDC1data; %remap to rxmajor
    plotRDC1=RDC1data;
    HWtxmap = repelem(antenna.realhwTxMap, 8);
    HWrxmap = repmat(antenna.realhwRxMap,[1 8]);
    if showAllVRx 
        plotmask=ismember(HWtxmap,antenna.realhwTxMap) & ismember(HWrxmap,antenna.realhwRxMap);
    else
        plotmask=zeros(1,64);
        plotmask(plotrxMajVrx)=1;
    end
    plotRDC1(:,~plotmask,:)=0;
    
    % plot Tx colored
    lin=lines;
    temp=[lin(1:ntx-1,:); .5 .5 .5];
    cmap=[repelem(temp(:,1),nrx), repelem(temp(:,2),nrx), repelem(temp(:,3),nrx)];
%     if numel(activeHWRX)==1
%         figure(activeHWRX+1);
%     else
%         figure(1);
%     end
    figure;
    vrxCohSum = mag2db(abs(squeeze(sum(plotRDC1,3))));
    if plotdBFS
        hplot = ribbon(vrxCohSum-repmat(mag2db(dBFScorr_lin.RDC2),1,size(plotRDC1,2)));
    else
        hplot = ribbon(vrxCohSum);
    end
    ylim(plotRbinMinMax);
    
    if sp_params.decorrelationIters
        if length(activeHWTX)==0
            set( hplot, 'EdgeColor', 'k' );
        elseif length(activeHWTX) == 1
            set( hplot, 'EdgeColor', 'm' );
        else
            set( hplot, 'EdgeColor', 'c' );
        end
    else
        if length(activeHWTX)==0
            set( hplot, 'EdgeColor', 'k' );
        elseif length(activeHWTX) == 1
            set( hplot, 'EdgeColor', 'r' );
        else
            set( hplot, 'EdgeColor', 'k' );
        end
    end
    %     if sp_params.decorrelationIters
    %         set( hplot, 'EdgeColor', 'k' );
    %     else
    %         set( hplot, 'EdgeColor', 'r' );
    %     end
    %         set( hplot, 'LineStyle', 'none' );
    grid minor;
    colormap(cmap);    
    xlabel 'HW Tx/Rx Major (not Spatial) VRx', ylabel Rangebin, zlabel Magnitude(dB);
    title 'Simulated-RDC1 Coherent Sum, RxMajor Order, Rx Tx';
    figure; 
    
    censData=vrxCohSum(rbinStartInterp:rbinStopInterp,:);
    padCensData = zeros(size(censData,1)+2,size(censData,2));
    padCensData(2:end-1,:)=censData;
    [ ~, maxInd] = max(padCensData);
    for itvrx=1:length(maxInd)
        [ interpMagRng(itvrx), interpBinRng(itvrx) ] = getInterpPeak( [ padCensData(maxInd(itvrx)-1,itvrx) padCensData(maxInd(itvrx),itvrx) padCensData(maxInd(itvrx)+1,itvrx) ], [rbinStartInterp+maxInd(itvrx)-3:rbinStartInterp+maxInd(itvrx)-1] );
    end
    if antenna.NVrx == 96
        imagesc(reshape(interpBinRng,8,12));
    else
        if antenna.NVrx == 64
            imagesc(reshape(interpBinRng,8,8));
        end
    end
    ylabel Rx, xlabel Tx
    title('Interpolated Rangebin of RDC1 Coherent Sum Peak')
    
end
% savefig([fp1 '/' fp2 'ribbons.fig']);

