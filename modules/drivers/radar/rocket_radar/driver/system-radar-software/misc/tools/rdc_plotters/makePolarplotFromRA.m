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

function [ ] = makePolarplotFromRA(ppInputData,plotRbinCut,plsub,ringdat,...
    PMCW,detection_params,plotdBFS,dBFScorr_lin,swExponent,noNewFigs,plot2DBscope,polarplot,polarplotrng)

validrbinmask = detection_params.rangeGatesMid>=0;

allAz= unique(detection_params.angleGatesMid(:,1));
allEl= unique(detection_params.angleGatesMid(:,2));
azLen = length(allAz);
elLen = length(allEl);

temp1 = unique(detection_params.angleGatesMid(:,1));
temp1(:,2) = zeros(size(temp1));
detection_params.angleGatesMid=temp1;
toplot = ppInputData;
if plotdBFS
    foo = transpose(dBFScorr_lin.RDC3(validrbinmask));
    toplot=toplot./repmat(foo,size(toplot,1),1).*2^0;% dont trust CI exp (swExponent.CI);
end
toplot = mag2db(toplot);

% collapse the height for bscope
if PMCW.MIMO2D
    toplot = squeeze(max(reshape(toplot,azLen,elLen,[]),[],2));
end

% plsub(1) tot, plsub(2) min, plsub(3) max
subplot(1,plsub(1),plsub(2):plsub(3));


if(~noNewFigs), figure; end

if 0
    allranges=detection_params.rangeGatesMid(validrbinmask);
    minrbin = 1;
    maxrbin = 15;
    minrng = allranges(minrbin);
    maxrng = allranges(maxrbin);
    
    toplot2=toplot;%(minrbin:maxrbin,:);
else
    minrng= floor(min(detection_params.rangeGatesMid(validrbinmask)));
    maxrng= ceil(max(detection_params.rangeGatesMid(validrbinmask)));
    toplot2=toplot;%(minrbin:maxrbin,:);
end
plotrng = min(polarplotrng,maxrng);
toplot2=toplot;%(minrbin:maxrbin,:);
toplot2(toplot2<median(toplot(:))-30)=median(toplot(:))-30;
[t,r] = meshgrid(detection_params.angleGatesMid(:,1),detection_params.rangeGatesMid);
[x,y] = pol2cart(t,r);

% Axis property cell array
% Define some angular and radial range vectors for example plots
plotrng=min(polarplotrng, maxrng);
rnglims = [minrng plotrng];
% ylim2 = ceil(rnglims(2)*sind(max(abs(detection_params.angleGatesMid(:,1)))));
% legible tick spacing
if (plotrng < 10)
    rngticks = 1;
    angticksp = 1;
elseif (plotrng < 81)
    rngticks = 5;
    angticksp = 5;  
else
    angticksp = round(floor(plotrng/10),-1);
    rngticks = round(floor(plotrng/10),-1);    
end
ylim2 = floor(rnglims(2)*sind(max(abs(detection_params.angleGatesMid(:,1)))-3));

anglims = [min(detection_params.angleGatesMid(:,1)) max(detection_params.angleGatesMid(:,1)) ];
axprop = {'DataAspectRatio',[1 1 1],'View', [-90 90],...
    'Xlim', [0 maxrng],       'Ylim', [-ylim2 ylim2],...
    'XTick',[rnglims(1):rngticks:rnglims(2)],    'YTick',[fliplr(0:-angticksp:-ylim2) angticksp:angticksp:ylim2]};

%         figure('color','white');
%         polarplot3d(toplot2.','plottype','surf','angularrange',anglims,'radialrange',rnglims,...
%             'polargrid',{8 8},'tickspacing',4,'colordata',gradient(toplot2.'),...
%             'plotprops',{'Linestyle','interp'});
%         polarplot3d(toplot2.','plottype','mesh','angularrange',anglims,'radialrange',rnglims,...
%             'polargrid',{8 8},'tickspacing',8,...
%             'plotprops',{'Linestyle','none'});
%         figure('color','white');
if ~PMCW.MIMO2D && any(plotRbinCut>0)
    warning('use polarplotrng instead of plotRbinCut for 1D polar plotting')
else
    polarplot3d(toplot2.','plottype','surfn','radialrange',[min(detection_params.rangeGatesMid(validrbinmask)) max(detection_params.rangeGatesMid(validrbinmask))],...
        'angularrange',[deg2rad(detection_params.angleGatesMid(:,1))],'polargrid',{detection_params.rangeGatesMid(validrbinmask) deg2rad(detection_params.angleGatesMid(:,1))},'tickspacing',25);
    set(gca,axprop{:});
    % polarplot3d(toplot,'plottype','meshc','angularrange',anglims,'radialrange',rnglims,...
    %               'meshscale',2,'polargrid',{8 8});
    set(gca,axprop{:});
    colormap jet;
    xlabel 'X (m)';
    ylabel 'Y (m)';
    xlim([0 plotrng]);
    
    %ylim([-plotrng/2 plotrng/2]);
    if plotdBFS
        zlabel('Mag dBFS');
    else
        zlabel('Mag dB');
    end
    zlim([min(toplot2(:)) max(toplot2(:))+3]);
    title('Polar Representation')
    set(gca, 'YDir', 'reverse');
    shading interp;
    
    [vars.cax1 vars.cax2] = caxis;
    vars.plotrng = plotrng;
    method = 4;
    caxis(getCaxisFunc(method,toplot2,vars));
    hold on;
    aa = deg2rad(-24):0.01:deg2rad(22);
    xx = cos(aa);
    yy = sin(aa);
    zz = ones(length(aa)) .* max(toplot2(:));
    allranges=detection_params.rangeGatesMid(validrbinmask);
    % plot rbincut range limit lines if enabled
    if isfield(ringdat,'binA')
        plot3(xx * allranges(ringdat.binA), yy * allranges(ringdat.binA), zz, 'k-');
        plot3(xx * allranges(ringdat.binB), yy * allranges(ringdat.binB), zz, 'k-');
    end
    hold off;
end

end
