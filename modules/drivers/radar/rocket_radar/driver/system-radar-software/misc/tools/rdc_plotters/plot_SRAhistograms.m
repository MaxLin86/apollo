% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
% START_SOFTWARE_LICENSE_NOTICE
% -------------------------------------------------------------------------------------------------------------------
% Copyright (C) 2019 Uhnder, Inc. All rights reserved.
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
%function [] = load_SRAhistograms(histin,plotRbins)
function [histograms_data] = plot_SRAhistograms(histin,plotRbins)

histin = load_SRAhistograms(histin);

%string to number conversion needed for executable input parameters 
if exist('plotRbins','var') 
    if ischar(plotRbins)
        plotRbins=str2num(plotRbins);   
    end
end

shiftExponents = 1; % show histograms shifted by their exponents
maxdB=200;
histin.histData = [histin.histData, zeros(size(histin.histData,1),maxdB-size(histin.histData,2))];
if ~exist('plotRbins','var')
    plotRbins = 1:size(histin.histData,1);
end

if shiftExponents
    for kk = 1:size(histin.histData,1)
        if(histin.Exponent(kk) < 0)
            abs_exp = abs(round(mag2db(2^histin.Exponent(kk))));
            histin.histData(kk,:) = [zeros(1,abs_exp) histin.histData(kk,abs_exp+1:end)];
        else
            histin.histData(kk,:) = [zeros(1,round(mag2db(2^histin.Exponent(kk)))) histin.histData(kk,1:end-round(mag2db(2^histin.Exponent(kk))))];
        end
        
    end
end

clines = colormap('lines');
figure,
hax=axes;
plot(1:maxdB,histin.histData(plotRbins,:),'.-')
legstr=[];
for itl=1:length(plotRbins)
    legstr=[legstr; ['Rbin ' num2str(plotRbins(itl),'%03d') ' Exponent ' num2str(histin.Exponent(plotRbins(itl)),'%02d')]];
end
legend(legstr);

histograms_data = histin.histData;

xlim([0,maxdB+1]),xlabel('dB'),ylabel('Count')
if shiftExponents
    title('SRA Histogram Data per Rangebin, Exponent Corrected');
else
    title('SRA Histogram Data per Rangebin');
end
