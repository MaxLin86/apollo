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

function [ cax ] = getCaxisFunc(method,Caxdata,vars)

switch method
    case 1
        %clutterImageSD
        c = colormap(jet(400));
        
        if vars.rangeNorm && vars.useSVAcolormap
            c(1,:) = [0 0 .4];
            colormap([c(1,:);c(25:end,:)]);
            % caxis([c1 c2]);
            % used for DeNoising plots for CES and Ford
            cmin = median(Caxdata(:))+7 ;
            cmax = max(max(Caxdata(:))-0,median(Caxdata(:))+vars.CaxisRngDb);
            cmax = max(cmax,cmin+30);
        else
            cmin = median(Caxdata(:))-4 ;
            cmax = max(max(Caxdata(:)),median(Caxdata(:))+vars.CaxisRngDb);
        end
    case 2
        %clutterImageSD old code path        
        if (vars.rangeNorm)
            cmin = min(Caxdata(:));
        else
            cmin = min(min(Caxdata(:,3:end)));
        end
        cmax = max(Caxdata(:));
        if cmin>=cmax,
            cmax=cmin+.1;
        end % make sure min<max
    case 3
        % makePolarplotFromRA.m
        if vars.plotrng <= 80 % JJP hax skip agressive caxis on long range, peaks disappear
            cmin = min(vars.cax1+30,vars.cax2-11) 
            cmax = vars.cax2;                  % CAXIS for BSCOPE
        else
            cmin = vars.cax1;
            cmax = vars.cax2;
        end
    case 4
        % makePolarplotFromRA.m no SVA
        cmin = median(Caxdata(:))+1;
        cmax =  max(Caxdata(:))-3;
    otherwise
        cmin = min(Caxdata(:));
        cmax = max(Caxdata(:));
        
end

cax = [cmin cmax];