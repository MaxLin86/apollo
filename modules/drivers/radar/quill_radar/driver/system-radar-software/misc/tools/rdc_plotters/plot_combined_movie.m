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

function plot_combined_movie(socpath,scannums)
%scannums=1:659;
%socpath='~/Documents/Uhnder/Bitbucket/sabine-radar-sw/build-x86';
    if ~exist('loadjson','file')
        addpath('../jsonlab/');
    end
    for itscan=scannums
        fig=figure;
        set(fig,'Position',[5 40 900 850]);
        colormap('jet');

        % Plot clutter image
        %plot_clutterImage([socpath '/socsim_' num2str(itscan,'%06d') '_clutterimage.bin'])
        plot_clutterImageSD([socpath '/scan_' num2str(itscan,'%06d') '_clutterimage.bin']);
        xlabel('X [pixels]');
        ylabel('Y [pixels]');
        title(['Scan: ' num2str(itscan) ', Clutter Image']);
        axis equal;
        xlim([0 384]);
        ylim([0 384]);
        %ca = caxis;

        hold on;

        % Plot detections
        %soc = loadDetectionData([socpath '/socsim_' num2str(itscan,'%06d') '_detectreport.bin']);
        sam_detectreport = [socpath '/scan_' num2str(itscan,'%06d') '_detectreport.bin'];
        [fp1, fp2, fp3] = fileparts(sam_detectreport);
        samjsonfilepath = [fp1 '/' fp2(1:strfind(fp2,'_detectreport')) 'info.json'];
        scanData = loadjson(samjsonfilepath);
        sam_num_detections=scanData.num_detections;
        soc = loadDetectionData(sam_detectreport,sam_num_detections);
        soc.mag = soc.mag - min(soc.mag) + 1;
        soc.mag = soc.mag ./ max(soc.mag);
        soc.mag = ((2+(soc.mag .* 3)) .^ 4);

        cm = colormap('jet');
        cx = soc.dopp;
        cx = min(cx, 26);
        cx = max(cx, -26);
        cx = cx + 27;
        cx = cx ./ 53;
      % cx = cx .* (ca(2) - ca(1));
      % cx = cx + ca(1);
        cx = cx .* size(cm, 1);
        c = cm(floor(cx),:);

        scale = 384 / (80 * 0.59958488);
        xx = 384 - (scale .* soc.range .* cos(soc.az));
        yy = (384/2) + (scale .* soc.range .* sin(soc.az));
        colormap('jet');    
        scatter3(xx, yy, 200 .* ones(1,length(xx)), soc.mag, c, 'square', 'LineWidth', 2);

        % Save image
        frame  = getframe(fig);
        im = frame2im(frame);
        drawnow
        [imind,cm] = rgb2ind(im,256);

        if itscan == scannums(1)
            imwrite(imind,cm,[socpath '/clutterImage_mov.gif'],'gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,[socpath '/clutterImage_mov.gif'],'gif','WriteMode','append');
        end
        drawnow
        pause(.25)
        close all
    end
end

