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
function [det] = loadDetectionData(filestr,numdets)
a = dir(filestr);

if (size(a,1) > 0)
    if numdets>=0
        oldDetFormat = a.bytes/64 == numdets;
    else
        oldDetFormat=1;
    end
    if oldDetFormat
        numDets = a.bytes/64;
        assert(mod(a.bytes,64)==0,'unexpected file size');
    else
        numDets = a.bytes/44;
        assert(mod(a.bytes,44)==0,'unexpected file size');
    end
    f = fopen(filestr, 'r');

    det.range = [];
    det.az = [];
    det.el = [];
    det.dopp = [];
    det.mag = [];
    det.rcs = [];
    det.xyz = [];
    det.snr = [];
    det.flags = [];

    if f>0
        for itd = 1:numDets
            det.range(itd) = fread(f, 1, 'float');
            det.az(itd) = fread(f, 1, 'float'); %az
            det.el(itd) = fread(f, 1, 'float'); %el
            det.dopp(itd) = fread(f, 1, 'float');
            det.mag(itd) = fread(f, 1,'float');
            det.snr(itd) = fread(f, 1, 'float');
            det.rcs(itd) = fread(f, 1, 'float'); %TODO RCS calculation
            det.xyz(3*(itd-1)+1:3*(itd-1)+3) = fread(f, 3, 'float'); %TODO XYZ
            det.flags(itd) = fread(f, 1, 'uint32');
            if oldDetFormat
                fread(f, 5, 'uint32');
            end
        end
        det.staticFlag=logical(bitget(int32(det.flags),1));
        det.xyzmx = reshape(det.xyz,3,[]);

    else
        disp(['*** Cant find Dets data file ***']);
    end
    fclose(f);
else
    det = [];
end
end


