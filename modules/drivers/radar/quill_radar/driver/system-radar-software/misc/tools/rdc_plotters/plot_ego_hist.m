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

% parameters:
% hist_fname - histogram file name
% data_fname - ego velocity data file name(list of points)
% example - plot_ego_hist('socsim_000000_ego_vel_histogram.bin', 'socsim_000000_ego_vel_data.bin')
function plot_ego_hist(hist_fname, data_fname)

% parse files
f = fopen(hist_fname, 'r');
hist_size = fread(f, 1, 'uint16');
hist = fread(f, hist_size, 'uint16');
fclose(f);

f = fopen(data_fname, 'r');
num_data = fread(f, 1, 'uint16');

ego_vel_angle = zeros(1, num_data);
ego_vel_doppler = zeros(1, num_data);
ego_vel_magnitude = zeros(1, num_data);
for iter=1:num_data
    ego_vel_angle(iter)=fread(f, 1, 'float');
    ego_vel_doppler(iter)=fread(f, 1, 'float');
    ego_vel_magnitude(iter)=fread(f, 1, 'float'); 
end
fclose(f);

% plot
subplot(2,2,1);
plot(hist);
title('histogram');

subplot(2,2,2);
plot(ego_vel_angle); % radian
title('angle');

subplot(2,2,3);
plot(ego_vel_doppler); % m/s
title('doppler');

subplot(2,2,4);
plot(ego_vel_magnitude);
title('magnitude');
