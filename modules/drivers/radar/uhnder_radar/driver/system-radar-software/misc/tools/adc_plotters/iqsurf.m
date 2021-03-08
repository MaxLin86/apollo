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

% Plot a probability distribution of IQ samples

function iqsurf(ii, qq, mm)
    %   ii        vector of I samples
    %   qq        vector of Q samples
    %   mm        largest magnitude of I or Q (used for plotting axis)

    cluster_factor = 1;     % increasing this number makes coarser histograms
                            % increase when there are insufficient number of samples

    view( 2 );
    grid on;
    xlabel( 'I   (ADC samples)' );
    ylabel( 'Q   (ADC samples)' );
    axis auto equal;
    xlim( [ -mm , mm ] );
    ylim( [ -mm , mm ] );

    if length(ii) <= 1 || length(qq) <= 1
        return
    end

    mii     = min( ii );
    mai     = max( ii );
    miq     = min( qq );
    maq     = max( qq );

    ai      = round( double( ii - mii ) / cluster_factor + 1 );
    aq      = round( double( qq - miq ) / cluster_factor + 1 );

    iqmat   = accumarray( [ aq , ai ] , 1 );

    xx      = linspace( double( mii ) ,             ...
                        double( mai ) ,             ...
                        size( iqmat , 2 ) );
    yy      = linspace( double( miq ) ,             ...
                        double( maq ) ,             ...
                        size( iqmat , 1 ) );


    % iqmat = mag2db(iqmat);            % don't use dB scale (or do?)
    iqmat(iqmat==0) = nan();            % turn zeros to NaNs to make background transparent

    surf( xx , yy , iqmat, 'EdgeColor' , 'none' );

    mx = max(iqmat(:));
    zlim( [ 0, mx+1 ] );
    caxis( [ 0 , 0.8 * mx ] );

    clear ai aq iqmat xx yy;
