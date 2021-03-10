% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details

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
