% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [  ] = ADCmultiplot( sig, PMCW, Fs, scrollon, analyzeSelectionOnly )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global result
if ~exist('Fs','var')
    disp('Assuming Fs=0.95625e9');
    Fs = 0.95625e9;
end

% ADC Data scroll mode
if ~exist('scrollon','var')
    scrollon=0;
end

if ~exist('analyzeSelectionOnly','var')
    analyzeSelectionOnly=0;
end

systemPRIsampsOSF = PMCW.Lc * PMCW.M * PMCW.G * PMCW.K * (PMCW.RxOSFnum / PMCW.RxOSFden);
sig = sig(1:(systemPRIsampsOSF * PMCW.N),:);
all_iq          = reshape(sig(:,result.plotADCphysRx), [systemPRIsampsOSF PMCW.N]).';

if analyzeSelectionOnly
    all_iq = all_iq(:,result.plotADCdispSampStart:result.plotADCdispSampStop);    
end

title_string    = sprintf( 'ADC Data, Lc = %d, N = %d' ,                ...
                            PMCW.Lc , PMCW.N )                          ;
% fft_points      = 2 ^ ( nextpow2( PMCW.Lc ) + 1 );
fft_points = PMCW.Lc;
all_i           = real(all_iq);
all_q           = imag(all_iq);
iq_mag          = abs(all_iq);

%%  Plot
end_point   = PMCW.N;

%close all
figure( 'Units' , 'Normalized' , 'Position' , [ 0.1 0.1 0.8 0.8 ] ,     ...
        'NumberTitle' , 'off' , 'Name' , title_string )
subplot( 2 , 2 , 3 )
log_image   = mag2db( iq_mag )      ;
log_image( isnan( log_image ) ) = 0             ;
imagesc( log_image )
caxis( [ min( log_image( : ) ) , max( log_image( : ) ) ] )
colormap jet
%colorbar
hold on
h_marker    = plot( [0 0] , [ result.plotADCdispSampStart result.plotADCdispSampStop ] , 'w', 'LineWidth',4 )              ;
h_t         = title( [ 'Magnitude' ] ,              ...
                       'Interpreter' , 'none' )   	;
xlabel( 'Chip Number (fast_time) -->' )
ylabel( '<-- Pulse Number (slow time)' )
drawnow

axi=subplot( 2 , 3 , 1:2 )                              ;
i_plot      = plot( (  all_i( 1 , : ) ) , '.-' ) 	;
hold on
q_plot      = plot( (  all_q( 1 , : ) ) , '.-' )        ;
h_ititle    = title( 'I' )                              ;
xlabel( sprintf( 'ADC Samples\n(zoom of sub-pulse window)' ) );
ylabel( 'Sample Value' )                                ;
grid on
legend( { 'I' , 'Q' } )
axis tight;
ylim( [ -max(abs(all_iq(:))) max(abs(all_iq(:))) ]  )
if ~analyzeSelectionOnly
    xlim( [ result.plotADCdispSampStart result.plotADCdispSampStop ] )
end
sp1(1) = subplot( 2, 3 , 3 )                                      ;
mm = max([abs(real(sig)); abs(imag(sig))]);     % for scaling the plot axes
ii              = all_i( 1 , : )            ;
qq              = all_q( 1 , : )            ;
[ xx , yy , iqmat ] = iq_surface_data( ii , qq )    ;
iqmat(isnan(iqmat)) = 0.001; % min plot -40dB
%     iqsurf(ii, qq, mm)

i_above         = mean( ii( ii > 0 ) )              ;
i_below         = mean( ii( ii < 0 ) )              ;
i_string        = sprintf( 'I: \\mu_{pos} = %0.2f, \\mu_{neg} = %0.2f' ,    ...
    i_above , i_below )                             ;
q_above         = mean( qq( qq > 0 ) )              ;
q_below         = mean( qq( qq < 0 ) )              ;
q_differential  = ( q_above + q_below ) / 2         ;
q_string        = sprintf( 'Q, \\mu_{pos} = %0.2f, \\mu_{neg} = %0.2f' ,    ...
    q_above , q_below )                             ;
h_ititle.String = [ i_string sprintf( '\n' ) q_string ]        ;

if min(size(iqmat))==1
    title('No Phase Histogram, insufficent data');
else
    c_plot  = surf( xx , yy , mag2db( iqmat ), 'EdgeColor' , 'none' )    ;

    view( 2 )
    xlabel( 'I' )
    ylabel( 'Q' )
    title('Phase Histogram');
    title( 'I and Q Phase Orientation' )
    axis equal  % square tight
    xlim( [ min( xx ) , max( xx ) ] )
    ylim( [ min( yy ) , max( yy ) ] )
    %zlim( [ -40 200 ])
end

fftdat = fftshift(fft( double( all_iq( 1 , : ) ) , fft_points ));
[maxmag, maxi ]=max(abs(fftdat));
maxphase=angle(fftdat(maxi));

subplot( 2 , 2 , 4 )                                      ;
m_plot          = plot( mag2db(abs(                           ...
    fftdat ) ) -mag2db(length(all_iq( 1 , : ))*2^7))                      ;
m_plot.XData    = linspace( -Fs/2 , Fs/2 , fft_points )           ;

xlim( [-Fs/2 Fs/2] )
ylim( [median(m_plot.YData) 0] )
h_ftitle = title( ['FFT of ADC for System pulse ' num2str(1) ] );
grid on
xlabel( 'Frequency, Hz' )
ylabel( 'Mag, dBFS' )

if scrollon
    h_marker.YData = [0 2];
    moveBar=ceil(diff([ result.plotADCdispSampStart result.plotADCdispSampStop ])/4);
    sizeBar=4*moveBar;
    fft_points      = 2 ^ ( nextpow2( sizeBar ) + 1 );
    m_plot.XData    = linspace( -Fs/2 , Fs/2 , fft_points )           ;

    for itsc=1:moveBar:length(all_i)-sizeBar
        
        ii              = all_i( 1 , itsc:itsc+sizeBar )            ;
        i_above         = mean( ii( ii > 0 ) )              ;
        i_below         = mean( ii( ii < 0 ) )              ;
        i_string        = sprintf( 'I: \\mu_{pos} = %0.2f, \\mu_{neg} = %0.2f' ,    ...
            i_above , i_below )                             ;
        qq              = all_q( 1 , itsc:itsc+sizeBar )            ;
        q_above         = mean( qq( qq > 0 ) )              ;
        q_below         = mean( qq( qq < 0 ) )              ;
        q_differential  = ( q_above + q_below ) / 2         ;
        q_string        = sprintf( 'Q, \\mu_{pos} = %0.2f, \\mu_{neg} = %0.2f' ,    ...
            q_above , q_below )                             ;
        h_ititle.String = [ i_string sprintf( '\n' ) q_string ]        ;
        
        raw_fft_data    = double( all_iq( 1, itsc:itsc+sizeBar ) );
        m_plot.YData    = mag2db(abs( fftshift(                    ...
        fft( raw_fft_data , fft_points )))) -mag2db(length(raw_fft_data)*2^7);
    
        xlim(axi,[itsc itsc+sizeBar]);
        h_marker.XData = [itsc+moveBar itsc+moveBar];

        [ xx , yy , iqmat ] = iq_surface_data( ii , qq );
        iqmat(isnan(iqmat)) = 0.001;
        
        c_plot.XData    = xx                                ;
        c_plot.YData    = yy                                ;
        %Sc_plot.CData    = iqmat                             ;
        c_plot.ZData    =  mag2db( iqmat )                             ;
        
        drawnow
        pause(.1);
        
    end
end
for i_samples = 2 : end_point
    
    % for i_samples = kron( [ 251 : 256 ] , ones( 1 , 5 ) )
    ii              = all_i( i_samples , : )            ;
    i_above         = mean( ii( ii > 0 ) )              ;
    i_below         = mean( ii( ii < 0 ) )              ;
    i_string        = sprintf( 'I: \\mu_{pos} = %0.2f, \\mu_{neg} = %0.2f' ,    ...
                                i_above , i_below )                             ;
    qq              = all_q( i_samples , : )            ;
    q_above         = mean( qq( qq > 0 ) )              ;
    q_below         = mean( qq( qq < 0 ) )              ;
    q_differential  = ( q_above + q_below ) / 2         ;
    q_string        = sprintf( 'Q, \\mu_{pos} = %0.2f, \\mu_{neg} = %0.2f' ,    ...
                                q_above , q_below )                             ;
    h_ititle.String = [ i_string sprintf( '\n' ) q_string ]        ;

    raw_fft_data    = double( all_iq( i_samples , : ) );
    fftdat=fftshift(fft( raw_fft_data , fft_points ));
    [maxmag(i_samples), maxi(i_samples)]=max(abs(fftdat));
    maxphase(i_samples)=angle(fftdat(maxi(i_samples)));

    q_plot.YData    = qq                                ;
    i_plot.YData    = ii                                ;
    m_plot.YData    = mag2db(abs(                     ...
    fftdat)) -mag2db(length(raw_fft_data)*2^7);
    h_marker.YData = i_samples * [ 1 1 ]                ;
    h_ftitle.String = ['FFT of ADC for System pulse ' num2str(i_samples) ];

    h_marker.XData = [ result.plotADCdispSampStart result.plotADCdispSampStop ];

    [ xx , yy , iqmat ] = iq_surface_data( ii , qq );
    iqmat(isnan(iqmat)) = 0.001;

    c_plot.XData    = xx                                ;
    c_plot.YData    = yy                                ;
    %Sc_plot.CData    = iqmat                             ;
    c_plot.ZData    =  mag2db( iqmat )                             ;
    
    drawnow
    pause(.01);
end
%% plots for DC phase over any data in analysis section
% figure, hist(mag2db(maxmag(maxi>=1024&maxi<=1026)),100); xlabel mag(dB); title histogram(DCmag);
% figure, hist(rad2deg(maxphase(maxi>=1024&maxi<=1026)),100); xlabel phase(deg); title histogram(DCphase)
% figure, hist(maxi,100);title histogram(maxFFTbin)
end
  

