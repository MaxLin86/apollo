% Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
% Refer to SOFTWARE_LICENSE file for details
function [ xx , yy , iqmat ] = iq_surface_data( all_i , all_q )
ii      = all_i( : )                    ;
mii     = min( ii )                     ;
mai     = max( ii )                     ;
qq      = all_q( : )                    ;
if all( ~qq )
    qq( 1 )     = -1                    ;
    qq(end )    =  1                    ;
end
miq     = min( qq )                     ;
maq     = max( qq )                     ;
ai      = round( double( ii - mii ) / 1 + 1 )                 ;
qi      = round( double( qq - miq ) / 1 + 1 )                 ;
iqmat   = accumarray( [ qi , ai ] , 1 ) ;
xx      = linspace( double( mii ) ,     ...
    double( mai ) ,     ...
    size( iqmat , 2 ) ) ;
yy      = linspace( double( miq ) ,     ...
    double( maq ) ,     ...
    size( iqmat , 1 ) ) ;
ii      = all_i( : )                    ;
% izi     = find( ii == 1 )               ;
% ii( izi( 1:2:end ) ) = -1               ;
mii     = min( ii )                     ;
mai     = max( ii )                     ;
qq      = all_q( : )                    ;
% qzi     = find( qq == 1 )               ;
% qq( qzi( 1:2:end ) ) = -1               ;
if all( ~qq )
    qq( 1 )     = -1                    ;
    qq(end )    =  1                    ;
end
miq     = min( qq )                     ;
maq     = max( qq )                     ;
ai      = round( double( ii - mii ) / 1 + 1 )                 ;
qi      = round( double( qq - miq ) / 1 + 1 )                 ;
iqmat   = accumarray( [ qi , ai ] , 1, [], @sum, NaN ) ;
xx      = linspace( double( mii ) ,     ...
    double( mai ) ,     ...
    size( iqmat , 2 ) ) ;
yy      = linspace( double( miq ) ,     ...
    double( maq ) ,     ...
    size( iqmat , 1 ) ) ;

mat_mu  = mean( iqmat( : ) )            ;
mat_std = std( iqmat( : ) )             ;

end
