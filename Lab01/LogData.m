% A function to log results at each step of the MagnetLoc program.
% Results are logged to "log.txt", which can then be read by 
% "PlotResults.m" for result display.
% Author: G. Garcia

function LogData( t , phase , X , P , U , Y ) 

persistent fid ;

if strcmp(phase,'init') 
    
    fid = fopen('log.txt','w') ;
    fprintf( fid , 'calcPhase t x y theta P11 P12 P13 P22 P23 P33 U1 U2 y1 y2\n') ;
    fprintf( fid , '0 ' ) ;
    fprintf( fid , '%8f ' , ...
        [t, X(1),X(2),X(3) , P(1,1),P(1,2),P(1,3),P(2,2),P(2,3),P(3,3) , 0,0 , 0,0 ] ) ;
    fprintf( fid , '\n' ) ;
    
elseif strcmp(phase,'prediction') 
    
    fprintf( fid , '1 ' ) ;
    fprintf( fid , '%8f ' , ...
        [t, X(1),X(2),X(3) , P(1,1),P(1,2),P(1,3),P(2,2),P(2,3),P(3,3) , U(1),U(2) , 0,0 ] ) ;
    fprintf( fid , '\n' ) ;
    
elseif strcmp(phase,'measurement') 
    
    fprintf( fid , '2 ' ) ;
    fprintf( fid , '%8f ' , ...
        [t, X(1),X(2),X(3) , P(1,1),P(1,2),P(1,3),P(2,2),P(2,3),P(3,3) , 0,0 , Y(1),Y(2) ] ) ;
    fprintf( fid , '\n' ) ;
    
elseif strcmp(phase,'update') 
    
    fprintf( fid , '3 ' ) ;
    fprintf( fid , '%8f ' , ...
        [t, X(1),X(2),X(3) , P(1,1),P(1,2),P(1,3),P(2,2),P(2,3),P(3,3) , 0,0 , 0,0 ] ) ;
    fprintf( fid , '\n' ) ;
    
elseif strcmp(phase,'termination') 
    
    fclose(fid) ;
    
else
    error('Incorrect phase specification in "logData".');
end