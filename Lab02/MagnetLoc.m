% Localization using a grid of magnets in the ground.
% -----
% Usage: 
%    - Set the characteristics of the robot and sensor in 
%      RobotAndSensorDefinition.m
%    - Set noise levels in DefineVariances.m
%    - Set robot initial position in the present file.
%    - Execute this file.
%    - Then, execute PlotResults.m to get the plots for analysis.
% You can also use, for a brand new fresh start each time:
% clear all; close all; MagnetLoc; PlotResults;
% -----
% Project history:
%    - Project initiator and principle of the sensor: Gaëtan Garcia
%    - Sensor manufacturing and test: Joël Rousseau.
%    - Characterization of the sensor: EMARO1/ARIA1 project, Hendry Chame
% and Marcelo Gaudenzi de Faria. Supervision: Gaëtan Garcia
%    - First implementation (Matlab): EMARO1/ARIA1 project, Filip Cichosz
% and Marteen Samuel. Supervision: Gaëtan Garcia.
%    - This program (using Samuel and Cichosz's work): Gaëtan Garcia

RobotAndSensorDefinition ;
DefineVariances ;

% Set initial posture and wheel radii. Use +error to introduce errors
% with respect to nominal radius rwheel.
X = [ 0, 0, 0*pi/180, rwheel, rwheel ].' ;  
%X = [ 0, 0, 0*pi/180, 20.9, 20.8 ].' ;
P = Pinit ; 

%Load the data file
dataFile = uigetfile('data/*.txt','Select data file') ;
if isunix 
    eval(['load data/' , dataFile]) ;
else
    eval(['load data\' , dataFile]) ;
end
dataFile = strrep(dataFile, '.txt', '') ;
eval(['data = ',dataFile,'; clear ',dataFile]) ;

disp('-----');
disp('MagnetLoc running with:');
fprintf('File: %s\n',dataFile);
fprintf('Initial posture: x: %d mm  y: %d mm  theta: %d deg.\n', X(1),X(2),X(3)*180/pi);
fprintf('Initial radii: right: %2.1f mm  left: %2.1f mm  \n', X(4),X(5));
fprintf('Tuning parameter: %d\n',sigmaTuning);
fprintf('Radius noise standard deviation in Pinit: %d mm\n', sqrt(Pinit(4,4)));

% Log data for display of results. Do not modify.

LogData( 0 , 'init' , X , P , [0;0] , [0;0] ) ;

% Skip motionless parts of the data at beginning and end of the experiment
% and return only meaningful data, with wheel rotations in radians.
% Also reduce encoder resolution and frequency according to factors
% set in RobotDefinition.m

[nbLoops,t,qL,qR,sensorReadings] = PreprocessData(data, dots2rad, dumbFactor, subSamplingFactor ) ;

wbHandle = waitbar(0,'Computing...') ;

for i = 2 : nbLoops 
    
    t = (i-1)*samplingPeriod ;
    
    waitbar(i/nbLoops) ;

    % Calculate input vector from proprioceptive sensors
    % Note: the definition of the input vector is different from that used
    % by the standard version of MagnetLoc (without radius identification).
    U = [ qR(i) - qR(i-1) ; 
          qL(i) - qL(i-1) ] ;
      
    % Elementary translation and rotation:
    deltaD     = (1/2)*( X(4)*U(1) + X(5)*U(2) )             ;
    deltaTheta = X(4)/trackGauge*U(1) - X(5)/trackGauge*U(2) ;
    
    % Calculate linear approximation of the system equation
    A = [ ***   ] ;
    B = [ ***      ] ;
       
    % Predic state (here odometry)
    X = EvolutionModel( X , U ) ;
    
    % Error propagation
    P = A*P*(A.') + B*Qbeta*(B.') + Qalpha ;
    
    LogData( t , 'prediction' , X , P , U , [0;0] ) ;
    
    % Vector of measurements. Size is zero if no magnet was detected.

    measures = ExtractMeasurements( sensorReadings(i), ...
        nbReedSensors, magnetDetected ) ;
            
    % When two or more magnets are detected simultaneously, they are taken
    % as independant measurements, for the sake of simplicity.

    for measNumber = 1 : length(measures) 
        
        % Homogeneous transform of robot frame with respect to world frame
        oTm = [ cos(X(3)) , -sin(X(3)) , X(1)  ; 
                sin(X(3)) ,  cos(X(3)) , X(2)  ; 
                    0     ,      0     ,  1    ] ;
        mTo = inv(oTm) ;
        
        % Measurement vector: coordinates of the magnet in Rm.
        Y = [ sensorPosAlongXm ; 
              sensorRes*( measures(measNumber) - sensorOffset ) ] ;
         
        % Now in homogeneous coordinates for calculations.
        mMeasMagnet = [ Y ;                
                        1 ] ;
                
        % Corresponding position in absolute frame. 
        oMeasMagnet = oTm * mMeasMagnet ;
        
        % Due to measurement and localization errors, the previously calculated
        % position does not match an actual magnet.
        % Which actual magnet is closest? It will be the candidate magnet for
        % the update phase of the state estimator.
        oRealMagnet = round( oMeasMagnet ./ [xSpacing ; ySpacing ; 1] ) .* [xSpacing ; ySpacing ; 1] ;

        % The position of the real magnet in robot frame. It will in general 
        % be different from the measured one. 
        mRealMagnet = oTm \ oRealMagnet ;  % That's inv(oTm)*oRealMagnet = mTo*oRealMagnet
        
        % The expected measurement are the two coordinates of the real 
        % magnet in the robot frame.
        Yhat = mRealMagnet(1:2) ;
        
        C = [ *** ] ;
                      
        innov = Y - Yhat ;   
        dMaha = sqrt( innov.' * inv( C*P*C.' + Qgamma) * innov ) ;
        
        LogData( t , 'measurement' , X , P , [0;0] , Y ) ;
        
        if dMaha <= mahaThreshold
            K = P * C.' * inv( C*P*C.' + Qgamma) ;
            X = X + K*innov ;
            P = (eye(length(X)) - K*C) * P ;
            LogData( t , 'update' , X , P , [0;0] , [0;0] ) ;
        end
        
    end
end

% Save all tunings and robot parameters, so the conditions under
% which the results were obtained are known. Also save inputs and 
% measurements for later display.    
    
save inputLog ...
     rwheel trackGauge encoderRes nbReedSensors samplingPeriod ...
     xSpacing ySpacing  ...
     U sensorReadings ...
     Pinit Qgamma sigmaTuning Qbeta Qalpha mahaThreshold 

 
LogData( t , 'termination' , X , P , [0;0] , [0;0] ) ;
close(wbHandle) ;
