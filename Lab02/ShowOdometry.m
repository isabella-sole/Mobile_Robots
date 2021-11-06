% ShowOdometry: Plot the odometry-estimated path corresponding to a data
% file. Uses nominal values of the wheel radii (from RobotAndSensorDefinition.m).
% Usage: ShowOdometry
% Remark: Update initial posture in the code if necessary.
% Author: G. Garcia
% March 2019.

RobotAndSensorDefinition ;

Xodom = [ 0 0 0*pi/180 ].' ;    % Set this according to robot initial position.
%Load the data file
dataFile = uigetfile('data/*.txt','Select data file') ;
if isunix 
    eval(['load data/' , dataFile]) ;
else
    eval(['load data\' , dataFile]) ;
end
dataFile = strrep(dataFile, '.txt', '');
eval(['data = ',dataFile,'; clear ',dataFile]) ;
travDistance = 0 ;

% Skip motionless parts of the data at beginning and end of the experiment
% and return only meaningful data, with wheel rotations in radians.
% Also reduce encoder resolution and frequency according to factors
% set in RobotDefinition.m

[nbLoops,t,qL,qR,sensorReadings] = PreprocessData(...
    data, dots2rad, dumbFactor, subSamplingFactor ) ;

Xodo = zeros(3,nbLoops) ;
vOdo = zeros(1,nbLoops) ;
wOdo = zeros(1,nbLoops) ;

Xodo(:,1) = Xodom ;

wbHandle = waitbar(0,'Computing...') ;

for mainLoopIndex = 2 : nbLoops
    
    waitbar(mainLoopIndex/nbLoops) ;

    % Calculate input vector from proprioceptive sensors
    deltaq = [ qR(mainLoopIndex) - qR(mainLoopIndex-1) ; 
               qL(mainLoopIndex) - qL(mainLoopIndex-1) ] ;
    % The jointToCartesian matrix is calculated with nominal radii.
    % Calculate elementary travelled distance and elementary rotation.
    Ucartesian = jointToCartesian * deltaq ;  
    
    % Predic state (here odometry)
    Xodom = Xodom + [ Ucartesian(1)*cos(Xodom(3)) ;
                Ucartesian(1)*sin(Xodom(3)) ;
                Ucartesian(2)              ;
              ] ;
    Xodo(:,mainLoopIndex) = Xodom ;
    vOdo(mainLoopIndex) = Ucartesian(1)/( t(mainLoopIndex)-t(mainLoopIndex-1) ) ;
    wOdo(mainLoopIndex) = Ucartesian(2)/( t(mainLoopIndex)-t(mainLoopIndex-1) ) ;
    
end

close(wbHandle) ;

% Plot robot path, Kalman fiter estimation and odometry only estimation

figure; 
plot( Xodo(1,:), Xodo(2,:) , 'r' , 'LineWidth',2 ) ;
zoom on ; grid on; axis('equal');
title('Path estimated by odometry (nominal wheel radii)');
xlabel('x (mm)');
ylabel('y (mm)');

% Plot odometry-estimated speed and rotation speed

figure; 
subplot(2,1,1);
plot( t,vOdo , 'LineWidth',2 );
xlabel('t (s)')
ylabel('v (mm/s)');
title('Odometry-estimated speed (nominal wheel radii)');
zoom on ; grid on;
subplot(2,1,2);
plot( t,wOdo*180/pi , 'LineWidth',2 );
xlabel('t (s)')
ylabel('w (deg/s)');
title('Odometry-estimated rotation speed (nominal wheel radii)');
zoom on ; grid on;
