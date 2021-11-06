% A script to display the results of the last execution of MagnetLoc.
% MagnetLoc stores its results in 'log.txt' and the meaningful input data
% in inputLog.mat (among which robot parameters and noise variances).
% Graphs displayed:
% Figure 1: 
%   - The path calculated by odometry only (red).
%   - The path estimated by the Kalman filter (blue).
%   - The locations of the magnets which have been detected (black dots).
%   - The estimated locations of the detected magnets in absolute frame
%       using the measurement (that's variable oMeasMagnet in the program).
% Figure 2: 
%   - Speed and rotation speed, as estimated using the encoders.
% Figure 3: 
%   - Estimated error standard deviations (extracted directly from
%       the diagonal of P, hence in absolute frame.
% Figure 4: 
%   - Estimated error standard deviations in robot frame.
% Figure 5: 
%   - Mahalanobis distances calculated with the magnet closest to 
%       the measurement point (candidate magnet) in blue.
%   - Mahalanobis distances calculated with the four nearest neighbors 
%       of the candidate magnet in red.
%   - Mahalanobis distance threshold used in the program (black line).
% Figure 6: 
%   - Estimated x, y, theta as functions of time.
% Figure 7:
%   - Number of magnets detected at each time instant.
% Figure 8:
%   - Raw sensor measurements as a function of the curvilinear abscissa
%       (distance traveled by point M). The vertical axis represents the
%       state of each Reed sensor. A vertical line indicates a closed 
%       sensor (a sensor which detects a magnet).
%   - You may comment out this graph when you don't need it anymore 
%       (when you're done estimating the measurement noise).

function PlotResults

% Load the inputs to the problem (robot charateristics + tuning +
% speed and rotation speed + measurements. They have been saved by
% MagnetLoc in inputLog.mat.

if ~exist('inputLog.mat','file')
    disp('File inputLog.mat not found. Did you run MagnetLoc first?');
    return
end
if ~exist('log.txt','file')
    disp('File log.txt not found. Did you run MagnetLoc first?');
    return
end

load inputLog ...
     nbReedSensors samplingPeriod xSpacing ySpacing  ...
     sensorReadings Qgamma mahaThreshold

 
% Load the results calculated by MagnetLoc, logged in log.txt.
fid = fopen('log.txt','r');
firstline = fgetl(fid) ;
numvars = numel(strread(firstline,'%s')); %#ok<DSTRRD>
fgetl(fid); %<-- Skip the second line
data = textscan(fid,repmat('%f',1,numvars)); %#ok<NASGU>
% The "unused variable" warning has been suppressed: variable "data" is
% used in the "eval" instruction, but code analyzer does not see it.

% Next instruction sets all variables whose name are on the first line
% of the file, here calcPhase, t, x, y, theta, P11...P33, U1, U2, Y1, Y2.
eval(['[' strrep(firstline,' ',',') '] = deal(data{:});']) ;
fclose(fid);

% Prepare vectors and matrices
nbRes = length(t) ;
nbPeriods = sum(calcPhase==1) ;
U    = zeros(2,nbPeriods) ;
Xodo = zeros(3,nbPeriods+1) ;

% Reconstruct the inputs. Suppress lines that are non prediction, since 
% their values have been set to zero conventionally in the logs.
% Keeping those would be impractical when displaying velocities.
U(1,:) = U1(find(calcPhase==1)) ;
U(2,:) = U2(find(calcPhase==1)) ;
tOdo = t(find(calcPhase==1));

% Calculate the measured positions of the magnets in the absolute frame
% and the magnets closest to these measured positions for display. 
% Do this only when calcPhase = 2 (measurement).
nbMeasurementPhases = sum(calcPhase==2) ;
estMagnetPos   = zeros(2,nbMeasurementPhases) ;
exactMagnetPos = zeros(2,nbMeasurementPhases) ;
j = 0 ;
for i = 1 : nbRes 
    if calcPhase(i) == 2
        j = j+1 ;
        oTm = [ cos(theta(i)) , -sin(theta(i)) , x(i) ;
                sin(theta(i)) ,  cos(theta(i)) , y(i) ;
                      0       ,        0       ,  1   ] ;
        oEstimatedMagnet = oTm * [ y1(i) ; y2(i) ; 1 ] ;
        oExactMagnetPos  = round( oEstimatedMagnet ./ [xSpacing ; ySpacing ; 1] ) .* [xSpacing ; ySpacing ; 1] ;
        estMagnetPos(:,j)   = oEstimatedMagnet(1:2) ;
        exactMagnetPos(:,j) = oExactMagnetPos(1:2)  ;
    end
end

% Compute odometry only estimated path
Xodo(:,1) = [x(1) ; y(1) ; theta(1)] ;
for i = 1 : nbPeriods
    Xodo(:,i+1) = Xodo(:,i) + ...
        [ U(1,i)*cos(Xodo(3,i)) ;
          U(1,i)*sin(Xodo(3,i)) ;
          U(2,i) ] ;
end
travDistance = zeros(1,nbPeriods) ;
for i = 2 : nbPeriods
    travDistance(i) = travDistance(i-1) + U(1,i) ;
end

% Plot robot path, Kalman fiter estimation and odometry only estimation

figure; 
plot( Xodo(1,:), Xodo(2,:) , 'r' , 'LineWidth', 2 ) ;
hold on ;
plot( x,y , 'b' , 'LineWidth', 2 ) ;
zoom on ; grid on; axis('equal');
title('Estimated path EKF (blue) and odometry (red)');
xlabel('x (mm)');
ylabel('y (mm)');

% On top of the path, indicate estimated and real magnet positions.

hold on;
plot( estMagnetPos(1,:), estMagnetPos(2,:) , 'g+' ) ;
hold on;
plot( exactMagnetPos(1,:), exactMagnetPos(2,:) , 'k.' ) ;

% Plot odometry-estimated speed and rotation speed

figure; 
subplot(2,1,1);
plot( tOdo,U(1,:)/samplingPeriod , 'LineWidth',2 );
xlabel('t (s)');
ylabel('v (mm/s)');
title('Odometry-estimated speed');
zoom on ; grid on;
subplot(2,1,2);
plot( tOdo,U(2,:)*180/pi/samplingPeriod , 'LineWidth',2 );
xlabel('t (s)')
ylabel('w (deg/s)' , 'LineWidth',2 );
title('Odometry-estimated rotation speed');
zoom on ; grid on;


% Plot estimated variances in absolute reference frame

sigx     = sqrt(P11) ;
sigy     = sqrt(P22) ;
sigtheta = sqrt(P33) ;
tRes = t( find(calcPhase==3) ) ;
figure;
subplot(3,1,1);
maximum = max(sigx) ;
for k=1:numel(tRes) 
    line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':' ,...
       'LineWidth',2 );      
end
hold on ;
plot( t,sigx , 'LineWidth',2 );
xlabel('t (s)') ;
ylabel('sigma_x (mm)') ;
title('Estimated standard deviations in absolute ref. frame');
title('Est. std dev. in abs. ref. frame');
zoom on ; grid on;
set(gca,'FontSize',14)
subplot(3,1,2);
maximum = max(sigy) ;
for k=1:numel(tRes) 
   line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':' , ...
       'LineWidth',2 );      
end
hold on ;
plot( t,sigy , 'LineWidth',2 );
xlabel('t (s)') ;
ylabel('sigma_y (mm)');
zoom on ; grid on;
subplot(3,1,3);
maximum = max(sigtheta*180/pi) ;
for k=1:numel(tRes) 
   line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':' ,...
       'LineWidth',2 );      
end
hold on ;
plot( t,sigtheta*180/pi , 'LineWidth',2 );
xlabel('t (s)') ;
ylabel('sigma_{theta} (deg.)');
zoom on ; grid on;

% Calculate covariance matrix in frame Rm
msigx     = zeros(1,length(t)) ;
msigy     = zeros(1,length(t)) ;
msigtheta = zeros(1,length(t)) ;
for i = 1 : length(t)
    m_Omega_o = [  cos(theta(i))  ,  sin(theta(i))  ,  0  ;
                  -sin(theta(i))  ,  cos(theta(i))  ,  0  ;
                        0         ,       0         ,  1  ] ;
    oP = [ P11(i)  ,  P12(i)  ,  P13(i)  ;
           P12(i)  ,  P22(i)  ,  P23(i)  ;
           P13(i)  ,  P23(i)  ,  P33(i)  ] ;
    mP = m_Omega_o * oP * m_Omega_o.' ;
    msigx(i)     = sqrt( mP(1,1) )   ;
    msigy(i)     = sqrt( mP(2,2) )   ;
    msigtheta(i) = sqrt( mP(3,3) )   ;
end

% Plot variances in robot frame.

figure;
subplot(3,1,1);
maximum = max(msigx) ;
for k=1:numel(tRes) 
   line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':' ,...
       'LineWidth',2 );      
end
hold on ;
plot( t,msigx , 'LineWidth',2 );
xlabel('t (s)') ;
ylabel('sigma_x (mm)');
title('Estimated standard deviations in robot frame');
zoom on ; grid on;
subplot(3,1,2);
maximum = max(msigy) ;
for k=1:numel(tRes) 
   line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':' , ...
       'LineWidth',2 );      
end
hold on ;
plot( t,msigy , 'LineWidth',2 );
xlabel('t (s)') ;
ylabel('sigma_y (mm)');
zoom on ; grid on;
subplot(3,1,3);
maximum = max(msigtheta*180/pi) ;
for k=1:numel(tRes) 
   line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':' ,...
       'LineWidth',2 );      
end
hold on ;
plot( t,msigtheta*180/pi , 'LineWidth',2 );
xlabel('t (s)') ;
ylabel('sigma_{theta} (deg.)');
zoom on ; grid on;

% Calculate Mahalanobis distances, including for magnets that are the 
% closest neighbors of the magnet closest to measurement point.

tMagnetDetection = zeros(1,sum(calcPhase==2)) ;
dMahaAll = zeros(5,sum(calcPhase==2)) ;
j = 0 ;
for i = 1 : length(t) 
    
    if calcPhase(i) ~= 2 
        continue ;  % Not a measurement phase
    end
     
    j = j+1 ;
    tMagnetDetection(j) = t(i) ;
    
    % Calculate homogeneous transform of the robot with respect to the world frame
    oTm = [ cos(theta(i)) , -sin(theta(i)) , x(i)  ;
            sin(theta(i)) ,  cos(theta(i)) , y(i)  ;
                  0       ,        0       ,  1    ] ;
    
    % Measurement vector: coordinates of the magnet measured in Rm.
    Y = [ y1(i) ; y2(i) ] ;
    
    % Now in homogeneous coordinates for calculations.
    mMeasMagnet = [ Y ; 1 ] ;
    
    % Corresponding position in absolute frame
    oMeasMagnet = oTm * mMeasMagnet ;
    
    % Which actual magnet is closest to the estimated position?
    oRealMagnet = round( oMeasMagnet ./ [xSpacing ; ySpacing ; 1] ) .* [xSpacing ; ySpacing ; 1] ;
    
    % The position of the real magnet in robot frame
    mRealMagnet = oTm \ oRealMagnet ; % That's inv(oTm)*oRealMagnet
    
    % The expected measurement are the two coordinates of the real
    % magnet in the robot frame.
    Yhat = mRealMagnet(1:2) ;
    
    C = [ -cos(theta(i)) , -sin(theta(i)) , -sin(theta(i))*(oRealMagnet(1)-x(i))+cos(theta(i))*(oRealMagnet(2)-y(i)) ;
           sin(theta(i)) , -cos(theta(i))   -sin(theta(i))*(oRealMagnet(2)-y(i))-cos(theta(i))*(oRealMagnet(1)-x(i)) ] ;
    
    innov = Y - Yhat ;
    P = [ P11(i)  ,  P12(i)  ,  P13(i)  ;
          P12(i)  ,  P22(i)  ,  P23(i)  ;
          P13(i)  ,  P23(i)  ,  P33(i)  ] ;
    dMaha = sqrt( innov.' / ( C*P*C.' + Qgamma) * innov ) ;
          
    dMahaAll(1,j)       = dMaha            ;
    estMagnetPos(:,j)   = oMeasMagnet(1:2) ;
    exactMagnetPos(:,j) = oRealMagnet(1:2) ;

    % Offset vectors to generate the neighbors, in homogeneous coordinates.

    deltas = [ xSpacing  -xSpacing      0           0       ;
                  0          0       ySpacing   -ySpacing   ;
                  0          0          0           0       ] ;

    for neighborIndex = 1 : 4
        oPneighbor = oRealMagnet + deltas(:,neighborIndex) ;

        % The position of the magnet in robot frame is the expected measurement
        % YhatNeighbor
        YhatNeighbor = oTm \ oPneighbor ; % That's inv(oTm)*oPneighbor

        Cneighbor = [ -cos(theta(i)) , -sin(theta(i)) , -sin(theta(i))*(oPneighbor(1)-x(i))+cos(theta(i))*(oPneighbor(2)-y(i)) ;
                       sin(theta(i)) , -cos(theta(i)) , -sin(theta(i))*(oPneighbor(2)-y(i))-cos(theta(i))*(oPneighbor(1)-x(i)) ] ;

        innovNeighbor = Y(1:2) - YhatNeighbor(1:2) ;    % Not in homogeneous coordinates.
        dMahaNeighbor = sqrt( innovNeighbor.' / ( Cneighbor*P*Cneighbor.' + Qgamma) * innovNeighbor ) ;
        dMahaAll(neighborIndex+1,j) = dMahaNeighbor ;
    end
    
end

% Plot Mahalanobis distances. Blue dots are for closest magnet,
% red dots are for neighbor magnets.

figure; 
plot( tMagnetDetection , dMahaAll(1,:) , 'bo' , 'LineWidth',2 ) ;
for k = 2:5
    hold on; 
    plot( tMagnetDetection , dMahaAll(k,:) , 'ro' , 'LineWidth',2 ) ;
end
hold on;
plot( tMagnetDetection , mahaThreshold*ones(1,size(dMahaAll,2)) , 'k' ,...
    'LineWidth',2 ) ; 
xlabel('t (s)');
ylabel('Mahalanobis distance (no dimension).');
title({'Mahalanobis distances:','closest magnet (blue) and neighbors (red)'});
zoom on; grid on; 

% Plot x, y and theta as functions of time

figure; 

subplot(3,1,1);
plot( t,x , 'LineWidth',2 );
xlabel('t (s)')
ylabel('x (mm)');
title('Position and heading as functions of time.');
zoom on ; grid on;

subplot(3,1,2);
plot( t,y , 'LineWidth',2 );
xlabel('t (s)')
ylabel('y (mm)');
zoom on ; grid on;

subplot(3,1,3);
plot( t,theta*180/pi , 'LineWidth',2 );
xlabel('t (s)')
ylabel('theta (deg.)');
zoom on ; grid on;

% Determine the number of measurements (i.e. the number of detected magnets)
% at each step. The idea is to show the students that, in most cases,
% a single magnet is detected (or, of course, zero).

i = 1 ;
k = 0 ;
tMeas  = zeros(1,nbPeriods) ;
nbMeas = zeros(1,nbPeriods) ;
while i <= length(t)
    k = k+1 ;
    tMeas(k) = t(i) ;
    nbMeas(k) = 0 ;
    j = 1 ;
    while (i+j)<=length(t) && t(i+j)==t(i)
        if y1(i+j) ~= 0 
            nbMeas(k) = nbMeas(k)+1 ;
        end
        j = j+1 ;
    end
    i = i+j ;
end

% Plot number of measurements at each time step.

figure; 
plot(tMeas,nbMeas,'o' , 'LineWidth', 2 ) ;
xlabel('time (s)');
%yticks([0 1 2]);
title('Number of magnets detected at each step.');
zoom on; grid on;


% Plot raw sensor measurements
rawMeas = zeros( nbReedSensors , nbPeriods ) ;
for k = 1 : nbPeriods
    rawMeas( : , k ) = bitget( sensorReadings(k) , 1:8 ) ;
end
figure; 
for n = 1 : nbReedSensors
    for k = 1 : nbPeriods
        if rawMeas(n,k) == 0
            hold on ;
            line([travDistance(k) travDistance(k)],[n-0.5 n+0.5],'Color','b','LineStyle','-' , 'LineWidth', 2 );
        end
    end
end
set(gca,'YLim',[0 nbReedSensors+1]) ;
xlabel('Travelled distance of point M (mm)');
ylabel('Reed sensor number') ;
title('State of Reed sensors (blue = magnet dectected)') ;
zoom on; grid on;

% Calculate and display odometry error (assuming KF is right).

fprintf('\nTotal travelled distance: %d mm\n',round(sum(abs(U1))));
fprintf('Final odometry error: %3.1f %%\n\n', ...
    (norm([x(size(x,1));y(size(y,1))]-Xodo(1:2,size(Xodo,2))) / sum(abs(U1)) )*100 );

% Calculate percentage of rejected closest magnets:

fprintf('Magnets rejected: %3.1f %%\n', ...
    100*numel(find(dMahaAll(1,:) > mahaThreshold ))/numel(dMahaAll(1,:)));

fprintf('Neighbor magnets under threshold: %3.1f %%\n\n', ...
    100*numel(find(dMahaAll(2:5,:) <= mahaThreshold ))/numel(dMahaAll(2:5,:))) ;
