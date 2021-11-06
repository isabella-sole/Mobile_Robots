% Set the parameters which have the "Determined by studen" comment to tune
% the Kalman filter. Do not modify anything else in this file.

global Pinit Qgamma Qbeta mahaThreshold ;

% Uncertainty on initial position of the robot.

sigmaX     = *** ;         % Determined by student
sigmaY     = *** ;         % Determined by student
sigmaTheta = ****pi/180;     % Determined by student
sigmaRL    = *** ; 
sigmaRR    = sigmaRL ;
Pinit = diag( [ sigmaX^2 sigmaY^2 sigmaTheta^2 ...
                sigmaRL^2 sigmaRR^2 ] ) ;


% Measurement noise.

sigmaXmeasurement = *** ; %  % Determined by student
sigmaYmeasurement = *** ;  % Determined by student
Qgamma = diag( [sigmaXmeasurement^2 sigmaYmeasurement^2] ) ;


% Input noise

sigmaTuning = *** ; 
Qwheels = sigmaTuning^2 * eye(2) ;
Qbeta   = Qwheels ; 

% State noise
 
sigmaR = *** ; 
Qalpha = [  zeros(3)    zeros(3,2)    ;
           zeros(2,3) sigmaR^2*eye(2) ] ;

% Mahalanobis distance threshold

mahaThreshold = *** ;  % Determined by student
