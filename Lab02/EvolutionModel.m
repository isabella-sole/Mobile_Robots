% Implements the evolution model of the system. Here, this model is simply
% the equations of odometry. Evolution model for the radius is a constant.

function Xnew = EvolutionModel( Xold , U )

% X=(x,y,theta,rR,rL)  U=(qRdot,qLdot)

global trackGauge ;

deltaD = (1/2)*( Xold(4)*U(1) + Xold(5)*U(2) ) ;
deltaTheta = Xold(4)/trackGauge*U(1) - Xold(5)/trackGauge*U(2) ;
    
Xnew = Xold + [ deltaD*cos(Xold(3)) ;   % x
                deltaD*sin(Xold(3)) ;   % y
                deltaTheta          ;   % theta
                0                   ;   % rL
                0                   ;   % rR
              ] ;
          
return          
