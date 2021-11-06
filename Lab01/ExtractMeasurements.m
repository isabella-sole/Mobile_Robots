% Data pre-processing.
% Transforms the integer value raw data of the magnetic sensor into a
% vector of binary bits. One bit with value noMagnetDetected is added
% to the left and to the right of the bit vector so that presence of a
% magnet always results in two transitions from noMagnetDetected to
% magnetDetected and vice versa.
% The measurements are the values located mid-way between the two
% transitions.

function measurements = ExtractMeasurements( ...
    rawSensorData, nbReedSensors, magnetDetected ) 

bitVector = [  not(magnetDetected) bitget(rawSensorData,1:nbReedSensors) not(magnetDetected)] ;
vectSize  = nbReedSensors + 2 ;
% Look for first transition in bit vector, if any
i = 2 ;
while (i<=vectSize) && (bitVector(i)==not(magnetDetected))
    i = i+1 ;
end
nbMeasurements = 0 ;
measurements = zeros(1,2) ;

while i < vectSize
    indBegin = i ;
    while bitVector(i)==magnetDetected
        i = i+1 ;
    end
    indEnd = i-1 ;
    nbMeasurements = nbMeasurements + 1 ;
    measurements(nbMeasurements) = (indBegin+indEnd)/2 - 1 ;
    while (i<vectSize) && (bitVector(i)==not(magnetDetected))
        i = i+1 ;
    end
end
measurements = measurements(1:nbMeasurements) ;

return
