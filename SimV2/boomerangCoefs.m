function [CL, CD, CMX, CMY, CMZ] = boomerangCoefs(Xi,alpha,B)

%% Aerodynamic constants (p is indicates the leading edge flow region, and m indicates the trailing edge flow region)
CL0p = B.Foil.CL0p; %always positive
CL0m = B.Foil.CL0m; %always negative
CLap = B.Foil.CLap; %always positive (rad^-1)
CLam = B.Foil.CLam; %always positive (rad^-1)
CD0p = B.Foil.CD0p; %always positive 
CD0m = B.Foil.CD0m; %always positive
CDap = B.Foil.CDap; %always positive (rad^-1)
CDam = B.Foil.CDam; %always positive (rad^-1)

%% Coeffiecient of Lift
% No Angle of Attack
CL0 = (1/2+Xi^2/3)+(1-CL0m/CL0p)*(1/(2*pi*Xi)*(sqrt(1-Xi^2)-(1-Xi^2)^1.5-Xi*acos(Xi))-1/(3*pi*Xi)*(Xi^3*acos(Xi)-1/3*((2+Xi^2)*sqrt(1-Xi^2)-2)));
% Angle of Attack
CLa = Xi/2 + 1/(2*pi)*(1+CLam/CLap)*(3/2*sqrt(1-Xi^2)+asin(Xi)/(2*Xi)-Xi*acos(Xi));
% Total
CL = real(CL0p*CL0+CLap*CLa*alpha);
%% Coefficient of Drag
% No Angle of Attack
CD0 = Xi/2+(1+CD0m/CD0p)*(1/(3*pi)*(sqrt(1-Xi^2)+asin(Xi)/Xi)+1/(24*pi)*((2*Xi^2-1)*sqrt(1-Xi^2)+asin(Xi)/Xi)-1/(4*pi)*(asin(Xi)/Xi+2*Xi*acos(Xi)-sqrt(1-Xi^2)));
% Angle of Attack
CDa = 1/2-(1-CDam/CDap)*1/(2*pi*Xi)*(2/3+Xi*acos(Xi)-sqrt(1-Xi^2)+1/3*(1-Xi^2)^1.5);
% Total
CD = real(CD0p*CD0+CDap*CDa*alpha);
%% Coefficient of Rolling Moment
% No Angle of Attack
CMX0 = Xi/3 +1/3*(1-CL0m/CL0p)*(1/(15*pi*Xi^2)*(2+(3*Xi^4-Xi^2-2)*sqrt(1-Xi^2))+2/(3*pi*Xi^2)*(1-(1-Xi^2)^1.5)-1/pi*(Xi*acos(Xi)-1/(3*Xi^2)*((2+Xi^2)*sqrt(1-Xi^2)-2)));
% Angle of Attack
CMXa = 1/4+(1+CLam/CLap)*(1/(16*pi*Xi)*((2*Xi^2-1)*sqrt(1-Xi^2)+asin(Xi)/Xi)-1/(8*pi*Xi)*(asin(Xi)/Xi+2*Xi*acos(Xi)-sqrt(1-Xi^2)));
% Total
CMX = real(CL0p*CMX0+CLap*CMXa*alpha);
%% Coefficient of Pitching Moment
% No Angle of Attack
CMY0 = Xi/2+(1+CL0m/CL0p)*(1/(3*pi)*(sqrt(1-Xi^2)+asin(Xi)/Xi)+1/(24*pi)*((2*Xi^2-1)*sqrt(1-Xi^2)+asin(Xi)/Xi)-1/(4*pi)*(asin(Xi)/Xi+2*Xi*acos(Xi)-sqrt(1-Xi^2)));
% Angle of Attack
CMYa = 1/2+(1-CLam/CLap)*(1/(6*pi*Xi)*(1-(1-Xi^2)^1.5)-1/(2*pi*Xi)*(1+Xi*acos(Xi)-sqrt(1-Xi^2)));
% Total
CMY = real(CL0p*CMY0 + CLap*CMYa*alpha);
%% Coefficient of Drag Force Moment
% No Angle of Attack
CMZ0 = 1/4+Xi^2/4-(1+CD0m/CD0p)*(-3/(16*pi*Xi)*((2*Xi^2-1)*sqrt(1-Xi^2)+asin(Xi)/Xi)+1/(8*pi*Xi)*(asin(Xi)/Xi+2*Xi*acos(Xi)-sqrt(1-Xi^2))+1/(32*pi*Xi)*(8*Xi^3*acos(Xi)-(2*Xi^2+3)*sqrt(1-Xi^2)+3*asin(Xi)/Xi));
% Angle of Attack
CMZa = Xi/3+(1-CDam/CDap)*(1/(3*pi*Xi^2)*(1-(1-Xi^2)^1.5)-1/(3*pi)*(Xi*acos(Xi)-1/(3*Xi^2)*((Xi^2+2)*sqrt(1-Xi^2)-2)));
% Total
CMZ = real(CD0p*CMZ0+CDap*CMZa*alpha);
end