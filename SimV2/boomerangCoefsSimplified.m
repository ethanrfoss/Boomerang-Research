function [CL, CD, CMX, CMY, CMZ] = boomerangCoefsSimplified(Xi,alpha,B)

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
CL0 = .822747*Xi-.114013;
% Angle of Attack
CLa = .253406*Xi+.5115438;
% Total
CL = real(CL0p*CL0+CLap*CLa*alpha);
%% Coefficient of Drag
% No Angle of Attack
CD0 = .36274*Xi+.272797;
% Angle of Attack
CDa = .5;
% Total
CD = real(CD0p*CD0+CDap*CDa*alpha);
%% Coefficient of Rolling Moment
% No Angle of Attack
CMX0 = .2663238*Xi+.102435;
% Angle of Attack
CMXa = .119789*Xi+.0581454;
% Total
CMX = real(CL0p*CMX0+CLap*CMXa*alpha);
%% Coefficient of Pitching Moment
% No Angle of Attack
CMY0 = .5*Xi;
% Angle of Attack
CMYa = .5;
% Total
CMY = real(CL0p*CMY0 + CLap*CMYa*alpha);
%% Coefficient of Drag Force Moment
% No Angle of Attack
CMZ0 = .574602*Xi-.092139;
% Angle of Attack
CMZa = .333333*Xi;
% Total
CMZ = real(CD0p*CMZ0+CDap*CMZa*alpha);
end