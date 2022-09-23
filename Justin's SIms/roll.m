function Mx = roll(R, w, C, V, rho, a, Clo_p, Clo_m, Cla_p, Cla_m, s)
% calcs Prasad's Mx (rolling moment)
chi=w*R/V;

%input s is number of segments

r = R/s:R/s:R;%starts at R/s then increments by step size R/s till R

dMxo_p = 1:s; %makes vector for dLo_p so we can integrate over r later
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi range for trapz
    fMxo = (cos(ang_psi)+r(dr)*chi/R).^2 *r(dr).*cos(ang_psi) ; %function we integrate over (a vector since ang_psi is vector)
    dMxo_p(dr) = trapz(ang_psi,fMxo); %either trapz or the previously commented out 2 lines works
end
Mxo_p = (1/pi)*trapz(r, dMxo_p); %this is same as paper except no constant multiplying the outside yet
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for Lo_m

dMxo_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fMxo = (cos(ang_psi)+r(dr)*chi/R).^2 *r(dr).*cos(ang_psi);
    dMxo_m(dr) = trapz(ang_psi,fMxo); %either trapz or the prev commented out 2 lines works
end
Mxo_m = (1/pi)*trapz(r, dMxo_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cmxo_ND = (Mxo_p + (Clo_m/Clo_p)*Mxo_m)/R^2; %divide by R^2 since the constant multiplying Clo_ND in the rolling mom. equation contains an R^2 which was taken out from the integration

Mxo = 1/2*R^2*rho*V^2*C*Clo_p*Cmxo_ND;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we calculate the lift due to angle of attack
dMxa_p = 1:s;
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi
    fMxa = abs(cos(ang_psi)+r(dr)*chi/R)*r(dr).*cos(ang_psi);
    dMxa_p(dr) = trapz(ang_psi,fMxa);
end
Mxa_p = (1/pi)*trapz(r, dMxa_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for La_m
dMxa_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fMxa = abs(cos(ang_psi)+r(dr)*chi/R)*r(dr).*cos(ang_psi); %negative sign since reversal of flow reverses angle of attack?? (i took it out and it was better)
    dMxa_m(dr) = trapz(ang_psi,fMxa);
end
Mxa_m = (1/pi)*trapz(r, dMxa_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cmxa_ND = (Mxa_p + (Cla_m/Cla_p)*Mxa_m)/R^2; %divide by R^2 since the constant multiplying Cla_ND in the lift equation contains an R^2 which was taken out from the integration

Mxa = 1/2*R^2*rho*V^2*C*a*Cla_p*Cmxa_ND; %where a is the angle of attack (alpha)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate total lift
Mx = 3*(Mxo + Mxa); %for 3 baldes so mult  by 3

end

