function L = lift(R, w, C, V, rho, a, Clo_p, Clo_m, Cla_p, Cla_m, s)
%calcs prasad's lift and torques using numerical integration
chi = R*w/V;

r = R/s:R/s:R;%starts at R/s then increments by step size R/s till R

dLo_p = 1:s; %makes vector for dLo_p so we can integrate over r later
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi range for trapz
    fLo = (cos(ang_psi)+r(dr)*chi/R).^2; %function we integrate over (a vector since ang_psi is vector)
    dLo_p(dr) = trapz(ang_psi,fLo); %if Clo is constant over the whole blade
end
Lo_p = (1/pi)*trapz(r, dLo_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for Lo_m

dLo_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fLo = (cos(ang_psi)+r(dr)*chi/R).^2;
    dLo_m(dr) = trapz(ang_psi,fLo); %either trapz or the prev commented out 2 lines works
end
Lo_m = (1/pi)*trapz(r, dLo_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Clo_ND = (Lo_p + (Clo_m/Clo_p)*Lo_m)/R; %divide by R since the constant multiplying Clo_ND in the lift equation contains an R which was taken out from the integration

Lo = 1/2*R*rho*V^2*C*Clo_p*Clo_ND;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we calculate the lift due to angle of attack
dLa_p = 1:s;
for dr = 1:s
    ang_psi = 0:real(pi-acos(r(dr)*chi/R))/s:real(pi-acos(r(dr)*chi/R)); %create values for psi
    fLa = abs(cos(ang_psi)+r(dr)*chi/R);
    dLa_p(dr) = trapz(ang_psi,fLa);
end
La_p = (1/pi)*trapz(r, dLa_p);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now we repeat the process for La_m
dLa_m = 1:s;
for dr = 1:s
    ang_psi = real(pi-acos(r(dr)*chi/R)):real(pi-(pi-acos(r(dr)*chi/R)))/s:pi; %create values for psi
    fLa = abs(cos(ang_psi)+r(dr)*chi/R); %negative sign since reversal of flow reverses angle of attack?? (i took it out and it was better)
    dLa_m(dr) = trapz(ang_psi,fLa);
end
La_m = (1/pi)*trapz(r, dLa_m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate non-dimensional basic lift coeff
Cla_ND = (La_p + (Cla_m/Cla_p)*La_m)/R; %divide by R since the constant multiplying Cla_ND in the lift equation contains an R which was taken out from the integration

La = 1/2*R*rho*V^2*C*a*Cla_p*Cla_ND; %where a is the angle of attack (alpha)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now calculate total lift
L = 3*(Lo + La); %for 3 blades so mult by 3

end

