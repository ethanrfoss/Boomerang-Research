classdef boomerang
    
    properties (Constant)
        CL0pDef = 0.39; 
        CL0mDef = -0.39; 
        CLapDef = 4.5; 
        CLamDef = 4.5;
        CD0pDef = 0.05; 
        CD0mDef = 0.05;
        CDapDef = 0.1;
        CDamDef = 0.1;
        
    end
    
    properties
        
        
        % Aerodynamic Coefficients
        CL0p %always positive
        CL0m %always negative
        CLap %always positive (rad^-1)
        CLam %always positive (rad^-1)
        CD0p %always positive 
        CD0m %always positive
        CDap %always positive (rad^-1)
        CDam %always positive (rad^-1)
        
        % Lengths
        R %Radius(m)
        C %Chord Length(m)
        Roffset %Offset of Lift(m)
        A %Area(m^2)
        l %Thickness
        
        
        % Mass, Inertia
        m %Mass of Boomerang(kg)
        rho %Density of Boomerang(kg/m^3)
        I %Inertia Tensor(kgm^2)
        
        % Other
        n %Blade Number
        
    end
    
    methods
        function obj = boomerang(Foil,R,C,A,Roffset,l,m,rho,I,n)
            
            if length(fieldnames(Foil)) == 8
                obj.CL0p = Foil.CL0p; 
                obj.CL0m = Foil.CL0m; 
                obj.CLap = Foil.CLap; 
                obj.CLam = Foil.CLam;
                obj.CD0p = Foil.CD0p; 
                obj.CD0m = Foil.CD0m;
                obj.CDap = Foil.CDap;
                obj.CDam = Foil.CDam;
            elseif isequal(Coef,'DefaultCoefficients')
                obj.CL0p = obj.CL0pDef; 
                obj.CL0m = obj.CL0mDef; 
                obj.CLap = obj.CLapDef; 
                obj.CLam = obj.CLamDef;
                obj.CD0p = obj.CD0pDef; 
                obj.CD0m = obj.CD0mDef;
                obj.CDap = obj.CDapDef;
                obj.CDam = obj.CDamDef;
            elseif isequal(Foil,'CustomAirfoil')
                disp('This function is not yet implemented');
                obj.CL0p = obj.CL0pDef; 
                obj.CL0m = obj.CL0mDef; 
                obj.CLap = obj.CLapDef; 
                obj.CLam = obj.CLamDef;
                obj.CD0p = obj.CD0pDef; 
                obj.CD0m = obj.CD0mDef;
                obj.CDap = obj.CDapDef;
                obj.CDam = obj.CDamDef;
            else
                disp('Invalid Input for Coefficients');
            end
            
            obj.R = R;
            obj.C = C;
            obj.Roffset = Roffset;
            if A < 0
                obj.A = n*R*C;
            else
                obj.A = A;
            end
            obj.l = l;
            
            obj.m = m;
            obj.rho = rho;
            if length(I) == 1 && I < 0
                obj.I = [1/4*m*R^2,0,0;0,1/4*m*R^2,0;0,0,1/2*m*R^2];
            elseif length(I) == 1 && I >=0
                obj.I = [.5*I,0,0;0,.5*I,0;0,0,I];
            elseif size(I) == [3,3]
                obj.I = I;
            else
                disp('Invalid Input for I');
            end
            
            obj.n = n;
            
        end
        
%         function obj = boomerang(Coef)
%             
%             if length(Coef) == 8
%                 CL0p = Coef(1); 
%                 CL0m = Coef(2); 
%                 CLap = Coef(3); 
%                 CLam = Coef(4);
%                 CD0p = Coef(5); 
%                 CD0m = Coef(6);
%                 CDap = Coef(7);
%                 CDam = Coef(8);
%             elseif Coef == 'DefaultCoefficients'
%                 CL0p = obj.CL0pDef; 
%                 CL0m = obj.CL0mDef; 
%                 CLap = obj.CLapDef; 
%                 CLam = obj.CLamDef;
%                 CD0p = obj.CD0pDef; 
%                 CD0m = obj.CD0mDef;
%                 CDap = obj.CDapDef;
%                 CDam = obj.CDamDef;
%             elseif Coef == 'CustomAirfoil'
%                 disp(Coef);
%             else
%                 disp('Invalid Input: ' + Coef);
%             end
%             
%             
%         end
    end
end
            
                
        