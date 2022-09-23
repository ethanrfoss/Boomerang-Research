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
        Foil % Includes Aerodynamic Coefficients
        
        % Lengths
        R %Radius(m)
        C %Chord Length(m)
        Roffset %Offset of Lift(m)
        A %Area(m^2)
        l %Thickness(m)
        
        
        % Mass, Inertia
        m %Mass of Boomerang(kg)
        rho %Density(kg/m^3)
        I %Inertia Tensor(kgm^2)
        
        % Other
        n %Blade Number
        Poly %Polyshape of 2D Boomerang
        X %X-Data of Boomerang
        Y %Y-Data of Boomerang
        Z %Z-Data of Boomerang
        Bound % Boundary of 3D Boomerang
    end
    
    methods
        function obj = boomerang(Foil,R,C,A,Roffset,l,m,rho,I,n)
            
            % Coefficients:
            obj.Foil = Foil;
            
            % Lengths:
            obj.R = R;
            obj.C = C;
            obj.Roffset = Roffset;
            if A < 0
                obj.A = n*R*C;
            else
                obj.A = A;
            end
            obj.l = l;
            
            % Mass Properties:
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
            
            % Other:
            obj.n = n;
            
            
            % Polyshape:
            warning('off');
            blade1 = polyshape([C/2 C/2 -C/2 -C/2],[0 R R 0]);
            blade2 = rotate(blade1,120);
            blade3 = rotate(blade1,240);
            circle = polyshape(R/4*cos((0:1/32:1)*2*pi),R/4*sin((0:1/32:1)*2*pi));
            obj.Poly = union([blade1,blade2,blade3,circle]);
            warning('on');
            
            % XYZ Coordinates of Boomerang Vertices
            obj.X = [obj.Poly.Vertices(:,1); obj.Poly.Vertices(:,1)];
            obj.Y = [obj.Poly.Vertices(:,2); obj.Poly.Vertices(:,2)];
            obj.Z = [zeros(length(obj.Poly.Vertices),1); l*ones(length(obj.Poly.Vertices),1)];
            
            % Boundary:
            obj.Bound = boundary(obj.X,obj.Y,obj.Z);
            
        end
        
        function obj = set.Foil(obj,Foil)
            
            if ~ischar(Foil) && length(fieldnames(Foil)) == 8
                obj.Foil.CL0p = Foil.CL0p; 
                obj.Foil.CL0m = Foil.CL0m; 
                obj.Foil.CLap = Foil.CLap; 
                obj.Foil.CLam = Foil.CLam;
                obj.Foil.CD0p = Foil.CD0p; 
                obj.Foil.CD0m = Foil.CD0m;
                obj.Foil.CDap = Foil.CDap;
                obj.Foil.CDam = Foil.CDam;
            elseif isequal(Foil,'DefaultCoefficients')
                obj.Foil.CL0p = obj.CL0pDef; 
                obj.Foil.CL0m = obj.CL0mDef; 
                obj.Foil.CLap = obj.CLapDef; 
                obj.Foil.CLam = obj.CLamDef;
                obj.Foil.CD0p = obj.CD0pDef; 
                obj.Foil.CD0m = obj.CD0mDef;
                obj.Foil.CDap = obj.CDapDef;
                obj.Foil.CDam = obj.CDamDef;
            elseif isequal(Foil,'CustomAirfoil')
                disp('This function is not yet implemented');
                obj.Foil.CL0p = obj.CL0pDef; 
                obj.Foil.CL0m = obj.CL0mDef; 
                obj.Foil.CLap = obj.CLapDef; 
                obj.Foil.CLam = obj.CLamDef;
                obj.Foil.CD0p = obj.CD0pDef; 
                obj.Foil.CD0m = obj.CD0mDef;
                obj.Foil.CDap = obj.CDapDef;
                obj.Foil.CDam = obj.CDamDef;
            else
                disp('Invalid Input for Coefficients');
            end
            
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
            
                
        