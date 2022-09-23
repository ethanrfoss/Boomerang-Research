classdef newBoomerangSimulation
    
    properties
        
        % Throw Inputs:
        V0 %initial speed magnitude(m/s)
        Z0 %initial height(m)
        w0 %spin(rad/s)
        
        % Angular Inputs
        thetaHor0 %(rad)
        thetaLay0 %(rad)
        alpha0 %(rad)
        
        % Wind
        Vw
        WindAng
        
        % Other
        dt
        tmax
        
    end
    
    methods
        function obj = newBoomerangSimulation(V0,Z0,w0,thetaHor0,thetaLay0,alpha0,Vw,WindAng,dt,tmax)
            
            obj.V0 = V0;
            obj.Z0 = Z0;
            obj.w0 = w0;
            obj.thetaHor0 = thetaHor0;
            obj.thetaLay0 = thetaLay0;
            obj.alpha0 = alpha0;
            obj.Vw = Vw;
            obj.WindAng = WindAng;
            obj.dt = dt;
            obj.tmax = tmax;
            
        end
    end
    
    methods (Static)
        function userSave(S,B,t,boomState,flightParams,f)
            
            user = input('Save Simulation Initial Conditions, Boomerang Profile, and Simulation Data? Type yes or no: ','s');
            if isequal(lower(user),'yes')
                saveName = input('Enter File Save Name: ','s');
                save([cd '\SavedSims\' saveName '.mat'],'S','B','t','boomState','flightParams','f');
                disp('Data Saved');
            else
                disp('Data Discarded');
            end
            
        end
    end
end