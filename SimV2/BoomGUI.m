classdef BoomGUI < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        SavedBoomerangsPanel           matlab.ui.container.Panel
        SavedBoomsListBoxLabel         matlab.ui.control.Label
        SavedBoomsListBox              matlab.ui.control.ListBox
        LoadBoomButton                 matlab.ui.control.Button
        SavedAirfoilsPanel             matlab.ui.container.Panel
        SavedFoilsListBoxLabel         matlab.ui.control.Label
        SavedFoilsListBox              matlab.ui.control.ListBox
        LoadFoilButton                 matlab.ui.control.Button
        BoomerangParametersPanel       matlab.ui.container.Panel
        AreaEditFieldLabel             matlab.ui.control.Label
        AreaEditField                  matlab.ui.control.NumericEditField
        ThicknessEditFieldLabel        matlab.ui.control.Label
        ThicknessEditField             matlab.ui.control.NumericEditField
        DensityEditFieldLabel          matlab.ui.control.Label
        DensityEditField               matlab.ui.control.NumericEditField
        MassEditFieldLabel             matlab.ui.control.Label
        MassEditField                  matlab.ui.control.NumericEditField
        InertiazEditFieldLabel         matlab.ui.control.Label
        InertiazEditField              matlab.ui.control.NumericEditField
        InertiaxEditFieldLabel       matlab.ui.control.Label
        InertiaxEditField            matlab.ui.control.NumericEditField
        InertiayEditFieldLabel         matlab.ui.control.Label
        InertiayEditField              matlab.ui.control.NumericEditField
        RadiusEditFieldLabel           matlab.ui.control.Label
        RadiusEditField                matlab.ui.control.NumericEditField
        CreateCustomBoomerangButton    matlab.ui.control.Button
        RoffsetEditFieldLabel          matlab.ui.control.Label
        RoffsetEditField               matlab.ui.control.NumericEditField
        ChordLengthEditFieldLabel       matlab.ui.control.Label
        ChordLengthEditField            matlab.ui.control.NumericEditField
        UpdateBoomerangParametersButton  matlab.ui.control.Button
        CustomInertiasMassCheckBox     matlab.ui.control.CheckBox
        SaveCurrentBoomerangButton     matlab.ui.control.Button
        AirfoilParametersPanel         matlab.ui.container.Panel
        CreateCustomAirfoilButton      matlab.ui.control.Button
        CL0pEditFieldLabel             matlab.ui.control.Label
        CL0pEditField                  matlab.ui.control.NumericEditField
        CL0mEditFieldLabel             matlab.ui.control.Label
        CL0mEditField                  matlab.ui.control.NumericEditField
        CLapEditFieldLabel             matlab.ui.control.Label
        CLapEditField                  matlab.ui.control.NumericEditField
        CLamLabel                      matlab.ui.control.Label
        CLamEditField                  matlab.ui.control.NumericEditField
        CDamEditFieldLabel             matlab.ui.control.Label
        CDamEditField                  matlab.ui.control.NumericEditField
        CD0mEditFieldLabel             matlab.ui.control.Label
        CD0mEditField                  matlab.ui.control.NumericEditField
        CDapEditFieldLabel             matlab.ui.control.Label
        CDapEditField                  matlab.ui.control.NumericEditField
        CD0pEditFieldLabel             matlab.ui.control.Label
        CD0pEditField                  matlab.ui.control.NumericEditField
        UpdateAirfoilParametersButton  matlab.ui.control.Button
        SaveCurrentAirfoilButton       matlab.ui.control.Button
        BoomerangPlotPanel             matlab.ui.container.Panel
        UIAxes                         matlab.ui.control.UIAxes
        ThrowParametersPanel           matlab.ui.container.Panel
        ThrowVelocityEditFieldLabel    matlab.ui.control.Label
        ThrowVelocityEditField         matlab.ui.control.NumericEditField
        ThrowHeightEditFieldLabel      matlab.ui.control.Label
        ThrowHeightEditField           matlab.ui.control.NumericEditField
        AngularVelocityEditFieldLabel  matlab.ui.control.Label
        AngularVelocityEditField       matlab.ui.control.NumericEditField
        HorizontalAngleEditFieldLabel  matlab.ui.control.Label
        HorizontalAngleEditField       matlab.ui.control.NumericEditField
        LayoverAngleEditFieldLabel     matlab.ui.control.Label
        LayoverAngleEditField          matlab.ui.control.NumericEditField
        AOAEditFieldLabel              matlab.ui.control.Label
        AOAEditField                   matlab.ui.control.NumericEditField
        UpdateThrowParametersButton    matlab.ui.control.Button
        EnvironmentPanel               matlab.ui.container.Panel
        GravityEditFieldLabel          matlab.ui.control.Label
        GravityEditField               matlab.ui.control.NumericEditField
        WindSpeedEditFieldLabel        matlab.ui.control.Label
        WindSpeedEditField             matlab.ui.control.NumericEditField
        WindAngleEditFieldLabel        matlab.ui.control.Label
        WindAngleEditField             matlab.ui.control.NumericEditField
        TurbulenceDropDownLabel        matlab.ui.control.Label
        TurbulenceDropDown             matlab.ui.control.DropDown
        UpdateEnvironmentButton        matlab.ui.control.Button
        SimulatePanel                  matlab.ui.container.Panel
        TimeStepEditFieldLabel         matlab.ui.control.Label
        TimeStepEditField              matlab.ui.control.NumericEditField
        SimulationTimeEditFieldLabel   matlab.ui.control.Label
        SimulationTimeEditField        matlab.ui.control.NumericEditField
        GenerateAnimationCheckBox      matlab.ui.control.CheckBox
        SaveasGIFCheckBox              matlab.ui.control.CheckBox
        GIFNameEditFieldLabel          matlab.ui.control.Label
        GIFNameEditField               matlab.ui.control.EditField
        RunSimulationButton            matlab.ui.control.Button
        SavedSimulationsPanel          matlab.ui.container.Panel
        LoadSimButton                  matlab.ui.control.Button
        SavedSimsListBoxLabel          matlab.ui.control.Label
        SavedSimsListBox               matlab.ui.control.ListBox
        
        B %Boomerang Object
        S %Simulation Object
    end
    
    methods (Access = private)
        
        % Plot Boomerang
        function plotBoom(app,name)
            
            Tib = @(hor,lay,a) [cos(hor)*cos(a)-sin(hor)*sin(lay)*sin(a), -cos(hor)*sin(a)-sin(hor)*sin(lay)*cos(a), -sin(hor)*cos(lay);
                                cos(lay)*sin(a), cos(lay)*cos(a), -sin(lay);
                                sin(hor)*cos(a)+cos(hor)*sin(lay)*sin(a), -sin(hor)*sin(a)+cos(hor)*sin(lay)*cos(a), cos(hor)*cos(lay)];
            i1 = [1 0 0]'; i2 = [0 1 0]'; i3 = [0 0 1]';
            
            BoomCoords = Tib(app.HorizontalAngleEditField.Value*pi/180,app.LayoverAngleEditField.Value*pi/180,app.AOAEditField.Value*pi/180)*[app.B.X';app.B.Y';app.B.Z']+[0;0;app.ThrowHeightEditField.Value];
            VI = [cos(app.HorizontalAngleEditField.Value*pi/180) 0 sin(app.HorizontalAngleEditField.Value*pi/180)];
            
            cla(app.UIAxes);
            hold(app.UIAxes,'on');
            trisurf(app.B.Bound,BoomCoords(1,:)',BoomCoords(2,:)',BoomCoords(3,:)','FaceColor',[.5 0 .5],'EdgeColor',[.5 0 .5],'Parent', app.UIAxes);
            %shading(app.UIAxes,'interp');
            quiver3(0,0,app.ThrowHeightEditField.Value,i1(1),i1(2),i1(3),.2,'color','k','Parent', app.UIAxes);
            quiver3(0,0,app.ThrowHeightEditField.Value,i2(1),i2(2),i2(3),.2,'color','k','Parent', app.UIAxes);
            quiver3(0,0,app.ThrowHeightEditField.Value,i3(1),i3(2),i3(3),.2,'color','k','Parent', app.UIAxes);
            quiver3(0,0,app.ThrowHeightEditField.Value,VI(1),VI(2),VI(3),.2,'color','b','Parent', app.UIAxes);
            
            axis(app.UIAxes,'equal');
            
            if nargin == 2
                title(app.UIAxes, name);
            else
                title(app.UIAxes, 'Unnamed Boomerang');
            end
        end
    end
    
    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: LoadBoomButton
        function LoadBoomButtonPushed(app, event)
            Bstruc = load([cd '\SavedBooms\' app.SavedBoomsListBox.Value]);
            app.B = Bstruc.B;
            
            app.AreaEditField.Value = app.B.A;
            app.ThicknessEditField.Value = app.B.l;
            app.RadiusEditField.Value = app.B.R;
            app.DensityEditField.Value = app.B.rho;
            app.RoffsetEditField.Value = app.B.Roffset;
            app.ChordLengthEditField.Value = app.B.C;
            
            app.InertiaxEditField.Value = app.B.I(1,1);
            app.InertiayEditField.Value = app.B.I(2,2);
            app.InertiazEditField.Value = app.B.I(3,3);
            app.MassEditField.Value = app.B.m;
            
            app.CL0pEditField.Value = app.B.Foil.CL0p;
            app.CL0mEditField.Value = app.B.Foil.CL0m;
            app.CLapEditField.Value = app.B.Foil.CLap;
            app.CLamEditField.Value = app.B.Foil.CLam;
            app.CD0pEditField.Value = app.B.Foil.CD0p;
            app.CD0mEditField.Value = app.B.Foil.CD0m;
            app.CDapEditField.Value = app.B.Foil.CDap;
            app.CDamEditField.Value = app.B.Foil.CDam;
            
            plotBoom(app,app.SavedBoomsListBox.Value);            
        end

        % Button pushed function: LoadFoilButton
        function LoadFoilButtonPushed(app, event)
            Fstruc = load([cd '\SavedFoils\' app.SavedFoilsListBox.Value]);
            Foil = Fstruc.Foil;
            
            app.CL0pEditField.Value = Foil.CL0p;
            app.CL0mEditField.Value = Foil.CL0m;
            app.CLapEditField.Value = Foil.CLap;
            app.CLamEditField.Value = Foil.CLam;
            app.CD0pEditField.Value = Foil.CD0p;
            app.CD0mEditField.Value = Foil.CD0m;
            app.CDapEditField.Value = Foil.CDap;
            app.CDamEditField.Value = Foil.CDam;
            
            B.Foil = Foil;
        end

        % Button pushed function: LoadSimButton
        function LoadSimButtonPushed(app, event)
            
        end

        % Button pushed function: CreateCustomBoomerangButton
        function CreateCustomBoomerangButtonPushed(app, event)
            msgbox('This function has not yet been implemented');
        end

        % Button pushed function: CreateCustomAirfoilButton
        function CreateCustomAirfoilButtonPushed(app, event)
            msgbox('This function has not yet been implemented');
        end
        
        % Value changed function: CustomInertiasMassCheckBox
        function CustomInertiasMassCheckBoxValueChanged(app, event)
            
            if app.CustomInertiasMassCheckBox.Value
                app.InertiaxEditField.Editable = 1;
                app.InertiayEditField.Editable = 1;
                app.InertiazEditField.Editable = 1;
                app.MassEditField.Editable = 1;
            else
                app.InertiaxEditField.Editable = 0;
                app.InertiayEditField.Editable = 0;
                app.InertiazEditField.Editable = 0;
                app.MassEditField.Editable = 0;
            end
        end
        
        % Button pushed function: UpdateBoomerangParametersButton
        function UpdateBoomerangParametersButtonPushed(app, event)
            
            if app.CustomInertiasMassCheckBox.Value
                boomerang(app.B.Foil,app.RadiusEditField.Value,app.ChordLengthEditField.Value,app.AreaEditField.Value,app.RoffsetEditField.Value,app.ThicknessEditField.Value,app.MassEditField.Value,app.DensityEditField.Value,[app.InertiaxEditField.Value 0 0;0 app.InertiayEditField.Value 0;0 0 app.InertiazEditField.Value],3);
            else
                mass = app.ThicknessEditField.Value*app.AreaEditField.Value*app.DensityEditField.Value;
                I = [1/4*mass*app.RadiusEditField.Value^2,0,0;0,1/4*mass*app.RadiusEditField.Value^2,0;0,0,1/2*mass*app.RadiusEditField.Value^2];
                
                app.InertiaxEditField.Value = I(1,1);
                app.InertiayEditField.Value = I(2,2);
                app.InertiazEditField.Value = I(3,3);
                app.MassEditField.Value = mass;
                
                boomerang(app.B.Foil,app.RadiusEditField.Value,app.ChordLengthEditField.Value,app.AreaEditField.Value,app.RoffsetEditField.Value,app.ThicknessEditField.Value,mass,app.DensityEditField.Value,I,3);
            end
            
            plotBoom(app);
        end

        % Button pushed function: UpdateAirfoilParametersButton
        function UpdateAirfoilParametersButtonPushed(app, event)
            
            Foil.CL0p = app.CL0pEditField.Value;
            Foil.CL0m = app.CL0mEditField.Value;
            Foil.CLap = app.CLapEditField.Value;
            Foil.CLam = app.CLamEditField.Value;
            Foil.CD0p = app.CD0pEditField.Value;
            Foil.CD0m = app.CD0mEditField.Value;
            Foil.CDap = app.CDapEditField.Value;
            Foil.CDam = app.CDamEditField.Value;
            
            B.Foil = Foil;
            
        end

        % Button pushed function: UpdateThrowParametersButton
        function UpdateThrowParametersButtonPushed(app, event)
            
            plotBoom(app);
            
        end

        % Button pushed function: UpdateEnvironmentButton
        function UpdateEnvironmentButtonPushed(app, event)
            
            msgbox('Wind Function has not yet been implemented');
            
        end

        % Button pushed function: SaveCurrentBoomerangButton
        function SaveCurrentBoomerangButtonPushed(app, event)
            
            saveName = inputdlg('Enter Save Name:');
            B = app.B;
            save([cd '\SavedBooms\' saveName{1} '.mat'],'B');
            
            boomList = dir([cd '\SavedBooms']);
            app.SavedBoomsListBox.Items = {boomList(3:end).name};
            
            plotBoom(app,[saveName{1} '.mat']);
            
        end

        % Button pushed function: SaveCurrentAirfoilButton
        function SaveCurrentAirfoilButtonPushed(app, event)
            
            saveName = inputdlg('Enter Save Name:');
            Foil = app.B.Foil;
            save([cd '\SavedFoils\' saveName{1} '.mat'],'Foil');
            
            foilList = dir([cd '\SavedFoils']);
            app.SavedFoilsListBox.Items = {foilList(3:end).name};
                        
        end

        % Button pushed function: RunSimulationButton
        function RunSimulationButtonPushed(app, event)
            
            msg = msgbox('Simulation Running...');
            S = newBoomerangSimulation(app.ThrowVelocityEditField.Value,app.ThrowHeightEditField.Value,app.AngularVelocityEditField.Value*2*pi,app.HorizontalAngleEditField.Value*pi/180,app.LayoverAngleEditField.Value*pi/180,app.AOAEditField.Value*pi/180,app.TimeStepEditField.Value,app.SimulationTimeEditField.Value);
            [t,boomState,alpha,Mx,My,Mz,f] = boomerangTrajectory(S,app.B);
            close(msg);
            
            msgbox('Simulation Complete');
            %generatePlots(rocketState,alpha,Mx,My,Mz);
        end

    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1247 544];
            app.UIFigure.Name = 'MATLAB App';

            % Create SavedBoomerangsPanel
            app.SavedBoomerangsPanel = uipanel(app.UIFigure);
            app.SavedBoomerangsPanel.TitlePosition = 'centertop';
            app.SavedBoomerangsPanel.Title = 'Saved Boomerangs';
            app.SavedBoomerangsPanel.Position = [1 385 260 160];

            % Create SavedBoomsListBoxLabel
            app.SavedBoomsListBoxLabel = uilabel(app.SavedBoomerangsPanel);
            app.SavedBoomsListBoxLabel.HorizontalAlignment = 'right';
            app.SavedBoomsListBoxLabel.Position = [16 104 80 22];
            app.SavedBoomsListBoxLabel.Text = 'Saved Booms';

            % Create SavedBoomsListBox
            app.SavedBoomsListBox = uilistbox(app.SavedBoomerangsPanel);
            boomList = dir([cd '\SavedBooms']);
            app.SavedBoomsListBox.Items = {boomList(3:end).name};
            app.SavedBoomsListBox.Position = [111 54 100 74];

            % Create LoadBoomButton
            app.LoadBoomButton = uibutton(app.SavedBoomerangsPanel, 'push');
            app.LoadBoomButton.ButtonPushedFcn = createCallbackFcn(app, @LoadBoomButtonPushed, true);
            app.LoadBoomButton.Position = [80 15 100 22];
            app.LoadBoomButton.Text = 'Load Boom';

            % Create SavedAirfoilsPanel
            app.SavedAirfoilsPanel = uipanel(app.UIFigure);
            app.SavedAirfoilsPanel.TitlePosition = 'centertop';
            app.SavedAirfoilsPanel.Title = 'Saved Airfoils';
            app.SavedAirfoilsPanel.Position = [260 385 260 160];

            % Create SavedFoilsListBoxLabel
            app.SavedFoilsListBoxLabel = uilabel(app.SavedAirfoilsPanel);
            app.SavedFoilsListBoxLabel.HorizontalAlignment = 'right';
            app.SavedFoilsListBoxLabel.Position = [44 104 68 22];
            app.SavedFoilsListBoxLabel.Text = 'Saved Foils';

            % Create SavedFoilsListBox
            app.SavedFoilsListBox = uilistbox(app.SavedAirfoilsPanel);
            foilList = dir([cd '\SavedFoils']);
            app.SavedFoilsListBox.Items = {foilList(3:end).name};
            app.SavedFoilsListBox.Position = [127 54 100 74];

            % Create LoadFoilButton
            app.LoadFoilButton = uibutton(app.SavedAirfoilsPanel, 'push');
            app.LoadFoilButton.ButtonPushedFcn = createCallbackFcn(app, @LoadFoilButtonPushed, true);
            app.LoadFoilButton.Position = [80 15 100 22];
            app.LoadFoilButton.Text = 'Load Foil';

            % Create BoomerangParametersPanel
            app.BoomerangParametersPanel = uipanel(app.UIFigure);
            app.BoomerangParametersPanel.TitlePosition = 'centertop';
            app.BoomerangParametersPanel.Title = 'Boomerang Parameters';
            app.BoomerangParametersPanel.Position = [1 3 260 382];

            % Create AreaEditFieldLabel
            app.AreaEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.AreaEditFieldLabel.HorizontalAlignment = 'right';
            app.AreaEditFieldLabel.Position = [35 289 31 22];
            app.AreaEditFieldLabel.Text = 'Area';

            % Create AreaEditField
            app.AreaEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.AreaEditField.Position = [81 289 31 22];

            % Create ThicknessEditFieldLabel
            app.ThicknessEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.ThicknessEditFieldLabel.HorizontalAlignment = 'right';
            app.ThicknessEditFieldLabel.Position = [133 289 64 22];
            app.ThicknessEditFieldLabel.Text = 'Thickness';

            % Create ThicknessEditField
            app.ThicknessEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.ThicknessEditField.Position = [212 289 31 22];

            % Create DensityEditFieldLabel
            app.DensityEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.DensityEditFieldLabel.HorizontalAlignment = 'right';
            app.DensityEditFieldLabel.Position = [151 248 46 22];
            app.DensityEditFieldLabel.Text = 'Density';

            % Create DensityEditField
            app.DensityEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.DensityEditField.Position = [212 248 31 22];

            % Create MassEditFieldLabel
            app.MassEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.MassEditFieldLabel.HorizontalAlignment = 'right';
            app.MassEditFieldLabel.Position = [163 78 34 22];
            app.MassEditFieldLabel.Text = 'Mass';

            % Create MassEditField
            app.MassEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.MassEditField.Position = [212 78 31 22];

            % Create InertiazEditFieldLabel
            app.InertiazEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.InertiazEditFieldLabel.HorizontalAlignment = 'right';
            app.InertiazEditFieldLabel.Position = [21 78 45 22];
            app.InertiazEditFieldLabel.Text = 'Inertiaz';

            % Create InertiazEditField
            app.InertiazEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.InertiazEditField.Position = [81 78 31 22];

            % Create InertiaxEditFieldLabel
            app.InertiaxEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.InertiaxEditFieldLabel.HorizontalAlignment = 'right';
            app.InertiaxEditFieldLabel.Position = [21 118 45 22];
            app.InertiaxEditFieldLabel.Text = 'Inertiax';

            % Create InertiaxEditField
            app.InertiaxEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.InertiaxEditField.Position = [81 118 31 22];

            % Create InertiayEditFieldLabel
            app.InertiayEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.InertiayEditFieldLabel.HorizontalAlignment = 'right';
            app.InertiayEditFieldLabel.Position = [152 118 45 22];
            app.InertiayEditFieldLabel.Text = 'Inertiay';

            % Create InertiayEditField
            app.InertiayEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.InertiayEditField.Position = [212 118 31 22];

            % Create RadiusEditFieldLabel
            app.RadiusEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.RadiusEditFieldLabel.HorizontalAlignment = 'right';
            app.RadiusEditFieldLabel.Position = [24 248 43 22];
            app.RadiusEditFieldLabel.Text = 'Radius';

            % Create RadiusEditField
            app.RadiusEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.RadiusEditField.Position = [82 248 31 22];

            % Create CreateCustomBoomerangButton
            app.CreateCustomBoomerangButton = uibutton(app.BoomerangParametersPanel, 'push');
            app.CreateCustomBoomerangButton.ButtonPushedFcn = createCallbackFcn(app, @CreateCustomBoomerangButtonPushed, true);
            app.CreateCustomBoomerangButton.Position = [51 333 162 22];
            app.CreateCustomBoomerangButton.Text = 'Create Custom Boomerang';

            % Create RoffsetEditFieldLabel
            app.RoffsetEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.RoffsetEditFieldLabel.HorizontalAlignment = 'right';
            app.RoffsetEditFieldLabel.Position = [24 206 43 22];
            app.RoffsetEditFieldLabel.Text = 'Roffset';

            % Create RoffsetEditField
            app.RoffsetEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.RoffsetEditField.Position = [82 206 31 22];

            % Create ChordLengthEditFieldLabel
            app.ChordLengthEditFieldLabel = uilabel(app.BoomerangParametersPanel);
            app.ChordLengthEditFieldLabel.HorizontalAlignment = 'right';
            app.ChordLengthEditFieldLabel.Position = [125 205 72 22];
            app.ChordLengthEditFieldLabel.Text = 'Chord Length';

            % Create ChordLengthEditField
            app.ChordLengthEditField = uieditfield(app.BoomerangParametersPanel, 'numeric');
            app.ChordLengthEditField.Position = [212 205 31 22];

            % Create UpdateBoomerangParametersButton
            app.UpdateBoomerangParametersButton = uibutton(app.BoomerangParametersPanel, 'push');
            app.UpdateBoomerangParametersButton.ButtonPushedFcn = createCallbackFcn(app, @UpdateBoomerangParametersButtonPushed, true);
            app.UpdateBoomerangParametersButton.Position = [40 46 185 22];
            app.UpdateBoomerangParametersButton.Text = 'Update Boomerang Parameters';

            % Create CustomInertiasMassCheckBox
            app.CustomInertiasMassCheckBox = uicheckbox(app.BoomerangParametersPanel);
            app.CustomInertiasMassCheckBox.ValueChangedFcn = createCallbackFcn(app, @CustomInertiasMassCheckBoxValueChanged, true);
            app.CustomInertiasMassCheckBox.Text = 'Custom Inertias/Mass';
            app.CustomInertiasMassCheckBox.Position = [74 161 138 22];
            app.CustomInertiasMassCheckBox.Value = 1;

            % Create SaveCurrentBoomerangButton
            app.SaveCurrentBoomerangButton = uibutton(app.BoomerangParametersPanel, 'push');
            app.SaveCurrentBoomerangButton.ButtonPushedFcn = createCallbackFcn(app, @SaveCurrentBoomerangButtonPushed, true);
            app.SaveCurrentBoomerangButton.Position = [57 12 152 22];
            app.SaveCurrentBoomerangButton.Text = 'Save Current Boomerang';

            % Create AirfoilParametersPanel
            app.AirfoilParametersPanel = uipanel(app.UIFigure);
            app.AirfoilParametersPanel.TitlePosition = 'centertop';
            app.AirfoilParametersPanel.Title = 'Airfoil Parameters';
            app.AirfoilParametersPanel.Position = [260 3 260 383];

            % Create CreateCustomAirfoilButton
            app.CreateCustomAirfoilButton = uibutton(app.AirfoilParametersPanel, 'push');
            app.CreateCustomAirfoilButton.ButtonPushedFcn = createCallbackFcn(app, @CreateCustomAirfoilButtonPushed, true);
            app.CreateCustomAirfoilButton.Position = [65 333 130 22];
            app.CreateCustomAirfoilButton.Text = 'Create Custom Airfoil';

            % Create CL0pEditFieldLabel
            app.CL0pEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CL0pEditFieldLabel.HorizontalAlignment = 'right';
            app.CL0pEditFieldLabel.Position = [26 290 34 22];
            app.CL0pEditFieldLabel.Text = 'CL0p';

            % Create CL0pEditField
            app.CL0pEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CL0pEditField.Position = [75 290 31 22];

            % Create CL0mEditFieldLabel
            app.CL0mEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CL0mEditFieldLabel.HorizontalAlignment = 'right';
            app.CL0mEditFieldLabel.Position = [127 290 64 22];
            app.CL0mEditFieldLabel.Text = 'CL0m';

            % Create CL0mEditField
            app.CL0mEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CL0mEditField.Position = [206 290 31 22];

            % Create CLapEditFieldLabel
            app.CLapEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CLapEditFieldLabel.HorizontalAlignment = 'right';
            app.CLapEditFieldLabel.Position = [26 249 34 22];
            app.CLapEditFieldLabel.Text = 'CLap';

            % Create CLapEditField
            app.CLapEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CLapEditField.Position = [75 249 31 22];

            % Create CLamLabel
            app.CLamLabel = uilabel(app.AirfoilParametersPanel);
            app.CLamLabel.HorizontalAlignment = 'right';
            app.CLamLabel.Position = [153 249 38 22];
            app.CLamLabel.Text = 'CLam';

            % Create CLamEditField
            app.CLamEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CLamEditField.Position = [206 249 31 22];

            % Create CDamEditFieldLabel
            app.CDamEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CDamEditFieldLabel.HorizontalAlignment = 'right';
            app.CDamEditFieldLabel.Position = [151 161 40 22];
            app.CDamEditFieldLabel.Text = 'CDam';

            % Create CDamEditField
            app.CDamEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CDamEditField.Position = [206 161 31 22];

            % Create CD0mEditFieldLabel
            app.CD0mEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CD0mEditFieldLabel.HorizontalAlignment = 'right';
            app.CD0mEditFieldLabel.Position = [151 206 40 22];
            app.CD0mEditFieldLabel.Text = 'CD0m';

            % Create CD0mEditField
            app.CD0mEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CD0mEditField.Position = [206 206 31 22];

            % Create CDapEditFieldLabel
            app.CDapEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CDapEditFieldLabel.HorizontalAlignment = 'right';
            app.CDapEditFieldLabel.Position = [24 161 36 22];
            app.CDapEditFieldLabel.Text = 'CDap';

            % Create CDapEditField
            app.CDapEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CDapEditField.Position = [75 161 31 22];

            % Create CD0pEditFieldLabel
            app.CD0pEditFieldLabel = uilabel(app.AirfoilParametersPanel);
            app.CD0pEditFieldLabel.HorizontalAlignment = 'right';
            app.CD0pEditFieldLabel.Position = [24 206 36 22];
            app.CD0pEditFieldLabel.Text = 'CD0p';

            % Create CD0pEditField
            app.CD0pEditField = uieditfield(app.AirfoilParametersPanel, 'numeric');
            app.CD0pEditField.Position = [75 206 31 22];

            % Create UpdateAirfoilParametersButton
            app.UpdateAirfoilParametersButton = uibutton(app.AirfoilParametersPanel, 'push');
            app.UpdateAirfoilParametersButton.ButtonPushedFcn = createCallbackFcn(app, @UpdateAirfoilParametersButtonPushed, true);
            app.UpdateAirfoilParametersButton.Position = [53 119 153 22];
            app.UpdateAirfoilParametersButton.Text = 'Update Airfoil Parameters';

            % Create SaveCurrentAirfoilButton
            app.SaveCurrentAirfoilButton = uibutton(app.AirfoilParametersPanel, 'push');
            app.SaveCurrentAirfoilButton.ButtonPushedFcn = createCallbackFcn(app, @SaveCurrentAirfoilButtonPushed, true);
            app.SaveCurrentAirfoilButton.Position = [70 85 120 22];
            app.SaveCurrentAirfoilButton.Text = 'Save Current Airfoil';

            % Create BoomerangPlotPanel
            app.BoomerangPlotPanel = uipanel(app.UIFigure);
            app.BoomerangPlotPanel.TitlePosition = 'centertop';
            app.BoomerangPlotPanel.Title = 'Boomerang Plot';
            app.BoomerangPlotPanel.Position = [519 150 470 395];

            % Create UIAxes
            app.UIAxes = uiaxes(app.BoomerangPlotPanel);
            title(app.UIAxes, 'Title')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.PlotBoxAspectRatio = [1.41573033707865 1 1];
            app.UIAxes.Position = [22 37 425 321];

            % Create ThrowParametersPanel
            app.ThrowParametersPanel = uipanel(app.UIFigure);
            app.ThrowParametersPanel.TitlePosition = 'centertop';
            app.ThrowParametersPanel.Title = 'Throw Parameters';
            app.ThrowParametersPanel.Position = [519 3 470 148];

            % Create ThrowVelocityEditFieldLabel
            app.ThrowVelocityEditFieldLabel = uilabel(app.ThrowParametersPanel);
            app.ThrowVelocityEditFieldLabel.HorizontalAlignment = 'right';
            app.ThrowVelocityEditFieldLabel.Position = [157 87 84 22];
            app.ThrowVelocityEditFieldLabel.Text = 'Throw Velocity';

            % Create ThrowVelocityEditField
            app.ThrowVelocityEditField = uieditfield(app.ThrowParametersPanel, 'numeric');
            app.ThrowVelocityEditField.Position = [248 87 43 22];
            app.ThrowVelocityEditField.Value = 25;

            % Create ThrowHeightEditFieldLabel
            app.ThrowHeightEditFieldLabel = uilabel(app.ThrowParametersPanel);
            app.ThrowHeightEditFieldLabel.HorizontalAlignment = 'right';
            app.ThrowHeightEditFieldLabel.Position = [315 87 77 22];
            app.ThrowHeightEditFieldLabel.Text = 'Throw Height';

            % Create ThrowHeightEditField
            app.ThrowHeightEditField = uieditfield(app.ThrowParametersPanel, 'numeric');
            app.ThrowHeightEditField.Position = [404 87 43 22];
            app.ThrowHeightEditField.Value = 1.5;

            % Create AngularVelocityEditFieldLabel
            app.AngularVelocityEditFieldLabel = uilabel(app.ThrowParametersPanel);
            app.AngularVelocityEditFieldLabel.HorizontalAlignment = 'right';
            app.AngularVelocityEditFieldLabel.Position = [6 87 92 22];
            app.AngularVelocityEditFieldLabel.Text = 'Angular Velocity';

            % Create AngularVelocityEditField
            app.AngularVelocityEditField = uieditfield(app.ThrowParametersPanel, 'numeric');
            app.AngularVelocityEditField.Position = [105 87 43 22];
            app.AngularVelocityEditField.Value = 15;

            % Create HorizontalAngleEditFieldLabel
            app.HorizontalAngleEditFieldLabel = uilabel(app.ThrowParametersPanel);
            app.HorizontalAngleEditFieldLabel.HorizontalAlignment = 'right';
            app.HorizontalAngleEditFieldLabel.Position = [4 46 94 22];
            app.HorizontalAngleEditFieldLabel.Text = 'Horizontal Angle';

            % Create HorizontalAngleEditField
            app.HorizontalAngleEditField = uieditfield(app.ThrowParametersPanel, 'numeric');
            app.HorizontalAngleEditField.Position = [105 46 43 22];
            app.HorizontalAngleEditField.Value = 10;

            % Create LayoverAngleEditFieldLabel
            app.LayoverAngleEditFieldLabel = uilabel(app.ThrowParametersPanel);
            app.LayoverAngleEditFieldLabel.HorizontalAlignment = 'right';
            app.LayoverAngleEditFieldLabel.Position = [157 46 82 22];
            app.LayoverAngleEditFieldLabel.Text = 'Layover Angle';

            % Create LayoverAngleEditField
            app.LayoverAngleEditField = uieditfield(app.ThrowParametersPanel, 'numeric');
            app.LayoverAngleEditField.Position = [249 46 43 22];
            app.LayoverAngleEditField.Value = 10;

            % Create AOAEditFieldLabel
            app.AOAEditFieldLabel = uilabel(app.ThrowParametersPanel);
            app.AOAEditFieldLabel.HorizontalAlignment = 'right';
            app.AOAEditFieldLabel.Position = [358 46 31 22];
            app.AOAEditFieldLabel.Text = 'AOA';

            % Create AOAEditField
            app.AOAEditField = uieditfield(app.ThrowParametersPanel, 'numeric');
            app.AOAEditField.Position = [404 46 43 22];

            % Create UpdateThrowParametersButton
            app.UpdateThrowParametersButton = uibutton(app.ThrowParametersPanel, 'push');
            app.UpdateThrowParametersButton.ButtonPushedFcn = createCallbackFcn(app, @UpdateThrowParametersButtonPushed, true);
            app.UpdateThrowParametersButton.Position = [157 12 156 22];
            app.UpdateThrowParametersButton.Text = 'Update Throw Parameters';

            % Create EnvironmentPanel
            app.EnvironmentPanel = uipanel(app.UIFigure);
            app.EnvironmentPanel.TitlePosition = 'centertop';
            app.EnvironmentPanel.Title = 'Environment';
            app.EnvironmentPanel.Position = [988 195 260 191];

            % Create GravityEditFieldLabel
            app.GravityEditFieldLabel = uilabel(app.EnvironmentPanel);
            app.GravityEditFieldLabel.HorizontalAlignment = 'right';
            app.GravityEditFieldLabel.Position = [84 141 44 22];
            app.GravityEditFieldLabel.Text = 'Gravity';

            % Create GravityEditField
            app.GravityEditField = uieditfield(app.EnvironmentPanel, 'numeric');
            app.GravityEditField.Position = [135 141 40 22];
            app.GravityEditField.Value = 9.81;

            % Create WindSpeedEditFieldLabel
            app.WindSpeedEditFieldLabel = uilabel(app.EnvironmentPanel);
            app.WindSpeedEditFieldLabel.HorizontalAlignment = 'right';
            app.WindSpeedEditFieldLabel.Position = [1 98 71 22];
            app.WindSpeedEditFieldLabel.Text = 'Wind Speed';

            % Create WindSpeedEditField
            app.WindSpeedEditField = uieditfield(app.EnvironmentPanel, 'numeric');
            app.WindSpeedEditField.Position = [80 98 40 22];

            % Create WindAngleEditFieldLabel
            app.WindAngleEditFieldLabel = uilabel(app.EnvironmentPanel);
            app.WindAngleEditFieldLabel.HorizontalAlignment = 'right';
            app.WindAngleEditFieldLabel.Position = [136 98 67 22];
            app.WindAngleEditFieldLabel.Text = 'Wind Angle';

            % Create WindAngleEditField
            app.WindAngleEditField = uieditfield(app.EnvironmentPanel, 'numeric');
            app.WindAngleEditField.Position = [210 98 40 22];

            % Create TurbulenceDropDownLabel
            app.TurbulenceDropDownLabel = uilabel(app.EnvironmentPanel);
            app.TurbulenceDropDownLabel.HorizontalAlignment = 'right';
            app.TurbulenceDropDownLabel.Position = [58 57 65 22];
            app.TurbulenceDropDownLabel.Text = 'Turbulence';

            % Create TurbulenceDropDown
            app.TurbulenceDropDown = uidropdown(app.EnvironmentPanel);
            app.TurbulenceDropDown.Items = {'None', 'Light', 'Moderate', 'Severe'};
            app.TurbulenceDropDown.Position = [133 57 67 22];
            app.TurbulenceDropDown.Value = 'None';

            % Create UpdateEnvironmentButton
            app.UpdateEnvironmentButton = uibutton(app.EnvironmentPanel, 'push');
            app.UpdateEnvironmentButton.ButtonPushedFcn = createCallbackFcn(app, @UpdateEnvironmentButtonPushed, true);
            app.UpdateEnvironmentButton.Position = [68 13 125 22];
            app.UpdateEnvironmentButton.Text = 'Update Environment';

            % Create SimulatePanel
            app.SimulatePanel = uipanel(app.UIFigure);
            app.SimulatePanel.TitlePosition = 'centertop';
            app.SimulatePanel.Title = 'Simulate';
            app.SimulatePanel.Position = [988 3 260 193];

            % Create TimeStepEditFieldLabel
            app.TimeStepEditFieldLabel = uilabel(app.SimulatePanel);
            app.TimeStepEditFieldLabel.HorizontalAlignment = 'right';
            app.TimeStepEditFieldLabel.Position = [8 139 60 22];
            app.TimeStepEditFieldLabel.Text = 'Time Step';

            % Create TimeStepEditField
            app.TimeStepEditField = uieditfield(app.SimulatePanel, 'numeric');
            app.TimeStepEditField.Position = [76 139 38 22];
            app.TimeStepEditField.Value = 1/10000;

            % Create SimulationTimeEditFieldLabel
            app.SimulationTimeEditFieldLabel = uilabel(app.SimulatePanel);
            app.SimulationTimeEditFieldLabel.HorizontalAlignment = 'right';
            app.SimulationTimeEditFieldLabel.Position = [119 139 91 22];
            app.SimulationTimeEditFieldLabel.Text = 'Simulation Time';

            % Create SimulationTimeEditField
            app.SimulationTimeEditField = uieditfield(app.SimulatePanel, 'numeric');
            app.SimulationTimeEditField.Position = [220 139 36 22];
            app.SimulationTimeEditField.Value = 10;

            % Create GenerateAnimationCheckBox
            app.GenerateAnimationCheckBox = uicheckbox(app.SimulatePanel);
            app.GenerateAnimationCheckBox.Text = 'Generate Animation';
            app.GenerateAnimationCheckBox.Position = [8 108 129 22];

            % Create SaveasGIFCheckBox
            app.SaveasGIFCheckBox = uicheckbox(app.SimulatePanel);
            app.SaveasGIFCheckBox.Text = 'Save as GIF';
            app.SaveasGIFCheckBox.Position = [148 108 89 22];

            % Create GIFNameEditFieldLabel
            app.GIFNameEditFieldLabel = uilabel(app.SimulatePanel);
            app.GIFNameEditFieldLabel.HorizontalAlignment = 'right';
            app.GIFNameEditFieldLabel.Position = [41 78 61 22];
            app.GIFNameEditFieldLabel.Text = 'GIF Name';

            % Create GIFNameEditField
            app.GIFNameEditField = uieditfield(app.SimulatePanel, 'text');
            app.GIFNameEditField.Position = [117 78 100 22];

            % Create RunSimulationButton
            app.RunSimulationButton = uibutton(app.SimulatePanel, 'push');
            app.RunSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @RunSimulationButtonPushed, true);
            app.RunSimulationButton.Position = [79 46 100 22];
            app.RunSimulationButton.Text = 'Run Simulation';

            % Create SavedSimulationsPanel
            app.SavedSimulationsPanel = uipanel(app.UIFigure);
            app.SavedSimulationsPanel.TitlePosition = 'centertop';
            app.SavedSimulationsPanel.Title = 'Saved Simulations';
            app.SavedSimulationsPanel.Position = [988 385 260 160];

            % Create LoadSimButton
            app.LoadSimButton = uibutton(app.SavedSimulationsPanel, 'push');
            app.LoadSimButton.ButtonPushedFcn = createCallbackFcn(app, @LoadSimButtonPushed, true);
            app.LoadSimButton.Position = [74 15 100 22];
            app.LoadSimButton.Text = 'Load Sim';

            % Create SavedSimsListBoxLabel
            app.SavedSimsListBoxLabel = uilabel(app.SavedSimulationsPanel);
            app.SavedSimsListBoxLabel.HorizontalAlignment = 'right';
            app.SavedSimsListBoxLabel.Position = [36 104 70 22];
            app.SavedSimsListBoxLabel.Text = 'Saved Sims';

            % Create SavedSimsListBox
            app.SavedSimsListBox = uilistbox(app.SavedSimulationsPanel);
            simList = dir([cd '\SavedSims']);
            app.SavedSimsListBox.Items = {simList(3:end).name};
            app.SavedSimsListBox.Position = [121 54 100 74];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = BoomGUI

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end