classdef DL5MotionApp < handle
    properties (SetAccess = private)
        Robot
        Figure
    end

    properties (Access = private)
        MechanismAxes
        ThetaSlider
        PhiSlider
        XSlider
        YSlider
        ThetaValueLabel
        PhiValueLabel
        XValueLabel
        YValueLabel
        StatusLamp
        StatusLabel
        IsUpdatingUI = false
    end

    methods
        function app = DL5MotionApp(robot, options)
            if nargin < 1 || isempty(robot)
                robot = duallink5.DL5();
            end
            if nargin < 2
                options = struct();
            end
            visible = validateInputs(robot, options);
            app.Robot = robot;

            try
                app.createComponents();
                app.refresh();
                app.Figure.UserData = app;
                app.Figure.Visible = char(visible);
            catch exception
                app.delete();
                rethrow(exception)
            end
        end

        function delete(app)
            if ~isempty(app.Figure) && isgraphics(app.Figure, 'figure')
                app.Figure.UserData = [];
                delete(app.Figure);
            end
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.Figure = uifigure( ...
                'Name', 'DualLink5 Motion', ...
                'Color', [0.96, 0.96, 0.96], ...
                'Position', [100, 100, 1180, 720], ...
                'Visible', 'off');

            mainGrid = uigridlayout(app.Figure, [1, 2]);
            mainGrid.ColumnWidth = {'1x', 360};
            mainGrid.Padding = [12, 12, 12, 12];
            mainGrid.ColumnSpacing = 12;

            mechanismPanel = uipanel(mainGrid, ...
                'Title', 'Complete mechanism configuration');
            mechanismPanel.Layout.Row = 1;
            mechanismPanel.Layout.Column = 1;
            mechanismGrid = uigridlayout(mechanismPanel, [1, 1]);
            mechanismGrid.Padding = [6, 6, 6, 6];
            app.MechanismAxes = uiaxes(mechanismGrid);
            app.MechanismAxes.Layout.Row = 1;
            app.MechanismAxes.Layout.Column = 1;

            controlsPanel = uipanel(mainGrid, 'Title', 'Motion control');
            controlsPanel.Layout.Row = 1;
            controlsPanel.Layout.Column = 2;
            controls = uigridlayout(controlsPanel, [12, 2]);
            controls.ColumnWidth = {'1x', 90};
            controls.RowHeight = {30, 24, 44, 24, 44, ...
                30, 24, 44, 24, 44, 30, '1x'};
            controls.Padding = [12, 10, 12, 12];

            addSectionTitle(controls, 'Joint-space control', 1);
            [app.ThetaValueLabel, app.ThetaSlider] = ...
                app.addSliderControl(controls, 2, 3, ...
                    '\theta [deg]', ...
                    rad2deg(app.Robot.Geometry.analysis.thetaRange), ...
                    app.Robot.QDegrees(1));
            [app.PhiValueLabel, app.PhiSlider] = ...
                app.addSliderControl(controls, 4, 5, ...
                    '\phi [deg]', ...
                    rad2deg(app.Robot.Geometry.analysis.phiRange), ...
                    app.Robot.QDegrees(2));

            addSectionTitle(controls, 'Point G / B_{k+1}', 6);
            [xLimits, yLimits] = app.pointSliderLimits();
            [app.XValueLabel, app.XSlider] = ...
                app.addSliderControl(controls, 7, 8, ...
                    'x_G [mm]', xLimits, ...
                    app.Robot.PointGMillimetres(1));
            [app.YValueLabel, app.YSlider] = ...
                app.addSliderControl(controls, 9, 10, ...
                    'y_G [mm]', yLimits, ...
                    app.Robot.PointGMillimetres(2));

            addSectionTitle(controls, 'Configuration status', 11);
            statusPanel = uipanel(controls);
            statusPanel.Layout.Row = 12;
            statusPanel.Layout.Column = [1, 2];
            statusGrid = uigridlayout(statusPanel, [1, 2]);
            statusGrid.ColumnWidth = {34, '1x'};
            statusGrid.Padding = [10, 10, 10, 10];
            app.StatusLamp = uilamp(statusGrid);
            app.StatusLamp.Layout.Row = 1;
            app.StatusLamp.Layout.Column = 1;
            app.StatusLabel = uilabel(statusGrid, ...
                'FontWeight', 'bold', 'WordWrap', 'on');
            app.StatusLabel.Layout.Row = 1;
            app.StatusLabel.Layout.Column = 2;

            app.ThetaSlider.ValueChangingFcn = ...
                @(~, event)app.onAngleRequest( ...
                    event.Value, app.PhiSlider.Value);
            app.ThetaSlider.ValueChangedFcn = ...
                @(~, event)app.onAngleRequest( ...
                    event.Value, app.PhiSlider.Value);
            app.PhiSlider.ValueChangingFcn = ...
                @(~, event)app.onAngleRequest( ...
                    app.ThetaSlider.Value, event.Value);
            app.PhiSlider.ValueChangedFcn = ...
                @(~, event)app.onAngleRequest( ...
                    app.ThetaSlider.Value, event.Value);
            app.XSlider.ValueChangingFcn = ...
                @(~, event)app.onPointRequest( ...
                    event.Value, app.YSlider.Value);
            app.XSlider.ValueChangedFcn = ...
                @(~, event)app.onPointRequest( ...
                    event.Value, app.YSlider.Value);
            app.YSlider.ValueChangingFcn = ...
                @(~, event)app.onPointRequest( ...
                    app.XSlider.Value, event.Value);
            app.YSlider.ValueChangedFcn = ...
                @(~, event)app.onPointRequest( ...
                    app.XSlider.Value, event.Value);
        end

        function [valueLabel, slider] = addSliderControl( ...
                ~, parent, labelRow, sliderRow, name, limits, value)
            nameLabel = uilabel(parent, 'Text', name, ...
                'Interpreter', 'tex');
            nameLabel.Layout.Row = labelRow;
            nameLabel.Layout.Column = 1;
            valueLabel = uilabel(parent, ...
                'HorizontalAlignment', 'right');
            valueLabel.Layout.Row = labelRow;
            valueLabel.Layout.Column = 2;
            slider = uislider(parent, ...
                'Limits', double(limits), 'Value', double(value));
            slider.Layout.Row = sliderRow;
            slider.Layout.Column = [1, 2];
        end

        function [xLimits, yLimits] = pointSliderLimits(app)
            links = app.Robot.Geometry.links;
            centerReach = links.link1 + links.link4 / 2;
            xLimits = 1e3 * [ ...
                -2 * centerReach - links.link5, ...
                2 * centerReach - links.link5];
            yLimits = 1e3 * [0, 2 * centerReach];
            point = app.Robot.PointGMillimetres;
            xLimits = includeValue(xLimits, point(1));
            yLimits = includeValue(yLimits, point(2));
        end

        function onAngleRequest(app, theta, phi)
            if app.IsUpdatingUI
                return
            end
            result = app.Robot.trySetJointAnglesDegrees(theta, phi);
            app.refresh(result);
        end

        function onPointRequest(app, x, y)
            if app.IsUpdatingUI
                return
            end
            result = app.Robot.trySetPointGMillimetres(x, y);
            app.refresh(result);
        end

        function refresh(app, varargin)
            app.IsUpdatingUI = true;
            cleanup = onCleanup(@()app.finishRefresh());

            qDegrees = app.Robot.QDegrees;
            pointMillimetres = app.Robot.PointGMillimetres;
            app.ThetaSlider.Value = qDegrees(1);
            app.PhiSlider.Value = qDegrees(2);
            app.XSlider.Value = pointMillimetres(1);
            app.YSlider.Value = pointMillimetres(2);
            app.ThetaValueLabel.Text = sprintf('%.2f', qDegrees(1));
            app.PhiValueLabel.Text = sprintf('%.2f', qDegrees(2));
            app.XValueLabel.Text = sprintf('%.2f', pointMillimetres(1));
            app.YValueLabel.Text = sprintf('%.2f', pointMillimetres(2));

            cla(app.MechanismAxes);
            app.Robot.plot(app.MechanismAxes, ...
                struct('showLabels', true));
            hold(app.MechanismAxes, 'on');
            plot(app.MechanismAxes, pointMillimetres(1), ...
                pointMillimetres(2), 'kp', ...
                'MarkerSize', 11, 'MarkerFaceColor', 'y');
            hold(app.MechanismAxes, 'off');
            title(app.MechanismAxes, sprintf( ...
                '\\theta = %.2f^\\circ, \\phi = %.2f^\\circ', ...
                qDegrees(1), qDegrees(2)), 'Interpreter', 'tex');

            if app.Robot.IsValid
                app.StatusLamp.Color = [0.20, 0.70, 0.25];
                app.StatusLabel.Text = 'VALID';
                app.StatusLabel.FontColor = [0.10, 0.45, 0.15];
            else
                app.StatusLamp.Color = [0.85, 0.20, 0.20];
                app.StatusLabel.Text = sprintf( ...
                    'INVALID REQUEST\n%s\nLast valid pose retained', ...
                    app.Robot.StatusCode);
                app.StatusLabel.FontColor = [0.70, 0.10, 0.10];
            end
            drawnow limitrate
            clear cleanup
        end

        function finishRefresh(app)
            app.IsUpdatingUI = false;
        end
    end
end

function visible = validateInputs(robot, options)
if ~isa(robot, 'duallink5.DL5') || ~isscalar(robot) || ~isvalid(robot)
    error('duallink5:ui:InvalidRobot', ...
        'robot must be a live scalar duallink5.DL5 object.');
end
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), {'visible'}))
    invalidOptions();
end
try
    if isfield(options, 'visible')
        visible = string(options.visible);
    else
        visible = "on";
    end
catch
    invalidOptions();
end
if ~isscalar(visible) || ismissing(visible) || ...
        ~ismember(visible, ["on", "off"])
    invalidOptions();
end
end

function addSectionTitle(parent, textValue, row)
label = uilabel(parent, 'Text', textValue, ...
    'Interpreter', 'tex', 'FontWeight', 'bold', 'FontSize', 14);
label.Layout.Row = row;
label.Layout.Column = [1, 2];
end

function limits = includeValue(limits, value)
margin = max(1, 0.01 * diff(limits));
limits(1) = min(limits(1), value - margin);
limits(2) = max(limits(2), value + margin);
end

function invalidOptions()
error('duallink5:ui:InvalidOptions', ...
    'options.visible must be scalar "on" or "off".');
end
