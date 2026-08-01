function run_thesis_menu()
% RUN_THESIS_MENU
% ---------------------------------------------------------------
% Main GUI launcher for the thesis simulations:
%
%   "Autonomous UAV Mission Planning Under Threat Using
%    Model Predictive Control with Proportional-Navigation Pursuers"
%
% GUI-based launcher providing four demos:
%
%   Demo 1) PN vs Pure Pursuit crossing-target engagement
%   Demo 2) MPC vs No-MPC NFZ avoidance
%   Demo 3) Fair full-mission comparison:
%              1) route + basic
%              2) route + PN
%              3) MPC   + basic
%              4) MPC   + PN
%              5) manual environment
%   Demo 4) Parameter sweeps and mission statistics:
%              - fair four-case comparison
%              - PN gain sweep
%              - chaser-to-UAV speed-ratio sweep
%              - MPC horizon sweep
%
% Demo 1, Demo 2, and Demo 3 support:
%   - Plot only
%   - Plot + animation
%
% Demo 4 is console/table based and exports result tables to the workspace.
%
% Usage:
%   >> run_thesis_menu
%
% Author:
%   Mehmet Barış Özçelik
%
% Program:
%   M.S. Aerospace Engineering, University of Texas at Arlington

    % Ensure all subfolders are on the MATLAB path.
    thisDir = fileparts(mfilename('fullpath'));
    addpath(genpath(thisDir));

    % Launch main GUI.
    create_gui_launcher();

    %======================================================================
    % Nested GUI creator
    %======================================================================
    function create_gui_launcher()

        % Main figure -----------------------------------------------------
        f = uifigure('Name','Thesis Mission Menu', ...
                     'Position',[100 100 700 450], ...
                     'Color',[1 1 1]);

        gl = uigridlayout(f,[4 2]);
        gl.RowHeight     = {45, '1x', '1x', 55};
        gl.ColumnWidth   = {'1x','1x'};
        gl.Padding       = [12 12 12 12];
        gl.RowSpacing    = 8;
        gl.ColumnSpacing = 10;

        % Title label -----------------------------------------------------
        lblTitle = uilabel(gl, ...
            'Text','Autonomous UAV Mission Planning Under Threat', ...
            'FontSize',17, ...
            'FontWeight','bold', ...
            'HorizontalAlignment','center');

        lblTitle.Layout.Row    = 1;
        lblTitle.Layout.Column = [1 2];

        %------------------------------------------------------------------
        % Left: demo selection
        %------------------------------------------------------------------
        bgDemo = uibuttongroup(gl, ...
            'Title','Select Thesis Demo', ...
            'FontWeight','bold');

        bgDemo.Layout.Row    = 2;
        bgDemo.Layout.Column = 1;

        rbDemo1 = uiradiobutton(bgDemo, ...
            'Text','Demo 1: PN vs Pure Pursuit', ...
            'Tag','demo1');

        rbDemo2 = uiradiobutton(bgDemo, ...
            'Text','Demo 2: MPC vs No-MPC NFZ Avoidance', ...
            'Tag','demo2');

        rbDemo3 = uiradiobutton(bgDemo, ...
            'Text','Demo 3: Fair Full-Mission Comparison', ...
            'Tag','demo3');

        rbDemo4 = uiradiobutton(bgDemo, ...
            'Text','Demo 4: Parameter Sweeps and Metrics', ...
            'Tag','demo4');

        rbDemo1.Position = [10 105 300 20];
        rbDemo2.Position = [10 78  300 20];
        rbDemo3.Position = [10 51  300 20];
        rbDemo4.Position = [10 24  300 20];

        bgDemo.SelectedObject = rbDemo3;

        %------------------------------------------------------------------
        % Right: full-mission scenario options
        %------------------------------------------------------------------
        bgScen = uibuttongroup(gl, ...
            'Title','Demo 3 Scenario', ...
            'FontWeight','bold');

        bgScen.Layout.Row    = 2;
        bgScen.Layout.Column = 2;

        rbSc1 = uiradiobutton(bgScen, ...
            'Text','1) route + basic', ...
            'Tag','1');

        rbSc2 = uiradiobutton(bgScen, ...
            'Text','2) route + PN', ...
            'Tag','2');

        rbSc3 = uiradiobutton(bgScen, ...
            'Text','3) MPC + basic', ...
            'Tag','3');

        rbSc4 = uiradiobutton(bgScen, ...
            'Text','4) MPC + PN', ...
            'Tag','4');

        rbSc5 = uiradiobutton(bgScen, ...
            'Text','5) Manual environment', ...
            'Tag','5');

        rbSc1.Position = [10 110 280 18];
        rbSc2.Position = [10 87  280 18];
        rbSc3.Position = [10 64  280 18];
        rbSc4.Position = [10 41  280 18];
        rbSc5.Position = [10 18  280 18];

        bgScen.SelectedObject = rbSc4;

        %------------------------------------------------------------------
        % Left, row 3: visualization options
        %------------------------------------------------------------------
        bgVis = uibuttongroup(gl, ...
            'Title','Visualization', ...
            'FontWeight','bold');

        bgVis.Layout.Row    = 3;
        bgVis.Layout.Column = 1;

        rbVis1 = uiradiobutton(bgVis, ...
            'Text','1) Plot only', ...
            'Tag','1');

        rbVis2 = uiradiobutton(bgVis, ...
            'Text','2) Plot + animation (MP4/GIF)', ...
            'Tag','2');

        rbVis1.Position = [10 55 260 20];
        rbVis2.Position = [10 27 260 20];

        bgVis.SelectedObject = rbVis1;

        %------------------------------------------------------------------
        % Right, row 3: description panel
        %------------------------------------------------------------------
        pnlInfo = uipanel(gl, ...
            'Title','Description', ...
            'FontWeight','bold');

        pnlInfo.Layout.Row    = 3;
        pnlInfo.Layout.Column = 2;

        lblInfo = uilabel(pnlInfo, ...
            'Position',[10 10 320 95], ...
            'Text', sprintf([ ...
                'Demo 1: PN vs Pure Pursuit under equal limits.\n', ...
                'Demo 2: MPC vs No-MPC NFZ avoidance.\n', ...
                'Demo 3: Fair route/MPC and basic/PN comparison.\n', ...
                'Demo 4: PN gain, chaser-to-UAV speed-ratio, and MPC horizon sweeps.']), ...
            'HorizontalAlignment','left');

        lblInfo.FontSize = 10;

        %------------------------------------------------------------------
        % Bottom row: run and close buttons
        %------------------------------------------------------------------
        btnRun = uibutton(gl, 'push', ...
            'Text','Run selected demo', ...
            'FontWeight','bold', ...
            'ButtonPushedFcn',@onRunPressed);

        btnRun.Layout.Row    = 4;
        btnRun.Layout.Column = 1;

        btnClose = uibutton(gl, 'push', ...
            'Text','Close', ...
            'ButtonPushedFcn',@(src,evt) delete(f));

        btnClose.Layout.Row    = 4;
        btnClose.Layout.Column = 2;

        % Update enabled/disabled controls based on selected demo.
        bgDemo.SelectionChangedFcn = @onDemoChanged;
        onDemoChanged([],[]);

        %==================================================================
        % Callback: demo selection changed
        %==================================================================
        function onDemoChanged(~, ~)

            sel = bgDemo.SelectedObject;

            if isempty(sel)
                return;
            end

            switch sel.Tag

                case 'demo1'
                    bgScen.Enable = 'off';
                    bgVis.Enable  = 'on';

                case 'demo2'
                    bgScen.Enable = 'off';
                    bgVis.Enable  = 'on';

                case 'demo3'
                    bgScen.Enable = 'on';
                    bgVis.Enable  = 'on';

                case 'demo4'
                    bgScen.Enable = 'off';
                    bgVis.Enable  = 'off';

                otherwise
                    bgScen.Enable = 'off';
                    bgVis.Enable  = 'on';
            end
        end

        %==================================================================
        % Callback: run selected demo
        %==================================================================
        function onRunPressed(~, ~)

            demoSel = bgDemo.SelectedObject;

            if isempty(demoSel)
                uialert(f,'Please select a demo.','No Demo Selected');
                return;
            end

            demoTag = demoSel.Tag;

            % Visualization selection
            visSelObj = bgVis.SelectedObject;

            if isempty(visSelObj)
                visSel = 1;
            else
                visSel = str2double(visSelObj.Tag);
            end

            % Scenario selection for Demo 3
            scenSelObj = bgScen.SelectedObject;

            if isempty(scenSelObj)
                scenSel = 4;
            else
                scenSel = str2double(scenSelObj.Tag);
            end

            drawnow;

            switch demoTag

                case 'demo1'
                    clc;
                    fprintf('==============================================\n');
                    fprintf('[GUI] Demo 1: PN vs Pure Pursuit\n');
                    fprintf('==============================================\n\n');

                    if visSel == 2
                        sim1_pn_vs_pp_crossing_anim();
                    else
                        sim1_pn_vs_pp_crossing();
                    end

                case 'demo2'
                    clc;
                    fprintf('==============================================\n');
                    fprintf('[GUI] Demo 2: MPC vs No-MPC NFZ Avoidance\n');
                    fprintf('==============================================\n\n');

                    if visSel == 2
                        sim2_mpc_avoid_anim();
                    else
                        sim2_mpc_avoid();
                    end

                case 'demo3'
                    clc;
                    fprintf('==============================================\n');
                    fprintf('[GUI] Demo 3: Fair Full-Mission Comparison\n');
                    fprintf('==============================================\n\n');

                    sim3_full_mission_compare(scenSel, visSel);

                case 'demo4'
                    clc;
                    fprintf('==============================================\n');
                    fprintf('[GUI] Demo 4: Parameter Sweeps and Metrics\n');
                    fprintf('==============================================\n\n');

                    sim3_param_sweep();

                otherwise
                    uialert(f,'Unknown demo selection.','Error');
            end

        end

    end

end