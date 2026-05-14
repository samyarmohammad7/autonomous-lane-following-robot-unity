% P2_main.m
% Part 2 workflow for the SCARA RRP robot.

clc;
clear;
close all;

params = scara_parameters();
outFolder = ensure_output_folder(fullfile(pwd, 'P2_outputs'));

check_robotics_toolbox();

print_section('Task 1 - Forward kinematics');
show_fk_summary(params);

print_section('Task 2 - RTB model and validation');
robot = build_scara_rtb(params.d1, params.d2, params.d3, params.d4, params.d5_limit);
robot.display();

caseNames = {'Home'; 'Case 2'; 'Case 3'};
qCases = [0, 0, 0;
          deg2rad(30), deg2rad(45), 0.10;
          deg2rad(90), deg2rad(-90), params.d5_limit];

validationTable = validate_fk_cases(robot, params, qCases, caseNames);
disp(validationTable);
writetable(validationTable, fullfile(outFolder, 'P2_validation_table.csv'));

homeFig = figure('Name', 'SCARA Home Position', 'Position', [100 100 850 650]);
robot.plot(qCases(1, :), 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6]);
title('SCARA Robot (RRP) - Home Position');
save_fig(homeFig, fullfile(outFolder, 'Fig_P2_Home_Position.png'));

qStart = qCases(1, :);
qEnd = [deg2rad(60), deg2rad(45), 0.15];
qTrajectory = jtraj(qStart, qEnd, 50);
eeTrajectory = fk_position_series(robot, qTrajectory);

trajFig = figure('Name', 'SCARA Test Trajectory', 'Position', [100 100 850 650]);
robot.plot(qTrajectory, 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6], 'trail', 'r-');
title('SCARA Robot - Test Trajectory');
save_fig(trajFig, fullfile(outFolder, 'Fig_P2_Test_Trajectory.png'));

profileFig = figure('Name', 'End-Effector Position vs Step', 'Position', [100 100 950 400]);
plot_position_components(profileFig, eeTrajectory);
save_fig(profileFig, fullfile(outFolder, 'Fig_P2_EndEffector_Position_vs_Step.png'));

print_section('Task 3 - Teaching mode and playback');
fprintf('Teaching mode is opening now.\n');
fprintf('Use the sliders to adjust q1, q2 and q3, then capture a few poses.\n\n');

figure('Name', 'SCARA Teaching Mode', 'Position', [100 100 950 700]);
robot.plot(qCases(1, :), 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6]);
robot.teach(qCases(1, :));
fprintf('Close the teach panel before continuing to the waypoint playback.\n\n');

waypoints = [0, 0, 0;
             deg2rad(30),  deg2rad(20), 0.05;
             deg2rad(60),  deg2rad(-30), 0.10;
             deg2rad(-45), deg2rad(45), 0.15;
             0, 0, 0];

report_waypoints(robot, waypoints);
qPlayback = build_waypoint_trajectory(waypoints, 30);

playbackFig = figure('Name', 'SCARA Waypoint Playback', 'Position', [100 100 950 700]);
robot.plot(qPlayback, 'workspace', [-0.8 0.8 -0.8 0.8 -0.2 0.6], 'trail', 'r-');
title('SCARA Robot - Teaching Mode Playback');
save_fig(playbackFig, fullfile(outFolder, 'Fig_P2_Waypoint_Playback.png'));

eePlayback = fk_position_series(robot, qPlayback);
pathFig = figure('Name', 'End-Effector 3D Path', 'Position', [100 100 850 650]);
plot_3d_path(pathFig, eePlayback, fk_position_series(robot, waypoints));
save_fig(pathFig, fullfile(outFolder, 'Fig_P2_EndEffector_Path_3D.png'));

fprintf('\nResults saved in %s\n', outFolder);

function params = scara_parameters()
params.d1 = 0.3;
params.d2 = 0.06;
params.d3 = 0.3;
params.d4 = 0.3;
params.d5_limit = 0.2;
end

function outFolder = ensure_output_folder(outFolder)
if ~exist(outFolder, 'dir')
    mkdir(outFolder);
end
end

function check_robotics_toolbox()
try
    startup_rvc;
catch ME
    error(['Robotics Toolbox could not be started.\n' ...
        'Add the rvctools folder to the MATLAB path first.\n%s'], ME.message);
end

requiredFunctions = {'SerialLink', 'Link', 'jtraj', 'transl'};
missingFunctions = strings(0, 1);

for k = 1:numel(requiredFunctions)
    if isempty(which(requiredFunctions{k}))
        missingFunctions(end + 1, 1) = string(requiredFunctions{k}); %#ok<AGROW>
    end
end

if ~isempty(missingFunctions)
    error('Missing RTB functions on the MATLAB path: %s', strjoin(missingFunctions, ', '));
end

fprintf('Robotics Toolbox loaded successfully.\n');
end

function show_fk_summary(params)
fprintf('Robot parameters (m): d1 = %.2f, d2 = %.2f, d3 = %.2f, d4 = %.2f, d5 = %.2f\n\n', ...
    params.d1, params.d2, params.d3, params.d4, params.d5_limit);

dhTable = table((1:3)', ...
    ["theta1"; "theta2"; "0"], ...
    ["d1"; "d2"; "d5"], ...
    ["d3"; "d4"; "0"], ...
    ["0"; "pi"; "0"], ...
    'VariableNames', {'Joint', 'theta', 'd', 'a', 'alpha'});

disp(dhTable);

fprintf('Transformation matrix:\n');
fprintf('    [ cos(th1+th2)   sin(th1+th2)   0   d4*cos(th1+th2)+d3*cos(th1) ]\n');
fprintf('T = [ sin(th1+th2)  -cos(th1+th2)   0   d4*sin(th1+th2)+d3*sin(th1) ]\n');
fprintf('    [      0              0        -1          d1 + d2 - d5         ]\n');
fprintf('    [      0              0         0                1               ]\n\n');

qExample = [deg2rad(30), deg2rad(45), 0.10];
TExample = scara_fkine_manual(qExample(1), qExample(2), qExample(3), ...
    params.d1, params.d2, params.d3, params.d4);

fprintf('Example result for [30 deg, 45 deg, 0.10 m]:\n');
disp(TExample);
end

function validationTable = validate_fk_cases(robot, params, qCases, caseNames)
numCases = size(qCases, 1);

manualPos = zeros(numCases, 3);
rtbPos = zeros(numCases, 3);
positionError = zeros(numCases, 1);
matrixError = zeros(numCases, 1);

for k = 1:numCases
    q = qCases(k, :);

    TManual = scara_fkine_manual(q(1), q(2), q(3), ...
        params.d1, params.d2, params.d3, params.d4);
    TRtb = robot.fkine(q);
    TRtbNum = pose_to_matrix(TRtb);

    manualPos(k, :) = TManual(1:3, 4).';
    rtbPos(k, :) = transl(TRtb).';

    positionError(k) = norm(manualPos(k, :) - rtbPos(k, :));
    matrixError(k) = norm(TManual - TRtbNum, 'fro');

    fprintf('\n%s\n', caseNames{k});
    fprintf('q = [%.4f rad, %.4f rad, %.4f m]\n', q(1), q(2), q(3));
    fprintf('Manual position: [%.4f, %.4f, %.4f]\n', manualPos(k, 1), manualPos(k, 2), manualPos(k, 3));
    fprintf('RTB position:    [%.4f, %.4f, %.4f]\n', rtbPos(k, 1), rtbPos(k, 2), rtbPos(k, 3));
    fprintf('Position error:  %.6e\n', positionError(k));
    fprintf('Matrix error:    %.6e\n', matrixError(k));
end

validationTable = table(caseNames, ...
    manualPos(:, 1), manualPos(:, 2), manualPos(:, 3), ...
    rtbPos(:, 1), rtbPos(:, 2), rtbPos(:, 3), ...
    positionError, matrixError, ...
    'VariableNames', {'Case', ...
    'px_manual', 'py_manual', 'pz_manual', ...
    'px_rtb', 'py_rtb', 'pz_rtb', ...
    'PositionError', 'MatrixError'});
end

function positions = fk_position_series(robot, qMatrix)
positions = zeros(size(qMatrix, 1), 3);

for k = 1:size(qMatrix, 1)
    positions(k, :) = transl(robot.fkine(qMatrix(k, :))).';
end
end

function report_waypoints(robot, waypoints)
fprintf('\nWaypoint list:\n');

for k = 1:size(waypoints, 1)
    position = transl(robot.fkine(waypoints(k, :))).';
    fprintf('WP%d: [%6.1f deg, %6.1f deg, %5.3f m] -> (%.4f, %.4f, %.4f)\n', ...
        k, rad2deg(waypoints(k, 1)), rad2deg(waypoints(k, 2)), waypoints(k, 3), ...
        position(1), position(2), position(3));
end
end

function qTrajectory = build_waypoint_trajectory(waypoints, stepsPerSegment)
segments = cell(size(waypoints, 1) - 1, 1);

for k = 1:numel(segments)
    segments{k} = jtraj(waypoints(k, :), waypoints(k + 1, :), stepsPerSegment);
end

qTrajectory = vertcat(segments{:});
end

function plot_position_components(figHandle, positions)
layout = tiledlayout(figHandle, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
labels = {'X', 'Y', 'Z'};

for k = 1:3
    ax = nexttile(layout);
    plot(ax, positions(:, k), 'b-', 'LineWidth', 1.5);
    xlabel(ax, 'Step');
    ylabel(ax, sprintf('%s (m)', labels{k}));
    title(ax, sprintf('%s Position', labels{k}));
    grid(ax, 'on');
end

sgtitle('End-Effector Position Along Trajectory');
end

function plot_3d_path(figHandle, pathPoints, waypointPoints)
ax = axes('Parent', figHandle);
plot3(ax, pathPoints(:, 1), pathPoints(:, 2), pathPoints(:, 3), 'r-', 'LineWidth', 2);
hold(ax, 'on');

for k = 1:size(waypointPoints, 1)
    plot3(ax, waypointPoints(k, 1), waypointPoints(k, 2), waypointPoints(k, 3), ...
        'b*', 'MarkerSize', 12, 'LineWidth', 1.5);
    text(ax, waypointPoints(k, 1) + 0.02, waypointPoints(k, 2) + 0.02, ...
        waypointPoints(k, 3) + 0.02, sprintf('WP%d', k), ...
        'FontSize', 10, 'FontWeight', 'bold');
end

xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
title(ax, 'End-Effector Path Through Waypoints');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 3);
hold(ax, 'off');
end

function print_section(titleText)
underline = repmat('-', 1, strlength(titleText));
fprintf('\n%s\n%s\n', titleText, underline);
end

function TNum = pose_to_matrix(TObj)
try
    TNum = double(TObj);
catch
    TNum = TObj.T;
end
end

function save_fig(figHandle, fileName)
try
    exportgraphics(figHandle, fileName, 'Resolution', 300);
catch
    saveas(figHandle, fileName);
end
end
