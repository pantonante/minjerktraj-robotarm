%%                          Clearing & Init                               %
clear; 
clc; 
%clf;

%example values
%ex =[12.565 -48.06 45.061]; %test point (q: [0 0.30129 0.21991 0.36444 -0.49214 0])
ex = [22.267 2.288 79.064]; %validation point (q: [0 1.2915 -0.63315 0.47805 -0.92281 0])
kman = 5.516e-5; %~0.5[error]->0.0029818; %~0.2->5.516e-5 
kpoten = 0.04087; %~0.5[error]->0.21973; %~0.2->0.04087

startRTB = input('Startup Robotic Toolbox? Y/N [N]:','s');
if strcmpi(startRTB,'y')
    startup_rvc   
end
clear startRTB

% initialize robot object
disp('Robot: lexos');
InitLexos
task.robot = lexos;
options.robot = lexos;

%%                            -- Params --                                %
disp('T stop: 5 (sec)')
task.Tstop = 5;
disp('Joint angular rate limit: 1 rad/sec')
task.dqLimit =  repmat(1, task.robot.n, 1); % angular speed limit

task.gridSize = input('Grid size [10]: ');
if isempty(task.gridSize)
    task.gridSize = 10;
elseif task.gridSize<4
    disp('Min. possible selected: 4');
    task.gridSize = 4;
end

task.qStart = input('Initial pose (joint angles) [0 0 ...]: ');
if length(task.qStart) == 6
    if isequal(size(task.qStart),[task.robot.n 1])
        task.qStart = task.qStart';
    end
else
    task.qStart = zeros(task.robot.n,1)';
end
options.qStart = task.qStart;

options.xStop=[];
while isempty(options.xStop)
    options.xStop =  input('Stop point (cartesian coordinate): '); %e.g. getEEpos(lexos, [0 0 -45 45 0 0]*(pi/180)) = [0 -22.8 82.289]
end
if(isequal(size(options.xStop),[1 3]))
        options.xStop = options.xStop';
end

askPenFactor = true;
penalizationFunc = input('Penalization function [a]: \n   a) Manipulability\n   b) Potential energy\n   c) Nothing\nChoice: ', 's');
if strcmpi(penalizationFunc,'b')
    disp('... -> Potential energy');
    options.penalizationFunctional = @(q) potentialEnergy(task.robot,q);
    k_suggestion = kpoten;
elseif strcmpi(penalizationFunc,'c')
    options.penalizationFunctional = @(q) 0;
    askPenFactor = false;
else
    disp('... -> Manipulability');
    options.penalizationFunctional = @(q) -abs(task.robot.maniplty(q));
    k_suggestion = kman;
end
clear penalizationFunc

if(askPenFactor)
    gain = input(['Penalization factor (initial guess) [', num2str(k_suggestion),']: ']);
    if isempty(gain)
        options.penalizationFactor = k_suggestion;
    else
        options.penalizationFactor = gain;
    end
else
    options.penalizationFactor = 0;
end
clear gain k_suggestion

if(askPenFactor)
    epsilon = input('Desired max error in cm [0.2]: ');
    if isempty(epsilon)
        epsilon = 0.2;
    end
    options.penalizationFactor = find4me(options, options.penalizationFactor, epsilon);
    disp(['Pen. Factor: ', num2str(options.penalizationFactor)])
end
clear askPenFactor

whichAnimation = input('Animation: "std" or "minjerk" otherwise nothing is displayed [<nothing>]? ','s');
displayPlot = input('Display plots? Y/N [N]: ','s');
if isempty(displayPlot)
    displayPlot = 'n';
end

%% Compute inverse kinematics
disp(sprintf ('Inverse kinematics optimization problem solution...'))
task.qStop = invk(task.robot, options);

%%                         Min. Jerk Trajectory                           %
disp(sprintf ('Minimum jerk optimization problem solution...'))
minJerkTraj = minJerkTrajectory(task);

if strcmpi(displayPlot,'y')
    % Angle plot
    subplot(4,1,1)
    plot(minJerkTraj.t, minJerkTraj.Q)
    ylabel('angle (rad)')
    title('Min. Jerk Traj. - Joint angles')
    legend('q1','q2','q3','q4','q5','q6');
    % Velocity plot
    subplot(4,1,2)
    plot(minJerkTraj.t, minJerkTraj.dQ)
    ylabel('rate (rad/s)')
    title('Min. Jerk Traj. - Joint angular velocity')
    %Acceleration plot
    subplot(4,1,3)
    plot(minJerkTraj.t(2:end), minJerkTraj.ddQ)
    ylabel('rate (rad/s^2)')
    title('Min. Jerk Traj. - Joint angular acceleration')
    %Jerk plot
    subplot(4,1,4)
    plot(minJerkTraj.t(3:end), minJerkTraj.dddQ)
    ylabel('rate (rad/s^3)')
    title('Min. Jerk Traj. - Joint jerk')
end

%%                    Standard Trajectory (jtraj)                         %
stdTraj = standardTrajectory(task);

if strcmpi(displayPlot,'y')
    figure
    % Angle plot
    subplot(4,1,1)
    plot(stdTraj.t, stdTraj.Q)
    ylabel('angle (rad)')
    title('Std. Traj. - Joint angles')
    legend('q1','q2','q3','q4','q5','q6');
    % Velocity plot
    subplot(4,1,2)
    plot(stdTraj.t(2:end), stdTraj.dQ)
    ylabel('rate (rad/s)')
    title('Std. Traj. - Joint angular velocity')
    %Acceleration plot
    subplot(4,1,3)
    plot(stdTraj.t(3:end), stdTraj.ddQ)
    ylabel('rate (rad/s^2)')
    title('Std. Traj. - Joint angular acceleration')
    %Jerk plot
    subplot(4,1,4)
    plot(stdTraj.t(4:end), stdTraj.dddQ)
    ylabel('rate (rad/s^3)')
    title('Std. Traj. - Joint jerk')
end

%%                       Manipulability comparison                        %
if strcmpi(displayPlot,'y')
    figure
    plot(minJerkTraj.t, minJerkTraj.manipulability, minJerkTraj.t, stdTraj.manipulability)
    legend('Min.Jerk Traj.','Std.Traj')
    title('Manipulability')
end

%%                              Jerks                                     %
if strcmpi(displayPlot,'y')
    figure
    hold on
    plot(minJerkTraj.jerkCostFunction)
    plot(stdTraj.jerkCostFunction)
    legend('Min.Jerk Traj.','Std.Traj')
    title('Jerk Const Functions')
    hold off
end

%%                            Comparisons
disp(sprintf ('-------- Results -----------'));
disp(['q: ', num2str(task.qStop)]);
disp(['Pos. Error: ', num2str(norm(options.xStop - getEEpos(task.robot, stdTraj.Q(end,:)))), ' (cm)']);
disp(['Manipulability: ', num2str(real(task.robot.maniplty(minJerkTraj.Q(end,:))))]);
disp(['Pot. Energy: ', num2str(potentialEnergy(task.robot, minJerkTraj.Q(end,:)))]);

disp(sprintf (''));
disp(['Jerk Cost (min.jerk traj.): ', num2str(minJerkTraj.jerkCostIntegral)])
disp(['Jerk Cost (std. traj.): ', num2str(stdTraj.jerkCostIntegral)])
if minJerkTraj.jerkCostIntegral < stdTraj.jerkCostIntegral
    v = (1-minJerkTraj.jerkCostIntegral/stdTraj.jerkCostIntegral)*100;
    disp(['Jerk cost decrease: ', num2str(v),'%']);
else
    v = (1-stdTraj.jerkCostIntegral/minJerkTraj.jerkCostIntegral)*100;
    disp(['Jerk cost increase: ', num2str(v),'%']);
end

disp(sprintf (''));
stdPeak = max(stdTraj.jerkCostFunction);
minJerkPeak = max(minJerkTraj.jerkCostFunction);
if minJerkPeak < stdPeak
    v = (1-minJerkPeak/stdPeak)*100;
    disp(['Jerk peak decrease: ', num2str(v),'%']);
else
    v = (1-stdPeak/minJerkPeak)*100;
    disp(['Jerk peak increase: ', num2str(v),'%']);
end
clear stdPeak minJerkPeak v

%%                         L-Exos Animation   
if(strcmpi(whichAnimation,'std'))
    figure
    hold on;
    T = lexos.fkine(stdTraj.Q);
    P = squeeze(T(1:3,4,:))';
    plot3(P(:,1), P(:,2), P(:,3));
    scatter3(options.xStop(1),options.xStop(2),options.xStop(3))
    lexos.plot(stdTraj.Q)
else if (strcmpi(whichAnimation,'minjerk'))
    figure
    hold on;
    T = lexos.fkine(minJerkTraj.Q);
    P = squeeze(T(1:3,4,:))';
    plot3(P(:,1), P(:,2), P(:,3));
    scatter3(options.xStop(1),options.xStop(2),options.xStop(3))
    lexos.plot(minJerkTraj.Q)
    end
end
clear T  P
%lexos.vellipse(minJerkTraj.Q(end,:))