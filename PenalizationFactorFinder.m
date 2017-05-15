%% Computes variable sizes (min, max)
% The whole script computes kbase given a maximum position error

    robot = lexos;

    % default options for ikcon
    problem.options = optimoptions('fmincon', ...
        'Algorithm', 'active-set', ...
        'Display', 'off');%,...
        %'PlotFcns', @optimplotfval);
    problem.solver = 'fmincon';
    problem.options.TolCon = 1e-12;
    % set the joint limit bounds
    problem.lb = robot.qlim(:,1);
    problem.ub = robot.qlim(:,2);
    % initial condition
    problem.x0 =  zeros(robot.n,1)';

    deltaErr = 190; %maxerr for inv. kinematics (min: 0)
    
%% min manply(q), q in Omega
    problem.objective = @(q) abs(robot.maniplty(q));
    [qstar, ~, ~] = fmincon(problem);
    minManply = abs(robot.maniplty(qstar));
    disp (['MIN manipulability: ', num2str(minManply)]);
    
%% max manply(q), q in Omega
    problem.objective = @(q) -abs(robot.maniplty(q));
    [qstar, ~, ~] = fmincon(problem);
    maxManply = abs(robot.maniplty(qstar));
    disp (['MAX manipulability: ', num2str(maxManply)]);
    
%% constant
 disp (['K manply: ', num2str(deltaErr/(maxManply-minManply))]);
    
%% min potEnergy(q), q in Omega
    problem.objective = @(q) potentialEnergy(robot,q);
    [qstar, ~, ~] = fmincon(problem);
    minPotEn = potentialEnergy(robot,qstar);
    disp (['MIN potential energy: ', num2str(minPotEn)]);
    
%% min potEnergy(q), q in Omega
    problem.objective = @(q) -potentialEnergy(robot,q);
    [qstar, ~, ~] = fmincon(problem);
    maxPotEn = potentialEnergy(robot, qstar);
    disp (['MAX potential energy: ', num2str(maxPotEn)]);
    
%% constant
 disp (['K pot. En.gy: ', num2str(deltaErr/(maxPotEn-minPotEn))]);
    
%% clearing
clear qstar problem robot deltaErr minManply maxManply minPotEn maxPotEn
