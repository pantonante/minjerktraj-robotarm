function qstar = invk(robot, options) 
% Inverse kinematics: from cartesian pt to q in joint space
% the obj.func. can be penalized by an arbitraty function
    
    % default options for ikcon
    problem.options = optimoptions('fmincon', ...
        'Algorithm', 'active-set', ...
        'Display', 'off');%,...             % alternative Display iter
        %'PlotFcns', @optimplotfval);
    problem.solver = 'fmincon';
    problem.options.TolCon = 1e-12;
    
    % set the joint limit bounds
    problem.lb = robot.qlim(:,1);
    problem.ub = robot.qlim(:,2);
   
    % initial condition
    problem.x0 =  options.qStart;
    
    % -----
    problem.objective = ...
        @(x) sumsqr(options.xStop - getEEpos(robot,x)) + ...
             options.penalizationFactor * options.penalizationFunctional(x);
         
    [qstar, ~, ~] = fmincon(problem);    
end

function s = sumsqr(A)
    s = sum(A(:).^2);
end

