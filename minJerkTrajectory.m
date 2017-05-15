function minJerkTraj = minJerkTrajectory( task )
%MINJERKTRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

minJerkTraj.execTime = tic();

tNode = [0, task.Tstop];
qNode =  [task.qStart', task.qStop'];

for i=1:task.robot.n;
    problem.qLow =  task.robot.qlim(i, 1);
    problem.qUpp =  task.robot.qlim(i, 2);
    problem.dqMax = task.dqLimit(i);
    problem.tNode = tNode;
    problem.qNode = qNode(i,:);
    problem.nGrid =  task.gridSize;
    soln(i) = smoothJointTrajectory(problem);
end

%%                   min. Jerk Trajectory computation                     %
minJerkTraj.N = task.gridSize * (task.robot.n - 1);
minJerkTraj.Ts = task.Tstop/(minJerkTraj.N-1);
minJerkTraj.Q    =  zeros(minJerkTraj.N, task.robot.n);
minJerkTraj.dQ   =  zeros(minJerkTraj.N, task.robot.n);
minJerkTraj.ddQ  =  zeros(minJerkTraj.N-1, task.robot.n);
minJerkTraj.dddQ =  zeros(minJerkTraj.N-2, task.robot.n);
minJerkTraj.manipulability = zeros(minJerkTraj.N, 1);

% Trajectory
minJerkTraj.t = soln(1).segment.interp.t;
for i = 1:task.robot.n
    minJerkTraj.Q(:,i) = soln(i).segment.interp.q';   % joint angles
    minJerkTraj.dQ(:,i) = soln(i).segment.interp.dq'; % joint angular velocity
    minJerkTraj.ddQ(:,i) = diff(minJerkTraj.dQ(:,i))/minJerkTraj.Ts;          % joint angular acceleration
    minJerkTraj.dddQ(:, i) = diff(minJerkTraj.ddQ(:,i))/minJerkTraj.Ts;       % joint jerk
end

% Manipulability
for i = 1:minJerkTraj.N
    minJerkTraj.manipulability(i) = real(task.robot.maniplty(minJerkTraj.Q(i,:))); %real - avoids numeric instabilities
    %J = task.robot.jacob0(minJerkTraj.Q(i,:));
    %minJerkTraj.manipulability(i) = sqrt(det(J*J'));
end 

% Jerk Cost
[minJerkTraj.jerkCostIntegral, minJerkTraj.jerkCostFunction] = jerkCost(minJerkTraj.dddQ);

minJerkTraj.execTime = toc(minJerkTraj.execTime);

end

