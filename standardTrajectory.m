function stdTraj = standardTrajectory( task )

stdTraj.execTime = tic();

%%                              Trajectory                                %
stdTraj.N    = task.gridSize * (task.robot.n - 1);
stdTraj.Ts   = task.Tstop/(stdTraj.N-1);
stdTraj.t    = 0 : stdTraj.Ts : task.Tstop;

stdTraj.Q    = jtraj(task.qStart, task.qStop, stdTraj.N);
stdTraj.dQ   = diff(stdTraj.Q)/stdTraj.Ts;
stdTraj.ddQ  = diff(stdTraj.dQ)/stdTraj.Ts;
stdTraj.dddQ = diff(stdTraj.ddQ)/stdTraj.Ts;


%%                               Jerk Cost                                %
[stdTraj.jerkCostIntegral, stdTraj.jerkCostFunction] = jerkCost(stdTraj.dddQ);


%%                             Manipulability                             %
for i = 1:stdTraj.N
    stdTraj.manipulability(i) = real(task.robot.maniplty(stdTraj.Q(i,:)));
    %J = task.robot.jacob0(stdTraj.Q(i,:));
    %stdTraj.manipolability(i) = sqrt(det(J*J'));
end

stdTraj.execTime = toc(stdTraj.execTime);
end

