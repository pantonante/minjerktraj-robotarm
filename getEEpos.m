function x = getEEpos( robot, q )
%GETEEPOS Summary of this function goes here
%   Detailed explanation goes here

x = robot.fkine(q);
x = x(1:3,4);

end

