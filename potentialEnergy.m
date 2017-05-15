function U = potentialEnergy( robot, q )

lexos = robot;
m1 = 1;
m2 = 1;
m3 = 1;
g = 9.81;

t = lexos.A([1], q);
x(1,:) = t(1:3,end);

t = lexos.A([1 2], q);
x(2,:) = t(1:3,end);

t = lexos.A(1:4, q);
x(3,:) = t(1:3,end);

t = lexos.A(1:6, q);
x(4,:) = t(1:3,end);

U = g*( m1*(x(2,3)-x(1,3)) + m2*(x(3,3)-x(2,3)) + m3*(x(4,3)-x(3,3)) );

end

