function [Q, j] = jerkCost( jerk )

j2=jerk.^2;
j= sum(j2,2); %?j2(:,i)
Q = trapz(j);

end

