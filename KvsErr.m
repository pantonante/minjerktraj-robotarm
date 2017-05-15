%% Params
%Makes the graph of penalization factor vs position error
robot = lexos;
xStop = [12.565 -48.06 45.061]'; % representative point
Kstart = 0.0029818/10;              % penalization factor subspace
Kstop = 0.0029818/40;
samples = 50;

penFunc = @(q) -abs(robot.maniplty(q));
%penFunc = @(q) potentialEnergy(robot,q);

%% Plot
k = Kstart : (Kstop-Kstart)/samples : Kstop;
e = zeros(length(k),1);
rho = zeros(length(k),1);
p=-1;

% invk option creation
options.qStart = zeros(robot.n, 1)';
options.xStop = xStop;
options.penalizationFunctional = penFunc;

disp('Processing percentage (%)...');
fprintf(sprintf('| '));
for i=1:length(k) 
    options.penalizationFactor = k(i);
    qStar = invk(robot, options);
    e(i) = norm(xStop - getEEpos(robot, qStar));
    rho(i) = penFunc(qStar);
    %process
    tempProcess=floor(i/(samples*10)*100);
    if(tempProcess ~= p)
        p = tempProcess;
        S = sprintf('%d | ', p*10); 
        fprintf(S); 
    end
end
S = sprintf('\n'); 
fprintf(S); 
    
% Position Err. vs K
plot(k,e);
xlabel('k')
ylabel('Error [cm]')

% Position Err. vs Penalization Function vs K
% figure
% plot3(k,e,rho);

%% clearing
clear robot xStop Kstart Kstop samples penFunc p options qStar tempProcess S