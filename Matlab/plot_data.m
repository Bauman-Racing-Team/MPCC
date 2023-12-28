addpath(genpath('plot'));

%% Plot race
plotObj = Plot(config,parameters,track,log);
plotObj.race();

% %% Plot car positions on all horizons
plotObj = Plot(config,parameters,track,log);
plotObj.carPositions();
% 
% %% Plot steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle on all horizons
% plotObj = Plot(config,parameters,track,log);
% plotObj.angles();
% 
% %% Plot bounded states on all horizons
% plotObj = Plot(config,parameters,track,log);
% plotObj.boundedStates();
% 
% %% Plot bounded inputs on all horizons
% plotObj = Plot(config,parameters,track,log);
% plotObj.boundedInputs();
% 
% %% Plot ax, ay
% plotObj = Plot(config,parameters,track,log);
% plotObj.accelerationsDistribution();
% 
%% Plot costs, upper bounds slacks and lower bounds slacks
plotObj = Plot(config,parameters,track,log);
plotObj.costsAndSlacks();
% 
% %% Plot steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle on a race
% plotObj = Plot(config,parameters,track,log);
% plotObj.raceAngles();
% 
% %% Plot vx, vy, r
plotObj = Plot(config,parameters,track,log);
plotObj.velocities();