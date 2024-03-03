addpath(genpath('plot'));

%% Plot race
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.race();
%% Plot car positions on all horizons
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.carPositions();

%% Plot steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle on all horizons
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.angles();

%% Plot bounded states on all horizons
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.boundedStates();

%% Plot bounded inputs on all horizons
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.boundedInputs();

%% Plot ax, ay
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.accelerationsDistribution();

%% Plot costs, upper bounds slacks and lower bounds slacks
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.costsAndSlacks();

%% Plot costs, upper bounds slacks and lower bounds slacks on only first horizon
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.constrAndSlacks();

%% Plot steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle on a race
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.raceAngles();

%% Plot vx, vy, r
plotObj = Plot(config,parameters,track,log,carModel);
plotObj.velocities();
