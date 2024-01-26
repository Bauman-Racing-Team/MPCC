    % Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% % You may obtain a copy of the License at

%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%% MPCC Simulation Script
clear
close all
clc

%% add subdirectories
addpath('model');
addpath('mpc');
addpath('parameters');
addpath('simulator');
addpath('spline');
addpath('tracks');
addpath('types');

%addpath('/opt/casadi/')
%% add subdirectories for the chosen solver

config = config();
parameters = Parameters(config);

global mpc simulator;
if strcmp(config.solver,'ipopt')
    addpath('ipopt');
    mpc = Ipopt(config,parameters);
elseif strcmp(config.solver,'acados')
    addpath('acados/');
    mpc = Acados(config,parameters);
else
    disp('Wrong solver, choose another one in config.m');
    return
end

trackNameFile = 'FSI.mat'; %track name
load(trackNameFile);

track = Track(cones_blue, cones_yellow);

simulator = Simulator(config,parameters.car,parameters.tire);

mpc.setTrack(track);

trackCenter = mpc.getTrack().getPath();

trackPath = mpc.getTrack().getPath();
trackLength = mpc.getTrack().getLength();

% initial point
point0 = 1;

phi0 = atan2(trackPath.y(point0+1) - trackPath.y(point0),trackPath.x(point0+1) - trackPath.x(point0));
x0 = [trackPath.x(point0);trackPath.y(point0);phi0;0;0;0;0;0;0;0;0;0;0];

mpc.initMPC();

% declare ros2 nodes
mpcsolverNode = ros2node("/mpcsolver");
simulatorNode = ros2node("/simulator");

global solverPub simulatorPub solverPubmsg simulatorPubmsg;
solverPub = ros2publisher(mpcsolverNode,'/kesi','std_msgs/Float64MultiArray');
solverPubmsg = ros2message(solverPub);
pause(1); 
simulatorPub = ros2publisher(simulatorNode,'/state','std_msgs/Float64MultiArray');
simulatorPubmsg = ros2message(simulatorPub);
pause(1); 

solverSub = ros2subscriber(mpcsolverNode,'/state',@solverCallback);
pause(1); 
simulatorSub = ros2subscriber(simulatorNode,'/kesi',@simulatorCallback);
pause(1);

global i log;
i=1;
log=MpcReturn;
simulatorPubmsg.data = x0;
send(simulatorPub,simulatorPubmsg)

% for i = 1:parameters.config.nSim
%     mpcSol = mpc.runMPC(x0(1:11));
%     x0 = simulator.simTimeStep(x0,mpcSol.u0,parameters.config.ts);
%     log(i) = mpcSol;
%     x00(:,i) = x0;
%     disp("Iteraton:");
%     disp(i);
%     if mpcSol.solverStatus ~= 0
%         error('solver returned status %d in closed loop iteration %d. Exiting.', mpcSol.solverStatus);
%     end
% end

function simulatorCallback(msg)
    global simulatorPub simulatorPubmsg simulator;
    ts = 0.05;
    kesi = msg.data;
    x0 = kesi(1:13,1);
    u0 = kesi(14:17,1);
    x0 = simulator.simTimeStep(x0,u0,ts);
    simulatorPubmsg.data = x0;
    send(simulatorPub,simulatorPubmsg)
end

function solverCallback(msg)
    global solverPub solverPubmsg mpc i log;
    x0 = msg.data;
    mpcSol = mpc.runMPC(x0(1:11));
    kesi = [msg.data;mpcSol.u0];
    solverPubmsg.data = kesi;
    send(solverPub,solverPubmsg)
    log(i) = mpcSol;
    disp("Iteraton:");
    disp(i);
    i = i+1;
    if mpcSol.solverStatus ~= 0
        error('solver returned status %d in closed loop iteration %d. Exiting.', mpcSol.solverStatus);
    end
end
