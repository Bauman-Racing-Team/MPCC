    % Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
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

config = configTRO();
parameters = Parameters(config);

if strcmp(config.solver,'ipopt')
    addpath('ipopt');
    TRO = Ipopt(config,parameters);
elseif strcmp(config.solver,'acados')
    addpath('acados/');
    TRO = AcadosTRO(config,parameters);
else
    disp('Wrong solver, choose another one in config.m');
    return
end

trackNameFile = 'FSG.mat'; %track name
load(trackNameFile);

track = Track(cones_blue, cones_yellow);

carModel = Model(parameters.car,parameters.tire);

TRO.setTrack(track);

trackCenter = TRO.getTrack().getPath();

trackPath = TRO.getTrack().getPath();
trackLength = TRO.getTrack().getLength();

% initial point
x0 = [0;0;0;0;0;0;0;0];

TRO.initMPC();
log = MpcReturn.empty(1, 0);

mpcSol = TRO.runTRO(x0);
if ~isempty(mpcSol.x0)
    log(end+1) = mpcSol;
end