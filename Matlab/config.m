function config = config()
    config.solver = 'acados'; % ipopt, hpipm, acados
    config.simulator = 'dynamic'; % kinematic, simple_dynamic, dynamic
    config.NX = 11;
    config.NU  = 4;
    
    config.NB = 11;  % max number of bounds
    config.NPC = 2;  % number of polytopic constraints (2 for hpipm and 3 for ipopt and acados)
    config.NS = 3; % number of softened constraints (2 for hpipm and 3 for ipopt and acados)

    config.N = 100;
    config.NSpline = 5000;

    config.siIndex = struct('x',1, ...
                     'y',2, ...
                     'yaw',3, ...
                     'vx',4, ...
                     'vy',5, ...
                     'r',6, ...
                     's',7, ...
                     'throttle',8, ...
                     'steeringAngle',9, ...
                     'brakes',10, ...
                     'vs',11, ...
                     'dThrottle',1, ...
                     'dSteeringAngle',2, ...
                     'dBrakes',3, ...
                     'dVs',4, ...
                     'conTrack',1, ...
                     'conAlpha',2);

end

