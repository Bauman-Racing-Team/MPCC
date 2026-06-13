function boundedStates(obj)
    f = figure(3);
    f.Name = 'bounded states';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,obj.parameters.config.n+1,length(obj.log));

    for i = 1:length(obj.log)
        states(:,:,i) = obj.log(i).mpcHorizon.states;
    end

    tiledlayout(obj.config.NX-2,1); % yaw, vx, vy, r, throttle, steeringAngle, brakes, vs

    % yaw coordinate of all horizons

    yaw = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(1:obj.parameters.config.n+1,states(3,:,i));
    end

    ylim padded;
    yline(yaw,obj.parameters.bounds.lowerStateBounds.yawL,'--red','yaw'); % lower bound
    yline(yaw,obj.parameters.bounds.upperStateBounds.yawU,'--red','yaw'); % upper bound
    
    title(yaw,'yaw');
    ylabel(yaw,'yaw');
    
    % vx coordinate of all horizons

    vx = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(vx,1:obj.parameters.config.n+1,states(4,:,i));
    end

    ylim padded;
    yline(vx,obj.parameters.bounds.lowerStateBounds.vxL,'--red','vx'); % lower bound
    yline(vx,obj.parameters.bounds.upperStateBounds.vxU,'--red','vx'); % upper bound
    
    title(vx,'vx');
    ylabel(vx,'vx');

    % vy coordinate of all horizons

    vy = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(vy,1:obj.parameters.config.n+1,states(5,:,i));
    end

    ylim padded;
    yline(vy,obj.parameters.bounds.lowerStateBounds.vyL,'--red','vy'); % lower bound
    yline(vy,obj.parameters.bounds.upperStateBounds.vyU,'--red','vy'); % upper bound
    
    title(vy,'vy');
    ylabel(vy,'vy');

    % r coordinate of all horizons

    r = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(r,1:obj.parameters.config.n+1,states(6,:,i));
    end

    ylim padded;
    yline(r,obj.parameters.bounds.lowerStateBounds.rL,'--red','r'); % lower bound
    yline(r,obj.parameters.bounds.upperStateBounds.rU,'--red','r'); % upper bound
    
    title(r,'r');
    ylabel(r,'r');

    % s coordinate of all horizons

    s = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(s,1:obj.parameters.config.n+1,states(7,:,i));
    end

    ylim padded;
    yline(s,obj.parameters.bounds.lowerStateBounds.sL,'--red','s'); % lower bound
    yline(s,obj.parameters.bounds.upperStateBounds.sU,'--red','s'); % upper bound
    
    title(s,'s');
    ylabel(s,'s');

    % throttle coordinate of all horizons

    throttle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(throttle,1:obj.parameters.config.n+1,states(8,:,i));
    end

    ylim padded;
    yline(throttle,obj.parameters.bounds.lowerStateBounds.throttleL,'--red','throttle'); % lower bound
    yline(throttle,obj.parameters.bounds.upperStateBounds.throttleU,'--red','throttle'); % upper bound
    
    title(throttle,'throttle');
    ylabel(throttle,'throttle');

    % steeringAngle coordinate of all horizons

    steeringAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(steeringAngle,1:obj.parameters.config.n+1,states(9,:,i));
    end

    ylim padded;
    yline(steeringAngle,obj.parameters.bounds.lowerStateBounds.steeringAngleL,'--red','steeringAngle'); % lower bound
    yline(steeringAngle,obj.parameters.bounds.upperStateBounds.steeringAngleU,'--red','steeringAngle'); % upper bound
    
    title(steeringAngle,'steeringAngle');
    ylabel(steeringAngle,'steeringAngle');

    % brakes coordinate of all horizons

    brakes = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(brakes,1:obj.parameters.config.n+1,states(10,:,i));
    end

    ylim padded;
    yline(brakes,obj.parameters.bounds.lowerStateBounds.brakesL,'--red','brakes'); % lower bound
    yline(brakes,obj.parameters.bounds.upperStateBounds.brakesU,'--red','brakes'); % upper bound
    
    title(brakes,'brakes');
    ylabel(brakes,'brakes');

    % vs coordinate of all horizons

    vs = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(vs,1:obj.parameters.config.n+1,states(11,:,i));
    end

    ylim padded;
    yline(vs,obj.parameters.bounds.lowerStateBounds.vsL,'--red','vs'); % lower bound
    yline(vs,obj.parameters.bounds.upperStateBounds.vsU,'--red','vs'); % upper bound
    
    title(vs,'vs');
    ylabel(vs,'vs');
end

