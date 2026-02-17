function stateVecs(obj)
    f = figure(9);
    f.Name = 'stateVecs';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    tiledlayout(11,1); % x, y, yaw, s, vx, vy, r, vs, throttle, steering, brakes

    % x coordinate

    x = nexttile;

    plot(x,1:length(obj.log),states(1,:));

    ylim padded;
    yline(x,obj.parameters.bounds.lowerStateBounds.xL,'--red','xMin'); % lower bound
    yline(x,obj.parameters.bounds.upperStateBounds.xU,'--red','xMax'); % upper bound

    title(x,'x');
    ylabel(x,'x');

    % y

    y = nexttile;

    plot(y,1:length(obj.log),states(2,:));

    ylim padded;
    yline(y,obj.parameters.bounds.lowerStateBounds.yL,'--red','yMin'); % lower bound
    yline(y,obj.parameters.bounds.upperStateBounds.yU,'--red','yMax'); % upper bound

    title(y,'y');
    ylabel(y,'y');

    % yaw

    yaw = nexttile;

    plot(yaw,1:length(obj.log),states(3,:));

    ylim padded;
    yline(yaw,obj.parameters.bounds.lowerStateBounds.yawL,'--red','yawMin'); % lower bound
    yline(yaw,obj.parameters.bounds.upperStateBounds.yawU,'--red','yawMax'); % upper bound

    title(yaw,'yaw');
    ylabel(yaw,'yaw');

    % vx

    vx = nexttile;

    plot(vx,1:length(obj.log),states(4,:));

    ylim padded;
    yline(vx,obj.parameters.bounds.lowerStateBounds.vxL,'--red','vxMin'); % lower bound
    yline(vx,obj.parameters.bounds.upperStateBounds.vxU,'--red','vxMax'); % upper bound

    title(vx,'vx');
    ylabel(vx,'vx');

    % vy

    vy = nexttile;

    plot(vy,1:length(obj.log),states(5,:));

    ylim padded;
    yline(vy,obj.parameters.bounds.lowerStateBounds.vyL,'--red','vyMin'); % lower bound
    yline(vy,obj.parameters.bounds.upperStateBounds.vyU,'--red','vyMax'); % upper bound

    title(vy,'vy');
    ylabel(vy,'vy');

    % r

    r = nexttile;

    plot(r,1:length(obj.log),states(6,:));

    ylim padded;
    yline(r,obj.parameters.bounds.lowerStateBounds.rL,'--red','rMin'); % lower bound
    yline(r,obj.parameters.bounds.upperStateBounds.rU,'--red','rMax'); % upper bound

    title(r,'r');
    ylabel(r,'r');

    % s

    s = nexttile;
    
    plot(s,1:length(obj.log),states(7,:));

    ylim padded;
    yline(s,obj.parameters.bounds.lowerStateBounds.sL,'--red','sMin'); % lower bound
    yline(s,obj.parameters.bounds.upperStateBounds.sU,'--red','sMax'); % upper bound
    
    title(s,'s');
    ylabel(s,'s');

    % throttle

    throttle = nexttile;

    plot(throttle,1:length(obj.log),states(8,:));

    ylim padded;
    yline(throttle,obj.parameters.bounds.lowerStateBounds.throttleL,'--red','throttleMin'); % lower bound
    yline(throttle,obj.parameters.bounds.upperStateBounds.throttleU,'--red','throttleMax'); % upper bound

    title(throttle,'throttle');
    ylabel(throttle,'throttle');

    % steeringAngle

    steeringAngle = nexttile;

    plot(steeringAngle,1:length(obj.log),states(9,:));

    ylim padded;
    yline(steeringAngle,obj.parameters.bounds.lowerStateBounds.steeringAngleL,'--red','steeringAngleMin'); % lower bound
    yline(steeringAngle,obj.parameters.bounds.upperStateBounds.steeringAngleU,'--red','steeringAngleMax'); % upper bound

    title(steeringAngle,'steeringAngle');
    ylabel(steeringAngle,'steeringAngle');

    % brakes

    brakes = nexttile;

    plot(brakes,1:length(obj.log),states(10,:));

    ylim padded;
    yline(brakes,obj.parameters.bounds.lowerStateBounds.brakesL,'--red','brakesMin'); % lower bound
    yline(brakes,obj.parameters.bounds.upperStateBounds.brakesU,'--red','brakesMax'); % upper bound

    title(brakes,'brakes');
    ylabel(brakes,'brakes');

    % vs

    vs = nexttile;

    plot(vs,1:length(obj.log),states(11,:));

    ylim padded;
    yline(vs,obj.parameters.bounds.lowerStateBounds.vsL,'--red','vsMin'); % lower bound
    yline(vs,obj.parameters.bounds.upperStateBounds.vsU,'--red','vsMax'); % upper bound

    title(vs,'vs');
    ylabel(vs,'vs');
end

