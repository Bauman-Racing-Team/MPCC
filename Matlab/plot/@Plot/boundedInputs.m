function boundedInputs(obj)
    f = figure(4);
    f.Name = 'bounded inputs';
    f.NumberTitle = 'off';

    inputs = zeros(obj.config.NU,obj.parameters.config.n,length(obj.log));

    for i = 1:length(obj.log)
        inputs(:,:,i) = obj.log(i).mpcHorizon.inputs;
    end

    tiledlayout(obj.config.NU,1);

    % dThrottle coordinate of all horizons

    dThrottle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(dThrottle,1:obj.parameters.config.n,inputs(1,:,i));
    end
    
    ylim padded;
    yline(dThrottle,obj.parameters.bounds.lowerInputBounds.dThrottleL,'--red','dThrottleL'); % lower bound
    yline(dThrottle,obj.parameters.bounds.upperInputBounds.dThrottleU,'--red','dThrottleU'); % upper bound

    title(dThrottle,'dThrottle');
    ylabel(dThrottle,'dThrottle');

    % dSteeringAngle coordinate of all horizons

    dSteeringAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(dSteeringAngle,1:obj.parameters.config.n,inputs(2,:,i));
    end

    ylim padded;
    yline(dSteeringAngle,obj.parameters.bounds.lowerInputBounds.dSteeringAngleL,'--red','dSteeringAngleL'); % lower bound
    yline(dSteeringAngle,obj.parameters.bounds.upperInputBounds.dSteeringAngleU,'--red','dSteeringAngleU'); % upper bound
    
    title(dSteeringAngle,'dSteeringAngle');
    ylabel(dSteeringAngle,'dSteeringAngle');

    % dBrakes coordinate of all horizons

    dBrakes = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(dBrakes,1:obj.parameters.config.n,inputs(3,:,i));
    end

    ylim padded;
    yline(dBrakes,obj.parameters.bounds.lowerInputBounds.dBrakesL,'--red','dBrakesL'); % lower bound
    yline(dBrakes,obj.parameters.bounds.upperInputBounds.dBrakesU,'--red','dBrakesU'); % upper bound
    
    title(dBrakes,'dBrakes');
    ylabel(dBrakes,'dBrakes');

    % dVs coordinate of all horizons

    dVs = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(dVs,1:obj.parameters.config.n,inputs(4,:,i));
    end

    ylim padded;
    yline(dVs,obj.parameters.bounds.lowerInputBounds.dVsL,'--red','dVsL'); % lower bound
    yline(dVs,obj.parameters.bounds.upperInputBounds.dVsU,'--red','dVsU'); % upper bound
    
    title(dVs,'dVs');
    ylabel(dVs,'dVs');
end

