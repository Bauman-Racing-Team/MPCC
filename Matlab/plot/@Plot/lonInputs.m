function lonInputs(obj)
    f = figure(10);
    f.Name = 'lonInputs';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    tiledlayout(2,1); % throttle, brakes

    % throttle

    throttle = nexttile;
    
    plot(throttle,1:length(obj.log),states(8,:));

    ylim padded;
    yline(throttle,obj.parameters.bounds.lowerStateBounds.throttleL,'--red','throttle'); % lower bound
    yline(throttle,obj.parameters.bounds.upperStateBounds.throttleU,'--red','throttle'); % upper bound
    
    title(throttle,'throttle');
    ylabel(throttle,'throttle');

    % brakes

    brakes = nexttile;
    
    plot(brakes,1:length(obj.log),states(10,:));

    ylim padded;
    yline(brakes,obj.parameters.bounds.lowerStateBounds.brakesL,'--red','brakes'); % lower bound
    yline(brakes,obj.parameters.bounds.upperStateBounds.brakesU,'--red','brakes'); % upper bound
    
    title(brakes,'brakes');
    ylabel(brakes,'brakes');
end