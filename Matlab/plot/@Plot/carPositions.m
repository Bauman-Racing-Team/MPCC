function carPositions(obj)
    f = figure(1);
    f.Name = 'carPositions';
    f.NumberTitle = 'off';

    hold on;

    plot(obj.track.xOuter,obj.track.yOuter,'black');
    plot(obj.track.xInner,obj.track.yInner,'black');
    plot(obj.track.x,obj.track.y,'r');

    states = zeros(obj.config.NX,obj.parameters.config.n+1,length(obj.log));

    for i = 1:length(obj.log)
        states(:,:,i) = obj.log(i).mpcHorizon.states;
    end

    for i = 1:length(obj.log)
        plot(states(1,1:obj.parameters.config.n-10,i),states(2,1:obj.parameters.config.n-10,i));
    end

end

