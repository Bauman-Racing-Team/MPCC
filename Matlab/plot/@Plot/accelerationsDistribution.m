function accelerationsDistribution(obj)
    f = figure(6);
    f.Name = 'accelerations';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    mat = zeros(length(obj.log)-1,2);
    
    for i = 1:length(obj.log)-1
        mat(i,1) = (obj.log(i+1).mpcHorizon.states(4) - obj.log(i).mpcHorizon.states(4))/obj.config.Ts;
        mat(i,2) = (obj.log(i+1).mpcHorizon.states(5) - obj.log(i).mpcHorizon.states(5))/obj.config.Ts;
    end

    plot(mat(:,1),mat(:,2),'.red','Marker','.');
    
end

