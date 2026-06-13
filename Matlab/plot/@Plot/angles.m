function angles(obj)
    f = figure(2);
    f.Name = 'angles';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,obj.parameters.config.n+1,length(obj.log));

    for i = 1:length(obj.log)
        states(:,:,i) = obj.log(i).mpcHorizon.states;
    end

    tiledlayout(5,1); % steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle

    % steeringAngle coordinate of all horizons

    steeringAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        plot(steeringAngle,1:obj.parameters.config.n+1,states(9,:,i));
    end

    ylim padded;
    yline(steeringAngle,obj.parameters.bounds.lowerStateBounds.steeringAngleL,'--red','steeringAngleL'); % lower bound
    yline(steeringAngle,obj.parameters.bounds.upperStateBounds.steeringAngleU,'--red','steeringAngleU'); % upper bound
    
    title(steeringAngle,'steeringAngle');
    ylabel(steeringAngle,'steeringAngle');

    % sideSlipAngle of all horizons

    sideSlipAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        sideSlipAngles = zeros(obj.parameters.config.n+1,1);

        for j = 1:obj.parameters.config.n+1

            vx = states(4,j,i);
            vy = states(5,j,i);

            sideSlipAngles(j) = atan2(vy,vx);
        end

        plot(sideSlipAngle,1:obj.parameters.config.n+1,sideSlipAngles);
    end

    ylim padded;
    
    title(sideSlipAngle,'sideSlipAngle');
    ylabel(sideSlipAngle,'sideSlipAngle');

    % kinematicSideSlipAngle of all horizons

    kinematicSideSlipAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        kinematicSideSlipAngles = zeros(obj.parameters.config.n+1,1);

        for j = 1:obj.parameters.config.n+1

            vx = states(4,j,i);
            l = obj.parameters.car.lf+obj.parameters.car.lr;
            steeringAngle = states(9,j,i);

            kinematicSideSlipAngles(j) = vx*tan(steeringAngle)/l;
        end

        plot(kinematicSideSlipAngle,1:obj.parameters.config.n+1,kinematicSideSlipAngles);
    end

    ylim padded;
    
    title(kinematicSideSlipAngle,'kinematicSideSlipAngle');
    ylabel(kinematicSideSlipAngle,'kinematicSideSlipAngle');

    % frontSlipAngle of all horizons

    frontSlipAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        frontSlipAngles = zeros(obj.parameters.config.n+1,1);

        for j = 1:obj.parameters.config.n+1

            vx = states(4,j,i);
            vy = states(5,j,i);
            r = states(6,j,i);
            steeringAngle = states(9,j,i);

            lf = obj.parameters.car.lf;

            frontSlipAngles(j) = atan2(vy+r*lf,vx)-steeringAngle;
        end

        plot(frontSlipAngle,1:obj.parameters.config.n+1,frontSlipAngles);
    end

    ylim padded;
    yline(frontSlipAngle,obj.parameters.bounds.lowerConstraintBounds.maxAlphaFrontL,'--red','maxAlphaFrontL'); % lower bound
    yline(frontSlipAngle,obj.parameters.bounds.upperConstraintBounds.maxAlphaFrontU,'--red','maxAlphaFrontU'); % upper bound
    
    title(frontSlipAngle,'frontSlipAngle');
    ylabel(frontSlipAngle,'frontSlipAngle');

    % rearSlipAngle of all horizons

    rearSlipAngle = nexttile;
    hold on;
    
    for i = 1:length(obj.log)
        rearSlipAngles = zeros(obj.parameters.config.n+1,1);

        for j = 1:obj.parameters.config.n+1

            vx = states(4,j,i);
            vy = states(5,j,i);
            r = states(6,j,i);

            lr = obj.parameters.car.lr;

            rearSlipAngles(j) = atan2(vy-r*lr,vx);
        end

        plot(rearSlipAngle,1:obj.parameters.config.n+1,rearSlipAngles);
    end

    ylim padded;
    yline(rearSlipAngle,obj.parameters.bounds.lowerConstraintBounds.maxAlphaRearL,'--red','maxAlphaRearL'); % lower bound
    yline(rearSlipAngle,obj.parameters.bounds.upperConstraintBounds.maxAlphaRearU,'--red','maxAlphaRearU'); % upper bound
    
    title(rearSlipAngle,'rearSlipAngle');
    ylabel(rearSlipAngle,'rearSlipAngle');
end