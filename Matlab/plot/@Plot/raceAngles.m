function raceAngles(obj)
    f = figure(8);
    f.Name = 'raceAngles';
    f.NumberTitle = 'off';

    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    tiledlayout(5,1); % steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle

    % steeringAngle coordinate

    steeringAngle = nexttile;
    
    plot(steeringAngle,1:length(obj.log),states(9,:));

    ylim padded;
    yline(steeringAngle,obj.parameters.bounds.lowerStateBounds.steeringAngleL,'--red','steeringAngleL'); % lower bound
    yline(steeringAngle,obj.parameters.bounds.upperStateBounds.steeringAngleU,'--red','steeringAngleU'); % upper bound
    
    title(steeringAngle,'steeringAngle');
    ylabel(steeringAngle,'steeringAngle');

    % sideSlipAngle

    sideSlipAngle = nexttile;

    sideSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
        vx = states(4,i);
        vy = states(5,i);
        sideSlipAngles(i) = atan2(vy,vx);
    end

    plot(sideSlipAngle,1:length(obj.log),sideSlipAngles);

    ylim padded;
    
    title(sideSlipAngle,'sideSlipAngle');
    ylabel(sideSlipAngle,'sideSlipAngle');

    % kinematicSideSlipAngle

    kinematicSideSlipAngle = nexttile;

    kinematicSideSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
            vx = states(4,i);
            l = obj.parameters.car.lf+obj.parameters.car.lr;
            steeringAngle = states(9,i);
            kinematicSideSlipAngles(i) = vx*tan(steeringAngle)/l;
    end

    plot(kinematicSideSlipAngle,1:length(obj.log),kinematicSideSlipAngles);

    ylim padded;
    
    title(kinematicSideSlipAngle,'kinematicSideSlipAngle');
    ylabel(kinematicSideSlipAngle,'kinematicSideSlipAngle');

    % frontSlipAngle

    frontSlipAngle = nexttile;

    frontSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
            vx = states(4,i);
            vy = states(5,i);
            r = states(6,i);
            steeringAngle = states(9,i);
            lf = obj.parameters.car.lf;
            frontSlipAngles(i) = atan2(vy+r*lf,vx)-steeringAngle;
    end

    plot(frontSlipAngle,1:length(obj.log),frontSlipAngles);

    ylim padded;
    yline(frontSlipAngle,-obj.parameters.mpcModel.maxAlpha,'--red','minAlpha'); % lower bound
    yline(frontSlipAngle,obj.parameters.mpcModel.maxAlpha,'--red','maxAlpha'); % upper bound
    
    title(frontSlipAngle,'frontSlipAngle');
    ylabel(frontSlipAngle,'frontSlipAngle');

    % rearSlipAngle

    rearSlipAngle = nexttile;

    rearSlipAngles = zeros(length(obj.log),1);
    
    for i = 1:length(obj.log)
            vx = states(4,i);
            vy = states(5,i);
            r = states(6,i);
            lr = obj.parameters.car.lr;
            rearSlipAngles(i) = atan2(vy-r*lr,vx);
    end

    plot(rearSlipAngle,1:length(obj.log),rearSlipAngles);

    ylim padded;
    yline(rearSlipAngle,-obj.parameters.mpcModel.maxAlpha,'--red','minAlpha'); % lower bound
    yline(rearSlipAngle,obj.parameters.mpcModel.maxAlpha,'--red','maxAlpha'); % upper bound
    
    title(rearSlipAngle,'rearSlipAngle');
    ylabel(rearSlipAngle,'rearSlipAngle');

    % Track error
    x = states(1,:);
    y = states(2,:);
    xTrack = zeros(1,length(obj.log));
    yTrack = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
        xTrack(i) = obj.log(i).circlesCenters(1,1);
        yTrack(i) = obj.log(i).circlesCenters(2,1);
    end
    trackError = sqrt((x-xTrack).^2 + (y-yTrack).^2);
    figure;
    plot(1:length(obj.log),trackError);

    yline(-obj.parameters.mpcModel.maxDistProj,'--red','minDistProj'); % lower bound
    yline(obj.parameters.mpcModel.maxDistProj,'--red','maxDistProj'); % upper bound
    yline(-obj.parameters.mpcModel.rOut,'--red','minROut'); % lower bound
    yline(obj.parameters.mpcModel.rOut,'--red','maxROut'); % upper bound
    axis ([0 length(obj.log) -obj.parameters.mpcModel.maxDistProj-0.5 obj.parameters.mpcModel.maxDistProj+0.5]);

    % Ellipse

    Frx = zeros(length(obj.log),1);
    Fry = zeros(length(obj.log),1);
    Ffx = zeros(length(obj.log),1);
    Ffy = zeros(length(obj.log),1);
    for i = 1:length(obj.log)
            vx = states(4,i);
            vy = states(5,i);
            r = states(6,i);
            throttle = states(8,i);
            steeringAngle = states(9,i);
            brakes = states(10,i);
            lr = obj.parameters.car.lr;
            rearSlipAngles(i) = atan2(vy-r*lr,vx);

    m = obj.parameters.car.m;
    gAcc = obj.parameters.car.g;
    cbf=obj.parameters.car.cbf;
    rDyn = obj.parameters.car.rDyn;
    sar = atan2((vy-r*lr),vx);

    Fry(i) = -sar*obj.parameters.tire.Cy;

    Ffz = lr*m*gAcc/(2.0*(lf+lr));
    saf = atan2((vy+r*lf),vx)-steeringAngle;

    Ffx(i) = (-cbf*brakes)/rDyn*tanh(vx)+2*obj.parameters.tire.QSY1*Ffz*tanh(vx);
  
    Ffy(i) = -saf*obj.parameters.tire.Cy;

    end

    figure;
    subplot(1,2,1);
    hold on;
    axis equal;
    equation = @(x, y) (x/obj.parameters.car.muxFz).^2 + (y/obj.parameters.car.muyFz).^2 - 1;
    ezplot(equation, [-obj.parameters.car.muxFz, obj.parameters.car.muxFz, -obj.parameters.car.muyFz, obj.parameters.car.muyFz]);
    h = get(gca, 'Children');
    set(h, 'Color', 'r','LineStyle','--');
    plot(Ffx,Ffy,'.')
    title('Tire force of front force');
    xlabel('Ffx');
    ylabel('Ffy');

    subplot(1,2,2);
    hold on;
    axis equal;
    equation = @(x, y) (x/obj.parameters.car.muxFz).^2 + (y/obj.parameters.car.muyFz).^2 - 1;
    ezplot(equation, [-obj.parameters.car.muxFz, obj.parameters.car.muxFz, -obj.parameters.car.muyFz, obj.parameters.car.muyFz]);
    h = get(gca, 'Children');
    set(h, 'Color', 'r','LineStyle','--');
    plot(Frx,Fry,'.')

%     plot(rearSlipAngle,1:length(obj.log),rearSlipAngles);

%     ylim padded;

    title('Tire force of rear force');
    xlabel('Frx');
    ylabel('Fry');


end



