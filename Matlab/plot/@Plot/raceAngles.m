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
    fzNominal = obj.parameters.car.fzNominal;
    cbr=obj.parameters.car.cbr;
    cbf=obj.parameters.car.cbf;
    rDyn = obj.parameters.car.rDyn;
    cdrv = obj.parameters.car.cm1 * obj.parameters.car.gearRatio;
    
    Frz = lf*m*gAcc/(2.0*(lf+lr));
    Drfz = (Frz-fzNominal)/fzNominal;
    Kry = obj.parameters.tire.PKY1*fzNominal * sin(2.0*atan2(Frz,(obj.parameters.tire.PKY2*fzNominal*obj.parameters.tire.LFZO)))*obj.parameters.tire.LFZO*obj.parameters.tire.LKY;
    Dry = (obj.parameters.tire.PDY1+obj.parameters.tire.PDY2*Drfz)*obj.parameters.tire.LMUY*Frz;
    Cry = obj.parameters.tire.PCY1*obj.parameters.tire.LCY;
    Bry = Kry/(Cry*Dry);
    Ery = (obj.parameters.tire.PEY1+obj.parameters.tire.PEY2*Drfz)*obj.parameters.tire.LEY;
    sar = atan2((vy-r*lr),vx);
    
    Frx(i) = (-cbr*brakes)/rDyn*tanh(vx)+(cdrv*throttle)/rDyn+2*obj.parameters.tire.QSY1*Frz*tanh(vx);
%     Fry(i) = 2*Dry*sin(Cry*atan(Bry*sar-Ery*(Bry*sar-atan(Bry*sar))));
    Fry(i) = -sar*29117;

    Ffz = lr*m*gAcc/(2.0*(lf+lr));
    Dffz = (Ffz-fzNominal)/fzNominal;
    Kfy = obj.parameters.tire.PKY1*fzNominal * sin(2.0*atan2(Ffz,(obj.parameters.tire.PKY2*fzNominal*obj.parameters.tire.LFZO)))*obj.parameters.tire.LFZO*obj.parameters.tire.LKY;        
    mufy = (obj.parameters.tire.PDY1+obj.parameters.tire.PDY2*Dffz)*obj.parameters.tire.LMUY;
    Dfy = mufy*Ffz;
    Cfy = obj.parameters.tire.PCY1*obj.parameters.tire.LCY;
    Bfy = Kfy/(Cfy*Dfy);
    Efy = (obj.parameters.tire.PEY1+obj.parameters.tire.PEY2*Dffz)*obj.parameters.tire.LEY;
    saf = atan2((vy+r*lf),vx)-steeringAngle;
    
%     Ffy(i) = 2*Dfy*sin(Cfy*atan(Bfy*saf-Efy*(Bfy*saf-atan(Bfy*saf))));
    Ffx(i) = (-cbf*brakes)/rDyn*tanh(vx)+2*obj.parameters.tire.QSY1*Ffz*tanh(vx);
  
    Ffy(i) = -saf*29117;

    end
    
    figure;
    hold on;
    equation = @(x, y) (x/2118.04).^2 + (y/2030.5).^2 - 1;
    ezplot(equation, [-2118.04, 2118.04, -2030.5, 2030.5]);
    h = get(gca, 'Children');
    set(h, 'Color', 'r','LineStyle','--');
    plot(Ffx,Ffy,'.')
    title('Tire force of front force');
    xlabel('Ffx');
    ylabel('Ffy');

    figure;
    hold on;
    equation = @(x, y) (x/2118.04).^2 + (y/2030.5).^2 - 1;
    fimplicit(equation, [-2118.04, 2118.04, -2030.5, 2030.5]);
    h = get(gca, 'Children');
    set(h, 'Color', 'r','LineStyle','--');
    plot(Frx,Fry,'.')

%     plot(rearSlipAngle,1:length(obj.log),rearSlipAngles);

%     ylim padded;

    title('Tire force of rear force');
    xlabel('Frx');
    ylabel('Fry');

end

