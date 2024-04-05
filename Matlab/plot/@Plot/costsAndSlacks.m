function costsAndSlacks(obj)
    f = figure(7);
    f.Name = 'costs and slacks';
    f.NumberTitle = 'off';

    slacks = zeros(2*obj.config.NS,obj.config.N,length(obj.log)); % structure: [su; ... ; su; sl; ... ; sl]

    for i = 1:length(obj.log)
        slacks(1:obj.config.NS,:,i) = obj.log(i).mpcHorizon.slacks.upper;
        slacks(obj.config.NS+1:2*obj.config.NS,:,i) = obj.log(i).mpcHorizon.slacks.lower;
    end

%     tiledlayout(2*obj.config.NS+1,1); % cost, upper and lower slacks on all soft constraints
    tiledlayout(1,1); 
    % cost values on all iterations

    cost = nexttile;
    hold on;

    costs = zeros(1,length(obj.log));

    for i = 1:length(obj.log)
        costs(i) = obj.log(i).cost;
    end

    plot(cost,1:length(obj.log),costs);
    ylim auto;

    title(cost,'cost');
    ylabel(cost,'cost');

    figure;
    tiledlayout(obj.config.NS,1); 
    % frontSlipAngle slack for upper bound

    frontSlipAngleUpperSlack = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(frontSlipAngleUpperSlack,1:obj.config.N,slacks(1,:,i));
    end

    ylim auto;

    title(frontSlipAngleUpperSlack,'frontSlipAngleUpperSlack');
    ylabel(frontSlipAngleUpperSlack,'frontSlipAngleUpperSlack');

    % frontSlipAngle slack for lower bound

    frontSlipAngleLowerSlack = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(frontSlipAngleLowerSlack,1:obj.config.N,slacks(obj.config.NS+1,:,i));
    end

    ylim auto;

    title(frontSlipAngleLowerSlack,'frontSlipAngleLowerSlack');
    ylabel(frontSlipAngleLowerSlack,'frontSlipAngleLowerSlack');

    % rearSlipAngle slack for upper bound

    rearSlipAngleUpperSlack = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(rearSlipAngleUpperSlack,1:obj.config.N,slacks(2,:,i));
    end

    ylim auto;

    title(rearSlipAngleUpperSlack,'rearSlipAngleUpperSlack');
    ylabel(rearSlipAngleUpperSlack,'rearSlipAngleUpperSlack');

    % rearSlipAngle slack for lower bound

    rearSlipAngleLowerSlack = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(rearSlipAngleLowerSlack,1:obj.config.N,slacks(obj.config.NS+2,:,i));
    end

    ylim auto;

    title(rearSlipAngleLowerSlack,'rearSlipAngleLowerSlack');
    ylabel(rearSlipAngleLowerSlack,'rearSlipAngleLowerSlack');


    figure;
    tiledlayout(obj.config.NS,1); 
    % Track R slack for upper bound

    trackRUpperSlack = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(trackRUpperSlack,1:obj.config.N,slacks(3,:,i));
    end

    ylim auto;

    title(trackRUpperSlack,'trackRUpperSlack');
    ylabel(trackRUpperSlack,'trackRUpperSlack');

    figure;
    tiledlayout(obj.config.NS,1); 
    
    % Track R slack for lower bound

    trackRLowerSlack = nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(trackRLowerSlack,1:obj.config.N,slacks(obj.config.NS+3,:,i));
    end

    ylim auto;

    title(trackRLowerSlack,'trackRLowerSlack');
    ylabel(trackRLowerSlack,'trackRLowerSlack');


    %Front Friction ellipse slack for lower bound
    
    frontFrictionUpperSlack= nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(frontFrictionUpperSlack,1:obj.config.N,slacks(4,:,i));
    end

    ylim auto;

    title(frontFrictionUpperSlack,'frontFrictionUpperSlack');
    ylabel(frontFrictionUpperSlack,'frontFrictionUpperSlack');

    % Front Friction ellipse slack for lower bound
    
    frontFrictionLowerSlack= nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(frontFrictionLowerSlack,1:obj.config.N,slacks(obj.config.NS+4,:,i));
    end

    ylim auto;

    title(frontFrictionLowerSlack,'frontFrictionLowerSlack');
    ylabel(frontFrictionLowerSlack,'frontFrictionLowerSlack');

    %Rear Friction ellipse slack for upper bound
    
    rearFrictionUpperSlack= nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(rearFrictionUpperSlack,1:obj.config.N,slacks(5,:,i));
    end

    ylim auto;

    title(rearFrictionUpperSlack,'rearFrictionUpperSlack');
    ylabel(rearFrictionUpperSlack,'rearFrictionUpperSlack');

    % Friction ellipse slack for upper bound
    
    rearFrictionLowerSlack= nexttile;
    hold on;

    for i = 1:length(obj.log)
        plot(rearFrictionLowerSlack,1:obj.config.N,slacks(obj.config.NS+5,:,i));
    end

    ylim auto;

    title(rearFrictionLowerSlack,'rearFrictionLowerSlack');
    ylabel(rearFrictionLowerSlack,'rearFrictionLowerSlack');

end