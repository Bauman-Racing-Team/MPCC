function constrAndSlacks(obj)
    f = figure(7);
    f.Name = 'costs and slacks';
    f.NumberTitle = 'off';

    slacks = zeros(2*obj.config.NS,obj.config.N,length(obj.log)+1); % structure: [su; ... ; su; sl; ... ; sl]

    for i = 1:length(obj.log)
        slacks(1:obj.config.NS,:,i+1) = obj.log(i).mpcHorizon.slacks.upper;
        slacks(obj.config.NS+1:2*obj.config.NS,:,i+1) = obj.log(i).mpcHorizon.slacks.lower;
    end

%% slacks
%     figure;
%     tiledlayout(obj.config.NS-1,1); 

    % frontSlipAngle slack for upper bound

%     frontSlipAngleUpperSlack = nexttile;
%     hold on;

    slacks1 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks1(i)=slacks(1,1,i);
    end
%     plot(frontSlipAngleUpperSlack,slacks1);

%     ylim auto;
% 
%     title(frontSlipAngleUpperSlack,'frontSlipAngleUpperSlack');
%     ylabel(frontSlipAngleUpperSlack,'frontSlipAngleUpperSlack');

    % frontSlipAngle slack for lower bound

%     frontSlipAngleLowerSlack = nexttile;
%     hold on;

    slacks2 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks2(i)=slacks(obj.config.NS+1,1,i);
    end
%     plot(frontSlipAngleLowerSlack,slacks2);
% 
%     ylim auto;
% 
%     title(frontSlipAngleLowerSlack,'frontSlipAngleLowerSlack');
%     ylabel(frontSlipAngleLowerSlack,'frontSlipAngleLowerSlack');

    % rearSlipAngle slack for upper bound

%     rearSlipAngleUpperSlack = nexttile;
%     hold on;

    slacks3 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks3(i)=slacks(2,1,i);
    end
%     plot(rearSlipAngleUpperSlack,slacks3);
% 
%     ylim auto;
% 
%     title(rearSlipAngleUpperSlack,'rearSlipAngleUpperSlack');
%     ylabel(rearSlipAngleUpperSlack,'rearSlipAngleUpperSlack');

    % rearSlipAngle slack for lower bound

%     rearSlipAngleLowerSlack = nexttile;
%     hold on;

    slacks4 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks4(i)=slacks(obj.config.NS+2,1,i);
    end
%     plot(rearSlipAngleLowerSlack,slacks4);
% 
%     ylim auto;
% 
%     title(rearSlipAngleLowerSlack,'rearSlipAngleLowerSlack');
%     ylabel(rearSlipAngleLowerSlack,'rearSlipAngleLowerSlack');


%     figure;
%     tiledlayout(obj.config.NS+1,1); 
    % Track R slack for upper bound

%     trackRUpperSlack = nexttile;
%     hold on;

    slacks5 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks5(i)=slacks(3,1,i);
    end
%     plot(trackRUpperSlack,slacks5);
% 
%     ylim auto;
% 
%     title(trackRUpperSlack,'trackRUpperSlack');
%     ylabel(trackRUpperSlack,'trackRUpperSlack');

    % Track R slack for lower bound

%     trackRLowerSlack = nexttile;
%     hold on;

    slacks6 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks6(i)=slacks(obj.config.NS+3,1,i);
    end
%     plot(trackRLowerSlack,slacks6);

%     ylim auto;
% 
%     title(trackRLowerSlack,'trackRLowerSlack');
%     ylabel(trackRLowerSlack,'trackRLowerSlack');

    % front Friction ellipse slack for upper bound
    
%     frontFrictionUpperSlack= nexttile;
%     hold on;

    slacks7 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks7(i)=slacks(4,1,i);
    end
%     plot(frontFrictionUpperSlack,slacks7);
% 
%     ylim auto;
% 
%     title(frontFrictionUpperSlack,'frontFrictionUpperSlack');
%     ylabel(FrictionUpperSlack,'FrictionUpperSlack');

    % front Friction ellipse slack for upper bound
    
%     frontFrictionLowerSlack= nexttile;
%     hold on;

    slacks8 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks8(i)=slacks(obj.config.NS+4,1,i);
    end
%     plot(frontFrictionLowerSlack,slacks8);
% 
%     ylim auto;
% 
%     title(frontFrictionLowerSlack,'frontFrictionLowerSlack');
%     ylabel(FrictionLowerSlack,'FrictionLowerSlack');

    % rear Friction ellipse slack for upper bound
    
%     rearFrictionUpperSlack= nexttile;
%     hold on;

    slacks9 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks9(i)=sum(slacks(5,:,i))/100;
    end
%     plot(rearFrictionUpperSlack,slacks9);
% 
%     ylim auto;
% 
%     title(rearFrictionUpperSlack,'rearFrictionUpperSlack');
%     ylabel(FrictionUpperSlack,'FrictionUpperSlack');

    % front Friction ellipse slack for upper bound
    
%     rearFrictionLowerSlack= nexttile;
%     hold on;

    slacks10 = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
      slacks10(i)=slacks(obj.config.NS+5,1,i);
    end
%     plot(rearFrictionLowerSlack,slacks10);
% 
%     ylim auto;
% 
%     title(rearFrictionLowerSlack,'rearFrictionLowerSlack');
%     ylabel(FrictionLowerSlack,'FrictionLowerSlack');

%% constraints plus slacks
    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    tiledlayout(3,1); % steeringAngle, sideSlipAngle, kinematicSideSlipAngle, frontSlipAngle, rearSlipAngle

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
    hold on;
    ylim padded;
    yline(frontSlipAngle,-obj.parameters.mpcModel.maxAlpha,'--red','minAlpha'); % lower bound
    yline(frontSlipAngle,obj.parameters.mpcModel.maxAlpha,'--red','maxAlpha'); % upper bound
    plot(slacks1+obj.parameters.mpcModel.maxAlpha)
    plot(-slacks2-obj.parameters.mpcModel.maxAlpha)   

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
    hold on;
    ylim padded;
    yline(rearSlipAngle,-obj.parameters.mpcModel.maxAlpha,'--red','minAlpha'); % lower bound
    yline(rearSlipAngle,obj.parameters.mpcModel.maxAlpha,'--red','maxAlpha'); % upper bound
    plot(slacks3+obj.parameters.mpcModel.maxAlpha)
    plot(-slacks4-obj.parameters.mpcModel.maxAlpha)   
    
    title(rearSlipAngle,'rearSlipAngle');
    ylabel(rearSlipAngle,'rearSlipAngle');

    % Track error
    trackError = nexttile;
    x = states(1,:);
    y = states(2,:);
    xTrack = zeros(1,length(obj.log));
    yTrack = zeros(1,length(obj.log));
    for i = 1:length(obj.log)
        xTrack(i) = obj.log(i).circlesCenters(1,1);
        yTrack(i) = obj.log(i).circlesCenters(2,1);
    end
    trackErr = sqrt((x-xTrack).^2 + (y-yTrack).^2);

    plot(1:length(obj.log),trackErr);
    hold on;
%     yline(trackError,-obj.parameters.mpcModel.maxDistProj,'--red','minDistProj'); % lower bound
%     yline(trackError,obj.parameters.mpcModel.maxDistProj,'--red','maxDistProj'); % upper bound
    yline(trackError,-obj.parameters.mpcModel.rOut,'--red','minDistProj'); % lower bound
    yline(trackError,obj.parameters.mpcModel.rOut,'--red','maxDistProj'); % upper bound
    plot(trackError,slacks5+1)
    plot(trackError,-slacks6-1)   
        
    title(trackError,'trackError');
    axis ([0 length(obj.log) -obj.parameters.mpcModel.maxDistProj-0.5 obj.parameters.mpcModel.maxDistProj+0.5]);

    carModel = Model(obj.parameters.car,obj.parameters.tire);
    for i = 1:length(states)
        [constrF(i),constrR(i)] = carModel.initFrictionEllipseConstraint(states(:,i));
    end

    figure;
    subplot(2,1,1)
    plot(constrF);
    hold on;
    yline(1,'--red','maxFrontEllipse');
    yline(0,'--red','minFrontEllipse');
    plot(slacks7+1)
    plot(-slacks8)   

    subplot(2,1,2)
    plot(constrR);
    hold on;
    yline(1,'--red','maxRearEllipse'); 
    yline(0,'--red','minRearEllipse'); 
    plot(slacks9+1)
    plot(-slacks10)   

end