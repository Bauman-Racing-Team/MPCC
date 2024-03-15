function race(obj)
    f = figure(5);
    f.Name = 'race';
    f.NumberTitle = 'off';

    hold on;
    axis equal;
    plot(obj.track.xOuter,obj.track.yOuter,'black');
    plot(obj.track.xInner,obj.track.yInner,'black');
    
    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    horizonsPositions = zeros(2,obj.config.N+1,length(obj.log));

    for i = 1:length(obj.log)
        horizonsPositions(:,:,i) = obj.log(i).mpcHorizon.states(1:2,:);
    end

    circlesCenters = zeros(2,obj.config.N+1,length(obj.log));

    for i = 1:length(obj.log)
        circlesCenters(:,:,i) = obj.log(i).circlesCenters(:,:);
    end

    for i = 1:length(obj.log)
        carBox = plotCarBox(states(:,i),obj.parameters.car.carW,obj.parameters.car.carL);
        horizonPositions = plotHorizonPositions(horizonsPositions(:,:,i));
        %circles = plotCircleConstraint(circlesCenters(:,:,i),obj.parameters.mpcModel.rOut);
        pause(0.05)
        %exportgraphics(gca,"race_FSG_track.gif","Append",true);
        delete(carBox);
        delete(horizonPositions);
        %delete(circles);
    end

    plot(states(1,:),states(2,:),"green");
    %exportgraphics(gca,"race_FSG_track.gif","Append",true);
    pause(5.00)
end


function carBox = plotCarBox(x0,w,l)
        car1 = x0(1:2) + [cos(x0(3))*l;sin(x0(3))*l] + [sin(x0(3))*w;-cos(x0(3))*w];
        car2 = x0(1:2) + [cos(x0(3))*l;sin(x0(3))*l] - [sin(x0(3))*w;-cos(x0(3))*w];
        car3 = x0(1:2) - [cos(x0(3))*l;sin(x0(3))*l] + [sin(x0(3))*w;-cos(x0(3))*w];
        car4 = x0(1:2) - [cos(x0(3))*l;sin(x0(3))*l] - [sin(x0(3))*w;-cos(x0(3))*w];

        carBox = plot([car1(1),car2(1),car4(1),car3(1),car1(1)],[car1(2),car2(2),car4(2),car3(2),car1(2)],'blue','LineWidth',0.5);
        axis equal;
end

function horizonPositions = plotHorizonPositions(horizonsPositions)
    horizonPositions = plot(horizonsPositions(1,:),horizonsPositions(2,:),'red');
end

function circles = plotCircleConstraint(circleCenters,r)
    circles = zeros(1,length(circleCenters));
    theta = linspace(0,2*pi);

    for i = 1:length(circleCenters)
        xc = circleCenters(1,i);
        yc = circleCenters(2,i);

        x = r*cos(theta)+xc;
        y = r*sin(theta)+yc;

        circles(1,i) = plot(x,y,'magenta');
    end
end

