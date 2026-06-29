function race(obj)
    f = figure(5);
    f.Name = 'race';
    f.NumberTitle = 'off';

    hold on;
    axis equal;
    plot(obj.track.xOuter,obj.track.yOuter,'blue');
    plot(obj.track.xInner,obj.track.yInner,'blue');
    
    states = zeros(obj.config.NX,length(obj.log));

    for i = 1:length(obj.log)
        states(:,i) = obj.log(i).mpcHorizon.states(:,1);
    end

    horizonsPositions = zeros(2,obj.parameters.config.n+1,length(obj.log));

    for i = 1:length(obj.log)
        horizonsPositions(:,:,i) = obj.log(i).mpcHorizon.states(1:2,:);
    end

    bordersCoordinates = zeros(length(obj.log), 4);

    for i = 1:length(obj.log)
       bordersCoordinates(i,:) = obj.log(i).bordersCoordinates;
    end

    colors = ["red", "green", "cyan", "magenta", "yellow", "white"];
    
    for i = 1:length(obj.log)
        carBox = plotCarBox(states(:,i),obj.parameters.car.carW,obj.parameters.car.carL);
        horizonPositions = plotHorizonPositions(horizonsPositions(:,:,i));
        pause(0.05)
        %exportgraphics(gca,"race_FSG_track.gif","Append",true);
        color = colors(mod(i,length(colors))+1);
        circle1 = plotCircle([bordersCoordinates(i, 1), bordersCoordinates(i, 2)], 0.1, color);
        circle2 = plotCircle([bordersCoordinates(i, 3), bordersCoordinates(i, 4)], 0.1, color);
        delete(horizonPositions);
        %delete(carBox);
    end

    plot(states(1,:),states(2,:),"green");
    %exportgraphics(gca,"race_FSG_track.gif","Append",true);
    pause(5.00)
end


function carBox = plotCarBox(x0,w,l)
        w = w/2;
        l = l/2;
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

function circle = plotCircle(circleCenter, r, color)
    theta = linspace(0,2*pi);
    xc = circleCenter(1);
    yc = circleCenter(2);
    x = r*cos(theta)+xc;
    y = r*sin(theta)+yc;
    circle = plot(x,y, color);
end

