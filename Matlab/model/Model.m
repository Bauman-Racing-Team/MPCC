classdef Model
    properties (Access = private)
        car
        tire
    end
    
    methods (Access = public)

        function obj = Model(car,tire)
            obj.car = car;
            obj.tire = tire;
        end

    function [Ffx,Ffy,Frx,Fry] = initSimpleFrictionEllipseConstraint(obj,states) 
                    vx = states(4);
                    vy = states(5);
                    r = states(6);
                    throttle = states(8);
                    steeringAngle = states(9);
                    brakes = states(10);
       
                    %Dynamic forces
                    rDyn = obj.car.rDyn;
                    
                    cdrv = obj.car.cm1 * obj.car.gearRatio;
                    cbf = obj.car.cbf;
                    cbr = obj.car.cbr;
        
                    m = obj.car.m;
                    lf = obj.car.lf;
                    lr = obj.car.lr;
                    gAcc = obj.car.g;

                    % normal load on the one front wheel
                    Ffz = lr*m*gAcc/(2.0*(lf+lr));
        
                    % normal load on the one rear wheel
                    Frz = lf*m*gAcc/(2.0*(lf+lr));
                    
                    % rolling resistance of the two front wheels
                    Ffrr = 2*obj.tire.QSY1*Ffz*tanh(vx);
        
                    % rolling resistance of the two rear wheels
                    Frrr = 2*obj.tire.QSY1*Frz*tanh(vx);
                    
                    % brakes front force
                    Fbf = (-cbf*brakes)/rDyn*tanh(vx);
        
                    % brakes rear force
                    Fbr = (-cbr*brakes)/rDyn*tanh(vx);
        
                    % drivetrain force
                    Fdrv = (cdrv*throttle)/rDyn;
        
                    % longitudinal front force
                    Ffx = Fbf+Ffrr;
                    
                    % longitudinal rear force
                    Frx = Fbr+Fdrv+Frrr;
                    saf = atan2((vy+r*lf),vx)-steeringAngle;
        
                    % slip angle of the rear wheel
                    sar = atan2((vy-r*lr),vx);
        
                    % latteral front force
                    Ffy = -saf*obj.tire.Cy;
                    % latteral rear force
                    Fry = -sar*obj.tire.Cy;
 
%                     constrF = (Ffx/obj.car.muxFz)^2+(Ffy/obj.car.muyFz)^2;
%                     constrR = (Frx/obj.car.muxFz)^2+(Fry/obj.car.muyFz)^2;
        end
        
        function [Ffx,Ffy,Frx,Fry] = initFrictionEllipseConstraint(obj, states)
                    vx = states(4);
                    vy = states(5);
                    r = states(6);
                    throttle = states(8);
                    steeringAngle = states(9);
                    brakes = states(10);
       
                    %Dynamic forces
                    rDyn = obj.car.rDyn;
                    
                    cdrv = obj.car.cm1 * obj.car.gearRatio;
                    cbf = obj.car.cbf;
                    cbr = obj.car.cbr;
        
                    m = obj.car.m;
                    lf = obj.car.lf;
                    lr = obj.car.lr;
                    gAcc = obj.car.g;
                    fzNominal = obj.car.fzNominal;
        
                    % normal load on the one front wheel
                    Ffz = lr*m*gAcc/(2.0*(lf+lr));
                    Dffz = (Ffz-fzNominal)/fzNominal;
        
                    % normal load on the one rear wheel
                    Frz = lf*m*gAcc/(2.0*(lf+lr));
                    Drfz = (Frz-fzNominal)/fzNominal;
                    
                    % rolling resistance of the two front wheels
                    Ffrr = 2*obj.tire.QSY1*Ffz*tanh(vx);
        
                    % rolling resistance of the two rear wheels
                    Frrr = 2*obj.tire.QSY1*Frz*tanh(vx);
                    
                    % brakes front force
                    Fbf = (-cbf*brakes)/rDyn*tanh(vx);
        
                    % brakes rear force
                    Fbr = (-cbr*brakes)/rDyn*tanh(vx);
        
                    % drivetrain force
                    Fdrv = (cdrv*throttle)/rDyn;
        
                    % longitudinal front force
                    Ffx = Fbf+Ffrr;
                    
                    % longitudinal rear force
                    Frx = Fbr+Fdrv+Frrr;
        
                    % slip angle of the front wheel
                    saf = atan2((vy+r*lf),vx)-steeringAngle;
        
                    % slip angle of the rear wheel
                    sar = atan2((vy-r*lr),vx);
        
                    % latteral tire force Pacejka coefficients
                    % front wheel coefficients
                    Kfy = obj.tire.PKY1*fzNominal *...
                    sin(2.0*atan2(Ffz,(obj.tire.PKY2*fzNominal*obj.tire.LFZO)))*obj.tire.LFZO*obj.tire.LKY;
        
                    mufy = (obj.tire.PDY1+obj.tire.PDY2*Dffz)*obj.tire.LMUY;
                    Dfy = mufy*Ffz;
        
                    Cfy = obj.tire.PCY1*obj.tire.LCY;
        
                    Bfy = Kfy/(Cfy*Dfy);
        
                    Efy = (obj.tire.PEY1+obj.tire.PEY2*Dffz)*obj.tire.LEY;
                    
                    % rear wheel coefficients
                    Kry = obj.tire.PKY1*fzNominal *...
                    sin(2.0*atan2(Frz,(obj.tire.PKY2*fzNominal*obj.tire.LFZO)))*obj.tire.LFZO*obj.tire.LKY;
        
                    mury = (obj.tire.PDY1+obj.tire.PDY2*Drfz)*obj.tire.LMUY;
                    Dry = mury*Frz;
        
                    Cry = obj.tire.PCY1*obj.tire.LCY;
        
                    Bry = Kry/(Cry*Dry);
        
                    Ery = (obj.tire.PEY1+obj.tire.PEY2*Drfz)*obj.tire.LEY;   
                    % latteral front force
                    Ffy = 2*Dfy*sin(Cfy*atan(Bfy*saf-Efy*(Bfy*saf-atan(Bfy*saf))));
                    % latteral rear force
                    Fry = 2*Dry*sin(Cry*atan(Bry*sar-Ery*(Bry*sar-atan(Bry*sar))));

%                     constrF = (Ffx/obj.car.muxFz)^2+(Ffy/obj.car.muyFz)^2;
%                     constrR = (Frx/obj.car.muxFz)^2+(Fry/obj.car.muyFz)^2;
        end

        function rhs = initKinematicModel(obj, states, controls)
            yaw = states(3);
            vx = states(4);
            vy = states(5);
            r = states(6);
            throttle = states(8);
            steeringAngle = states(9);
            brakes = states(10);
            vs = states(11);

            dThrottle = controls(1);
            dSteeringAngle = controls(2);
            dBrakes = controls(3);
            dVs = controls(4);
            
            % car parameters
            rDyn = obj.car.rDyn;
            
            cdrv = obj.car.cm1*obj.car.gearRatio;
            cbf = obj.car.cbf;
            cbr = obj.car.cbr;

            m = obj.car.m;
            lf = obj.car.lf;
            lr = obj.car.lr;
            gAcc = obj.car.g;

            % normal load on the one front wheel
            Ffz = lr*m*gAcc/(2.0*(lf+lr));

            % normal load on the one rear wheel
            Frz = lf*m*gAcc/(2.0*(lf+lr));
            
            % rolling resistance of the two front wheels
            Ffrr = 2*obj.tire.QSY1*Ffz*tanh(vx);

            % rolling resistance of the two rear wheels
            Frrr = 2*obj.tire.QSY1*Frz*tanh(vx);

            % brakes front force
            Fbf = (-cbf*brakes)/rDyn*tanh(vx);

            % brakes rear force
            Fbr = (-cbr*brakes)/rDyn*tanh(vx);

            % drivetrain force
            Fdrv = (cdrv*throttle)/rDyn;

            % longitudinal front force
            Ffx = Fbf+Ffrr;
            
            % longitudinal rear force
            Frx = Fbr+Fdrv+Frrr;

            % drag force
            Fdrag = obj.car.cd*vx^2.0;

            % dot vx
            vxDot = 1/m*(Frx+cos(steeringAngle)*Ffx+Fdrag);

            rhs = [vx*cos(yaw)-vy*sin(yaw);
                   vx*sin(yaw)+vy*cos(yaw);
                   r;
                   vxDot;
                   lr/(lr+lf)*(dSteeringAngle*vx+steeringAngle*vxDot);
                   1/(lr+lf)*(dSteeringAngle*vx+steeringAngle*vxDot);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];
        end

        function rhs = initSimpleDynamicModel(obj,state,input)
            
            yaw = state(3);
            vx = state(4);
            vy = state(5);

            r = state(6);
            throttle = state(8);
            steeringAngle = state(9);
            brakes = state(10);
            vs = state(11);
            
            dThrottle = input(1);
            dSteeringAngle = input(2);
            dBrakes = input(3);
            dVs = input(4);

            m = obj.car.m;
            iz = obj.car.iz;
            lf = obj.car.lf;
            lr = obj.car.lr;
            gAcc = obj.car.g;
            cbf = obj.car.cbf;
            cbr = obj.car.cbr;
            rDyn = obj.car.rDyn;
            cdrv = obj.car.cm1 * obj.car.gearRatio;

            % normal load on the one front wheel
            Ffz = lr*m*gAcc/(2.0*(lf+lr));

            % normal load on the one rear wheel
            Frz = lf*m*gAcc/(2.0*(lf+lr));
            
            % rolling resistance of the two front wheels
            Ffrr = 2*obj.tire.QSY1*Ffz*tanh(vx);

            % rolling resistance of the two rear wheels
            Frrr = 2*obj.tire.QSY1*Frz*tanh(vx);
            
            % brakes front force
            Fbf = (-cbf*brakes)/rDyn*tanh(vx);

            % brakes rear force
            Fbr = (-cbr*brakes)/rDyn*tanh(vx);

            % drivetrain force
            Fdrv = (cdrv*throttle)/rDyn;

            % longitudinal front force
            Ffx = Fbf+Ffrr;
            
            % longitudinal rear force
            Frx = Fbr+Fdrv+Frrr;

            % slip angle of the front wheel
            saf = atan2((vy+r*lf),vx)-steeringAngle;

            % slip angle of the rear wheel
            sar = atan2((vy-r*lr),vx);

            % latteral front force
            Ffy = -saf*obj.tire.Cy;

            % latteral rear force
            Fry = -sar*obj.tire.Cy;

            % drag force
            Fdrag = obj.car.cd*vx^2.0;

            rhs = [vx*cos(yaw)-vy*sin(yaw);
                   vx*sin(yaw)+vy*cos(yaw);
                   r;
                   1/m*...
                   (Frx+cos(steeringAngle)*Ffx+Fdrag-sin(steeringAngle)*Ffy +...
                   m*vy*r);
                   1/m*...
                   (Fry+cos(steeringAngle)*Ffy+sin(steeringAngle)*Ffx-m*vx*r);
                   1/iz *...
                   (-Fry*lr+(cos(steeringAngle)*Ffy+sin(steeringAngle)*Ffx)*lf);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];
        end

        function rhs = initDynamicModel(obj, states, controls)
                    yaw = states(3);
                    vx = states(4);
                    vy = states(5);
                    r = states(6);
                    throttle = states(8);
                    steeringAngle = states(9);
                    brakes = states(10);
                    vs = states(11);
        
                    dThrottle = controls(1);
                    dSteeringAngle = controls(2);
                    dBrakes = controls(3);
                    dVs = controls(4);
        
                    %Dynamic forces
                    rDyn = obj.car.rDyn;
                    
                    cdrv = obj.car.cm1 * obj.car.gearRatio;
                    cbf = obj.car.cbf;
                    cbr = obj.car.cbr;
        
                    m = obj.car.m;
                    iz = obj.car.iz;
                    lf = obj.car.lf;
                    lr = obj.car.lr;
                    gAcc = obj.car.g;
                    fzNominal = obj.car.fzNominal;
        
                    % normal load on the one front wheel
                    Ffz = lr*m*gAcc/(2.0*(lf+lr));
                    Dffz = (Ffz-fzNominal)/fzNominal;
        
                    % normal load on the one rear wheel
                    Frz = lf*m*gAcc/(2.0*(lf+lr));
                    Drfz = (Frz-fzNominal)/fzNominal;
                    
                    % rolling resistance of the two front wheels
                    Ffrr = 2*obj.tire.QSY1*Ffz*tanh(vx);
        
                    % rolling resistance of the two rear wheels
                    Frrr = 2*obj.tire.QSY1*Frz*tanh(vx);
                    
                    % brakes front force
                    Fbf = (-cbf*brakes)/rDyn*tanh(vx);
        
                    % brakes rear force
                    Fbr = (-cbr*brakes)/rDyn*tanh(vx);
        
                    % drivetrain force
                    Fdrv = (cdrv*throttle)/rDyn;
        
                    % longitudinal front force
                    Ffx = Fbf+Ffrr;
                    
                    % longitudinal rear force
                    Frx = Fbr+Fdrv+Frrr;
        
                    % slip angle of the front wheel
                    saf = atan2((vy+r*lf),vx)-steeringAngle;
        
                    % slip angle of the rear wheel
                    sar = atan2((vy-r*lr),vx);
        
                    % latteral tire force Pacejka coefficients
                    % front wheel coefficients
                    Kfy = obj.tire.PKY1*fzNominal *...
                    sin(2.0*atan2(Ffz,(obj.tire.PKY2*fzNominal*obj.tire.LFZO)))*obj.tire.LFZO*obj.tire.LKY;
        
                    mufy = (obj.tire.PDY1+obj.tire.PDY2*Dffz)*obj.tire.LMUY;
                    Dfy = mufy*Ffz;
        
                    Cfy = obj.tire.PCY1*obj.tire.LCY;
        
                    Bfy = Kfy/(Cfy*Dfy);
        
                    Efy = (obj.tire.PEY1+obj.tire.PEY2*Dffz)*obj.tire.LEY;
                    
                    % rear wheel coefficients
                    Kry = obj.tire.PKY1*fzNominal *...
                    sin(2.0*atan2(Frz,(obj.tire.PKY2*fzNominal*obj.tire.LFZO)))*obj.tire.LFZO*obj.tire.LKY;
        
                    mury = (obj.tire.PDY1+obj.tire.PDY2*Drfz)*obj.tire.LMUY;
                    Dry = mury*Frz;
        
                    Cry = obj.tire.PCY1*obj.tire.LCY;
        
                    Bry = Kry/(Cry*Dry);
        
                    Ery = (obj.tire.PEY1+obj.tire.PEY2*Drfz)*obj.tire.LEY;
        
                    % latteral front force
                    Ffy = 2*Dfy*sin(Cfy*atan(Bfy*saf-Efy*(Bfy*saf-atan(Bfy*saf))));
        
                    % latteral rear force
                    Fry = 2*Dry*sin(Cry*atan(Bry*sar-Ery*(Bry*sar-atan(Bry*sar))));
                    
                    % drag force
                    Fdrag = obj.car.cd*vx^2.0;
        
                    rhs = [vx*cos(yaw)-vy*sin(yaw);
                           vx*sin(yaw)+vy*cos(yaw);
                           r;
                           1/m*...
                           (Frx+cos(steeringAngle)*Ffx+Fdrag-sin(steeringAngle)*Ffy +...
                           m*vy*r);
                           1/m*...
                           (Fry+cos(steeringAngle)*Ffy+sin(steeringAngle)*Ffx-m*vx*r);
                           1/iz *...
                           (-Fry*lr+(cos(steeringAngle)*Ffy+sin(steeringAngle)*Ffx)*lf);
                           vs;
                           dThrottle;
                           dSteeringAngle;
                           dBrakes;
                           dVs];
        end

        function rhs = initSimpleCombinedModel(obj,state,input)
            dynamicRhs = obj.initSimpleDynamicModel(state,input);
            kinematicRhs = obj.initKinematicModel(state,input);

            yaw = state(3);
            vx = state(4);
            vy = state(5);
            r = state(6);
            vs = state(11);

            dThrottle = input(1);
            dSteeringAngle = input(2);
            dBrakes = input(3);
            dVs = input(4);

            lambda = min(max((vx-3)/2,0),1);

            rhs = [vx*cos(yaw)-vy*sin(yaw);
                   vx*sin(yaw)+vy*cos(yaw);
                   r;
                   lambda*dynamicRhs(4)+(1-lambda)*kinematicRhs(4);
                   lambda*dynamicRhs(5)+(1-lambda)*kinematicRhs(5);
                   lambda*dynamicRhs(6)+(1-lambda)*kinematicRhs(6);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];
        end

        function rhs = initCombinedModel(obj,state,input)
            dynamicRhs = obj.initDynamicModel(state,input);
            kinematicRhs = obj.initKinematicModel(state,input);

            yaw = state(3);
            vx = state(4);
            vy = state(5);
            r = state(6);
            vs = state(11);

            dThrottle = input(1);
            dSteeringAngle = input(2);
            dBrakes = input(3);
            dVs = input(4);

            lambda = min(max((vx-3)/2,0),1);

            rhs = [vx*cos(yaw)-vy*sin(yaw);
                   vx*sin(yaw)+vy*cos(yaw);
                   r;
                   lambda*dynamicRhs(4)+(1-lambda)*kinematicRhs(4);
                   lambda*dynamicRhs(5)+(1-lambda)*kinematicRhs(5);
                   lambda*dynamicRhs(6)+(1-lambda)*kinematicRhs(6);
                   vs;
                   dThrottle;
                   dSteeringAngle;
                   dBrakes;
                   dVs];
        end
    end
end

