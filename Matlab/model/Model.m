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

    function [constrF,constrR] = initSimpleFrictionEllipseConstraint(obj,states) 
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

                    constrF = (Ffx/obj.car.muxFz)^2+(Ffy/obj.car.muyFz)^2;
                    constrR = (Frx/obj.car.muxFz)^2+(Fry/obj.car.muyFz)^2;
        end
        
        function [constrF,constrR] = initFrictionEllipseConstraint(obj, states)
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

                    constrF = (Ffx/obj.car.muxFz)^2+(Ffy/obj.car.muyFz)^2;
                    constrR = (Frx/obj.car.muxFz)^2+(Fry/obj.car.muyFz)^2;
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
            Ffy = -saf*29700;

            % latteral rear force
            Fry = -sar*29700;

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

        function rhs = initCombinedSlipDynamicModel(obj, states, controls)
                    yaw = states(3);
                    vx = states(4);
                    vy = states(5);
                    r = states(6);
                    throttle = states(8);
                    steeringAngle = states(9);
                    brakes = states(10);
                    vs = states(11);
                    omegaf = states(12);
                    omegar = states(13);
        
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
                    iw = obj.car.iw;
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
%                     Ffrr = 200*tanh(vx);
        
                    % rolling resistance of the two rear wheels
                    Frrr = 2*obj.tire.QSY1*Frz*tanh(vx);
%                     Frrr = 200*tanh(vx);
                    
                    % brakes front force
                    Fbf = (-cbf*brakes)/rDyn*tanh(vx);
        
                    % brakes rear force
                    Fbr = (-cbr*brakes)/rDyn*tanh(vx);
        
                    % drivetrain force
                    Fdrv = (cdrv*throttle)/rDyn;
               
                    % slip angle of the front wheel
                    saf0 = atan2((vy+r*lf),vx)-steeringAngle;
        
                    % slip angle of the rear wheel
                    sar0 = atan2((vy-r*lr),vx);
        
                    % slip ratio of the front wheel
                    vlf = (vy+r*lf)*sin(steeringAngle)+vx*cos(steeringAngle);
                    kappaf0 = (omegaf * rDyn - vlf) / (max(1.0, vlf));

                    % slip ratio of the rear wheel
                    kappar0 = (omegar * rDyn - vx) / (max(1.0, vx));

                    [Ffy0,Ffx0] = obj.CombinedSlipTireModel(saf0,kappaf0,Ffz,Dffz,fzNominal);
                    [Fry0,Frx0] = obj.CombinedSlipTireModel(sar0,kappar0,Frz,Drfz,fzNominal);
                    Ffy = 2*Ffy0;
                    Ffx = 2*Ffx0;
                    Fry = 2*Fry0;
                    Frx = 2*Frx0;
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
                           dVs;
                           -(Ffx-Fbf-Ffrr)/ 2 * rDyn / iw;
                           (Fdrv + Fbr - Frx + Frrr) / 2 * rDyn / iw];
        end
        
        
        function [Fy,Fx] = CombinedSlipTireModel(obj,alpha,kappa,Fz,Dfz,fzNominal)
            % latteral tire force Pacejka (Combined slip)
            SHy = (obj.tire.PHY1+obj.tire.PHY2*Dfz)*obj.tire.LHY;
            SVy = Fz*((obj.tire.PVY1 + obj.tire.PVY2 * Dfz) * obj.tire.LVY ) * obj.tire.LMUY;

            Ky = obj.tire.PKY1*fzNominal *...
            sin(2.0*atan2(Fz,(obj.tire.PKY2*fzNominal*obj.tire.LFZO)))*obj.tire.LFZO*obj.tire.LKY;

            muy = (obj.tire.PDY1+obj.tire.PDY2*Dfz)*obj.tire.LMUY;
            Dy = muy*Fz;

            Cy = obj.tire.PCY1*obj.tire.LCY;

            By = Ky/(Cy*Dy);

            Ey = (obj.tire.PEY1+obj.tire.PEY2*Dfz)*obj.tire.LEY;

            % combined latteral force
            sa = alpha + SHy;
            Fy0 = Dy*sin(Cy*atan(By*sa-Ey*(By*sa-atan(By*sa))))+SVy;
            Byk = obj.tire.RBY1 * cos(atan(obj.tire.RBY2 * (alpha - obj.tire.RBY3))) * obj.tire.LKY;
            Cyk = obj.tire.RCY1;
            Eyk = obj.tire.REY1 + obj.tire.REY2 * Dfz;
            SHyk = obj.tire.RHY1 + obj.tire.RHY2 * Dfz;
            Dyk = Fy0 / (cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk)))));
            DVyk = muy * Fz * (obj.tire.RVY1 + obj.tire.RVY2 * Dfz ) * cos(atan(Byk * SHyk));
            kappa_s = kappa + SHyk;
            SVyk = DVyk * sin(obj.tire.RVY5 * atan(obj.tire.RVY6 * kappa)) * obj.tire.LVYKA;

            Fy = Dyk * cos(Cyk * atan(Byk * kappa_s - Eyk * (Byk * kappa_s - atan(Byk * kappa_s))))+SVyk;

            % longitudinal tire force Pacejka (Combined slip)
            SHx = (obj.tire.PHX1 + obj.tire.PHX2 * Dfz) * obj.tire.LHX;
            SVx = Fz * (obj.tire.PVX1 + obj.tire.PVX2 * Dfz) * obj.tire.LVX * obj.tire.LMUX;
            Kx = Fz * (obj.tire.PKX1 + obj.tire.PKX2 * Dfz) * exp(obj.tire.PKX3 * Dfz) * obj.tire.LKX;
            mux = (obj.tire.PDX1 + obj.tire.PDX2 * Dfz);
            Cx = obj.tire.PCX1 * obj.tire.LCX;
            Dx = mux * Fz;
            Bx = Kx / (Cx * Dx);
            % combined longitudinal force
            kappax = kappa + SHx;
            Ex = (obj.tire.PEX1 + obj.tire.PEX2 * Dfz + obj.tire.PEX3 * Dfz * Dfz) * (1 - obj.tire.PEX4 * sign(kappax)) * obj.tire.LEX;
            Fx0 = Dx * sin(Cx * atan(Bx * kappax - Ex * (Bx * kappax - atan(Bx * kappax)))) + SVx;

            Bxa = obj.tire.RBX1 * cos(atan(obj.tire.RBX2 * kappa)) * obj.tire.LXAL;
            Cxa = obj.tire.RCX1;
            Exa = obj.tire.REX1 + obj.tire.REX2 * Dfz;
            SHxa = obj.tire.RHX1;
            Dxa = Fx0 / (cos(Cxa * atan(Bxa * SHxa - Exa * (Bxa * SHxa - atan(Bxa * SHxa)))));
            alpha_s = alpha + SHxa;
            Fx = Dxa * cos(Cxa * atan(Bxa * alpha_s - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s))));

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

