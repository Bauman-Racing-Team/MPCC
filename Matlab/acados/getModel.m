function model = getModel(parameters)
    import casadi.*;

    % State
    x = SX.sym('x');
    y = SX.sym('y');
    yaw = SX.sym('yaw');
    vx = SX.sym('vx');
    vy = SX.sym('vy');
    r = SX.sym('r');
    s = SX.sym('s');
    throttle = SX.sym('throttle');
    steeringAngle = SX.sym('steeringAngle');
    brakes = SX.sym('brakes');
    vs = SX.sym('vs');

    state = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs];

    % Controls
    dThrottle = SX.sym('dThrottle');
    dSteeringAngle = SX.sym('dSteeringAngle');
    dBrakes = SX.sym('dBrakes');
    dVs = SX.sym('dVs');

    input = [dThrottle;dSteeringAngle;dBrakes;dVs];

    % xdot
    xDot = SX.sym('xDot');
    yDot = SX.sym('yDot');
    yawDot = SX.sym('yawDot');
    vxDot = SX.sym('vxDot');
    vyDot = SX.sym('vyDot');
    rDot = SX.sym('rDot');
    sDot = SX.sym('sDot');
    throttleDot = SX.sym('throttleDot');
    steeringAngleDot = SX.sym('steeringAngleDot');
    brakesDot = SX.sym('brakesDot');
    vsDot = SX.sym('vsDot');

    xdot = [xDot;yDot;yawDot;vxDot;vyDot;rDot;sDot;throttleDot;steeringAngleDot;brakesDot;vsDot];
    
    % algebraic variables
    z = [];

    % parameters
    xTrack = SX.sym('xTrack');
    yTrack = SX.sym('yTrack');
    phiTrack = SX.sym('phiTrack');
    s0 = SX.sym('s0');

    p = [xTrack;yTrack;phiTrack;s0];
    
    % dynamics
    carModel = Model(parameters.car,parameters.tire);
    f_expl = carModel.initSimpleCombinedModel(state,input);
    %f_expl = carModel.initKinematicModel(state,input);
    f_impl = f_expl - xdot;

    f = Function('f',{state,input},{f_expl});

    % cost

    xRef = xTrack + (s-s0)*cos(phiTrack);
    yRef = yTrack + (s-s0)*sin(phiTrack);
    % contouring error
    ec = -cos(phiTrack)*(yRef-y)+sin(phiTrack)*(xRef-x);
    % lag error
    el = cos(phiTrack)*(xRef-x)+sin(phiTrack)*(yRef-y);
   
    error = [ec;el];

    % Coeffs for laf and contouring errors penallization
    Q = diag([parameters.costs.qC,parameters.costs.qL]);

    qVs = parameters.costs.qVs;
    vRef = parameters.mpcModel.vRef;

    % Coeffs for control inputs penalization
    R = diag([parameters.costs.rThrottle, ...
              parameters.costs.rSteeringAngle, ...
              parameters.costs.rBrakes, ...
              parameters.costs.rVs]);

%     cost_expr_ext_cost = error'*Q*error+input'*R*input+qVs*(vRef-vs)^2;
%     cost_expr_ext_cost_e = error'*Q*error+qVs*(vRef-vs)^2;
    cost_expr_ext_cost = error'*Q*error+input'*R*input-qVs*vs;
    cost_expr_ext_cost_e = error'*Q*error-qVs*vs;

    % constraints 
    lf = parameters.car.lf;
    lr = parameters.car.lr;
    
    constr_expr_h = [];

    lambda = min(max((vx - 3)/2,0),1);

    % front slip angle constraint
    constr_expr_h = [constr_expr_h;(atan2((vy + r*lf),vx) - steeringAngle)*lambda];

    % rear slip angle constraint
    constr_expr_h = [constr_expr_h;(atan2((vy - r*lr),vx))*lambda];

    % track constraint
    constr_expr_h = [constr_expr_h;(x-xTrack)^2 + (y-yTrack)^2];

    % friction ellipse constraint
    m = parameters.car.m;
    gAcc = parameters.car.g;
    fzNominal = parameters.car.fzNominal;
    cbr=parameters.car.cbr;
    cbf=parameters.car.cbf;
    rDyn = parameters.car.rDyn;
    cdrv = parameters.car.cm1 * parameters.car.gearRatio;
    
    Frz = lf*m*gAcc/(2.0*(lf+lr));
    Drfz = (Frz-fzNominal)/fzNominal;
    Kry = parameters.tire.PKY1*fzNominal * sin(2.0*atan2(Frz,(parameters.tire.PKY2*fzNominal*parameters.tire.LFZO)))*parameters.tire.LFZO*parameters.tire.LKY;
    Dry = (parameters.tire.PDY1+parameters.tire.PDY2*Drfz)*parameters.tire.LMUY*Frz;
    Cry = parameters.tire.PCY1*parameters.tire.LCY;
    Bry = Kry/(Cry*Dry);
    Ery = (parameters.tire.PEY1+parameters.tire.PEY2*Drfz)*parameters.tire.LEY;
    sar = atan2((vy-r*lr),vx);
    
    Frx = (-cbr*brakes)/rDyn*tanh(vx)+(cdrv*throttle)/rDyn+2*parameters.tire.QSY1*Frz*tanh(vx);
    Fry = 2*Dry*sin(Cry*atan(Bry*sar-Ery*(Bry*sar-atan(Bry*sar))));
    
    Ffz = lr*m*gAcc/(2.0*(lf+lr));
    Dffz = (Ffz-fzNominal)/fzNominal;
    Kfy = parameters.tire.PKY1*fzNominal * sin(2.0*atan2(Ffz,(parameters.tire.PKY2*fzNominal*parameters.tire.LFZO)))*parameters.tire.LFZO*parameters.tire.LKY;        
    mufy = (parameters.tire.PDY1+parameters.tire.PDY2*Dffz)*parameters.tire.LMUY;
    Dfy = mufy*Ffz;
    Cfy = parameters.tire.PCY1*parameters.tire.LCY;
    Bfy = Kfy/(Cfy*Dfy);
    Efy = (parameters.tire.PEY1+parameters.tire.PEY2*Dffz)*parameters.tire.LEY;
    saf = atan2((vy+r*lf),vx)-steeringAngle;
    
    Ffy = 2*Dfy*sin(Cfy*atan(Bfy*saf-Efy*(Bfy*saf-atan(Bfy*saf))));
    Ffx = (-cbf*brakes)/rDyn*tanh(vx)+2*parameters.tire.QSY1*Ffz*tanh(vx);
    
    % latteral front force
    Ffy = -saf*29117;

    % latteral rear force
    Fry = -sar*29117;

    %   muy*Fz=2030.5; %a
    %   mux*Fz=2118.04; %b
    %   (Ffx/2118.04)^2+(Ffy/2030.5)^2<1;
    %   (Frx/2118.04)^2+(Fry/2030.5)^2<1;
    constr_expr_h = [constr_expr_h;(Ffx/2118.04)^2+(Ffy/2030.5)^2;(Frx/2118.04)^2+(Fry/2030.5)^2];

    % model filling
    model.f_expl_expr = f_expl;
    model.f_impl_expr = f_impl;
    model.f = f;
    model.x = state;
    model.xdot = xdot;
    model.u = input;
    model.p = p;
    model.z = z;
    model.cost_expr_ext_cost = cost_expr_ext_cost;
    model.cost_expr_ext_cost_e = cost_expr_ext_cost_e;
    model.constr_expr_h = constr_expr_h;
end

