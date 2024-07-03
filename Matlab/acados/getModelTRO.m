function model = getModelTRO(parameters)
    import casadi.*;

    % State
    n = SX.sym('n');
    miu = SX.sym('miu');
    vx = SX.sym('vx');
    vy = SX.sym('vy');
    r = SX.sym('r');
    throttle = SX.sym('throttle');
    steeringAngle = SX.sym('steeringAngle');
    brakes = SX.sym('brakes');

    state = [n;miu;vx;vy;r;throttle;steeringAngle;brakes];

    % Controls
    dThrottle = SX.sym('dThrottle');
    dSteeringAngle = SX.sym('dSteeringAngle');
    dBrakes = SX.sym('dBrakes');
    Mtv = SX.sym('Mtv');

    input = [dThrottle;dSteeringAngle;dBrakes;Mtv];

    % xdot
    nDot = SX.sym('nDot');
    miuDot = SX.sym('miuDot');
    vxDot = SX.sym('vxDot');
    vyDot = SX.sym('vyDot');
    rDot = SX.sym('rDot');
    throttleDot = SX.sym('throttleDot');
    steeringAngleDot = SX.sym('steeringAngleDot');
    brakesDot = SX.sym('brakesDot');

    xdot = [nDot;miuDot;vxDot;vyDot;rDot;throttleDot;steeringAngleDot;brakesDot];
    
    % algebraic variables
    z = [];

    % parameters
    curvTrack = SX.sym('curvTrack');
    qBeta = SX.sym('qBeta');

    rdThrottle = SX.sym('rdThrottle');
    rdSteeringAngle = SX.sym('rdSteeringAngle');
    rdBrakes = SX.sym('rdBrakes');
    rMtv = SX.sym('rMtv');


    p = [curvTrack;qBeta;rdThrottle;rdSteeringAngle;rdBrakes;rMtv];

    % dynamics
    carModel = Model(parameters.car,parameters.tire);
    f_expl = carModel.initCurvilinearDynamicModel(state,input);
    f_impl = f_expl - xdot;

    f = Function('f',{state,input},{f_expl});

    % cost
    lf = parameters.car.lf;
    lr = parameters.car.lr;
    % Coeffs for slip angle penallization
    betaKin = atan2(steeringAngle*lr,lf+lr);
    betaDyn = atan2(vy,vx);

    % Coeffs for control inputs penalization
    R = diag([rdThrottle, ...
              rdSteeringAngle, ...
              rdBrakes, ...
              rMtv]);

    cost_expr_ext_cost = -(vx*cos(miu)-vy*sin(miu))/(1-n*curvTrack)+input'*R*input+qBeta*(betaKin-betaDyn)^2;
    cost_expr_ext_cost_e = -(vx*cos(miu)-vy*sin(miu))/(1-n*curvTrack)+qBeta*(betaKin-betaDyn)^2;

    % constraints 
    carL = parameters.car.carL;
    carW = parameters.car.carW;
    
    constr_expr_h = [];

    lambda = min(max((vx - 3)/2,0),1);

    % front slip angle constraint
    constr_expr_h = [constr_expr_h;(atan2((vy + r*lf),vx) - steeringAngle)*lambda];

    % rear slip angle constraint
    constr_expr_h = [constr_expr_h;(atan2((vy - r*lr),vx))*lambda];

    % track constraint
    constr_expr_h = [constr_expr_h;n - carL/2*sin(abs(miu))+carW/2*cos(miu)];

    constr_expr_h = [constr_expr_h;-n + carL/2*sin(abs(miu))+carW/2*cos(miu)];

    % friction ellipse constraint
    [Ffx,Ffy,Frx,Fry] = carModel.initCurvFrictionEllipseConstraint(state);
    constrF = (Ffx/parameters.car.muxFz)^2+(Ffy/parameters.car.muyFz)^2;
    constrR = (Frx/parameters.car.muxFz)^2+(Fry/parameters.car.muyFz)^2;
    constr_expr_h = [constr_expr_h;constrF;constrR];

    constr_expr_h = [constr_expr_h;throttle*brakes];

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