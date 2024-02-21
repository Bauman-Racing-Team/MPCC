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
    qC = SX.sym('qC');
    qL = SX.sym('qL');
    qVs = SX.sym('qVs');
    rThrottle = SX.sym('rThrottle');
    rSteeringAngle = SX.sym('rSteeringAngle');
    rBrakes = SX.sym('rBrakes');
    rVs = SX.sym('rVs');
    xTrack = SX.sym('xTrack');
    yTrack = SX.sym('yTrack');
    phiTrack = SX.sym('phiTrack');
    s0 = SX.sym('s0');

    p = [xTrack;yTrack;phiTrack;s0;qC;qL;qVs;rThrottle;rSteeringAngle;rBrakes;rVs];
    
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
    Q = diag([qC,qL]);

%     qVs = parameters.costs.qVs;
    vRef = parameters.mpcModel.vRef;

    % Coeffs for control inputs penalization
    R = diag([rThrottle, ...
              rSteeringAngle, ...
              rBrakes, ...
              rVs]);

    cost_expr_ext_cost = error'*Q*error+input'*R*input+qVs*(vRef-vs)^2;
    cost_expr_ext_cost_e = error'*Q*error+qVs*(vRef-vs)^2;
    
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
    [constrF,constrR] = carModel.initFrictionEllipseConstraint(state);
    constr_expr_h = [constr_expr_h;constrF;constrR];

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

