classdef Ipopt < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        config
        parameters
        ts
        car

        model

        track
        outerBorder
        innerBorder

        trackInterpolation;
        outerBorderInterpolation;
        innerBorderInterpolation;

        X
        U
        P
        S
        f
        g
        p
        objective

        Q
        R
        Z

        solver
        opts

        initialStateGuess
        initialControlGuess

        validInitialGuess

        args
    end
    
    methods (Access = public)
        function obj = Ipopt(config,parameters)
            obj.config = config;
            obj.parameters = parameters;
            obj.ts = parameters.config.ts;
            obj.car = parameters.car;

            obj.model = Model(parameters.car,parameters.tire);


            obj.validInitialGuess = false;

            obj.track.centerLine = ArcLengthSpline(config,parameters.mpcModel);
            obj.track.outerBorder = ArcLengthSpline(config,parameters.mpcModel);
            obj.track.innerBorder = ArcLengthSpline(config,parameters.mpcModel);
        end

        function initMPC(obj)
            obj.initSystemModel();
            obj.initEqualityConstraints();
            %obj.initLinearTrackConstraint();
            obj.initNewTrackConstraint();
            %obj.initTrackConstraint();
            obj.initTiresConstraints();
            obj.initCostFunction();
            obj.initIpoptSolver();
            obj.setBounds();
        end

        function setTrack(obj,track)
            import casadi.*;

            obj.track.centerLine.gen2DSpline([track.x;track.x],[track.y;track.y]);
            obj.track.outerBorder.gen2DSpline([track.xOuter;track.xOuter],[track.yOuter,track.yOuter]);
            obj.track.innerBorder.gen2DSpline([track.xInner;track.xInner],[track.yInner,track.yInner]);

            centerLine = obj.track.centerLine.getPath();
            outerBoredr = obj.track.outerBorder.getPath();
            innerBorder = obj.track.innerBorder.getPath();

            centerLineDerivatives = zeros(2,length(centerLine.s));
            
            for i = 1:length(centerLine.s)
                centerLineDerivatives(:,i) = obj.track.centerLine.getDerivative(centerLine.s(i));
            end
            
            obj.track.centerLineInterpolation.x = interpolant('center_line_interpolation_x','bspline',{centerLine.s},centerLine.x);
            obj.track.centerLineInterpolation.y = interpolant('center_line_interpolation_y','bspline',{centerLine.s},centerLine.y);
            obj.track.centerLineDerivativesInterpolation.x = interpolant('center_line_derivative_interpolation_x','bspline',{centerLine.s},centerLineDerivatives(1,:));
            obj.track.centerLineDerivativesInterpolation.y = interpolant('center_line_derivative_interpolation_y','bspline',{centerLine.s},centerLineDerivatives(2,:));

            obj.track.outerBorderInterpolation.x = interpolant('outerBorder_interpolation_x','bspline',{outerBoredr.s},outerBoredr.x);
            obj.track.outerBorderInterpolation.y = interpolant('outerBorder_interpolation_y','bspline',{outerBoredr.s},outerBoredr.y);

            obj.track.innerBorderInterpolation.x = interpolant('innerBorder_interpolation_x','bspline',{innerBorder.s},innerBorder.x);
            obj.track.innerBorderInterpolation.y = interpolant('innerBorder_interpolation_y','bspline',{innerBorder.s},innerBorder.y);
        end

        function track = getTrack(obj)
            track = obj.track.centerLine;
        end

        function initSystemModel(obj)
            import casadi.*;

            % States
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
            
            states = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs];
            
            % Controls
            dThrottle = SX.sym('dThrottle');
            dSteeringAngle = SX.sym('dSteeringAngle');
            dBrakes = SX.sym('dBrakes');
            dVs = SX.sym('dVs');

            controls = [dThrottle,dSteeringAngle,dBrakes,dVs];

            rhs = obj.model.initSimpleCombinedModel(states,controls);
            % rhs = obj.initDynamicModel(states,controls);

            obj.f = Function('f',{states,controls},{rhs});
            
            obj.U = SX.sym('U',obj.config.NU,obj.config.N);
            obj.P = SX.sym('P',obj.config.NX + 4*(obj.config.N+1),1); % NX - initial state, 4*(obj.config.N+1) - [xTrack,yTrack,phiTrack,s0]
            %obj.P = MX.sym('P',obj.config.NX,1); % NX - initial state
            obj.X = SX.sym('X',obj.config.NX,(obj.config.N+1));
            % slack variables matrix for soft constraints
            obj.S = SX.sym('S',obj.config.NS,(obj.config.N+1));
            
            % objective function
            obj.objective = 0;

            % constraints
            obj.g = [];
            
            % Coeffs for laf and contouring errors penallization
            obj.Q = zeros(2,2);
            obj.Q(1,1) = obj.parameters.costs.qC;
            obj.Q(2,2) = obj.parameters.costs.qL;
            
            % Coeffs for control inputs penalization
            obj.R = zeros(obj.config.NU,obj.config.NU);
            obj.R(1,1) = obj.parameters.costs.rdThrottle;
            obj.R(2,2) = obj.parameters.costs.rdSteeringAngle;
            obj.R(3,3) = obj.parameters.costs.rdBrakes;
            obj.R(4,4) = obj.parameters.costs.rdVs;

            % Coeffs for soft constraints penalization
            obj.Z = zeros(obj.config.NS,obj.config.NS);
            obj.Z(1,1) = obj.parameters.costs.scQuadTrack;
            obj.Z(2,2) = obj.parameters.costs.scQuadAlpha;
            obj.Z(3,3) = obj.parameters.costs.scQuadAlpha;
        end

        function initEqualityConstraints(obj)
            x0 = obj.X(:,1);
            obj.g = [obj.g;x0-obj.P(1:obj.config.NX)];
            for i = 1:obj.config.N
                state = obj.X(:,i);
                stateNext = obj.X(:,i+1);
                control = obj.U(:,i);

                % rk4 integration
                stateNextRK4 = obj.ode4(state,control);
                
                % base constraint
                obj.g = [obj.g;stateNext-stateNextRK4];
            end
        end

        function initTrackConstraint(obj)
            for i = 1:obj.config.N+1

                x = obj.X(1,i);
                y = obj.X(2,i);
                s = obj.X(7,i);
                % track constraint: 0.0 <= (X - Xcen(S))^2 + (Y - Ycen(S))^2 <= rOut
                obj.g = [obj.g;(x-obj.track.centerLineInterpolation.x(s))^2 + (y-obj.track.centerLineInterpolation.y(s))^2 + obj.S(1,i)];

                % objective function with slack vars
                obj.objective = obj.objective + obj.Z(1,1) * obj.S(1,i)^2;
            end
        end

        function initNewTrackConstraint(obj)
            for i = 1:obj.config.N+1

                x = obj.X(1,i);
                y = obj.X(2,i);
                s = obj.X(7,i);

                params = obj.P(obj.config.NX+4*(i-1)+1:obj.config.NX+4*i);

                xTrack = params(1);
                yTrack = params(2);
                phiTrack = params(3);
                s0 = params(4);

                xRef = xTrack + cos(phiTrack)*(s-s0);
                yRef = yTrack + sin(phiTrack)*(s-s0);

                % track constraint: 0.0 <= (X - Xcen(S))^2 + (Y - Ycen(S))^2 <= rOut
                obj.g = [obj.g;(x-xRef)^2 + (y-yRef)^2 + obj.S(1,i)];

                % objective function with slack vars
                obj.objective = obj.objective + obj.Z(1,1) * obj.S(1,i)^2;
            end
        end

        function initLinearTrackConstraint(obj)
            for i = 1:obj.config.N+1

                x = obj.X(1,i);
                y = obj.X(2,i);
                s = obj.X(7,i);

                yOuter = obj.track.outerBorderInterpolation.y(s);
                xOuter = obj.track.outerBorderInterpolation.x(s);

                yInner = obj.track.innerBorderInterpolation.y(s);
                xInner = obj.track.innerBorderInterpolation.x(s);

                k = obj.track.centerLineDerivativesInterpolation.y(s)/obj.track.centerLineDerivativesInterpolation.x(s);

                b1 = max(yOuter - k*xOuter,yInner - k*xInner);
                b2 = min(yOuter - k*xOuter,yInner - k*xInner);

                % track constraint: -inf <= y-kx-b1 <= 0.0
                obj.g = [obj.g;y-k*x-b1+obj.S(1,i)];
                
                % track constraint: 0.0 <= y-kx-b2 <= inf
                obj.g = [obj.g;y-k*x-b2+obj.S(1,i)];

                % objective function with slack vars
                obj.objective = obj.objective + obj.Z(1,1) * obj.S(1,i)^2;
            end
        end

        function initTiresConstraints(obj)
            for i = 1:obj.config.N+1
    
                vx = obj.X(4,i);
                vy = obj.X(5,i);
                r = obj.X(6,i);
                steeringAngle = obj.X(9,i);
    
                lf = obj.car.lf;
                lr = obj.car.lr;

                % front slip angle constraint with slack variable
                safg = atan2((vy + r*lf),vx) - steeringAngle + obj.S(2,i);
                obj.g = [obj.g;safg];  
    
                % rear slip angle constraint with slack variable
                sarg = atan2((vy - r*lr),vx) + obj.S(3,i);
                obj.g = [obj.g;sarg];

                % soft constraints slack variables matrix
                slacks = obj.S(2:3,i);
                
                % objective function with slack vars
                obj.objective = obj.objective + slacks' *obj.Z(2:3,2:3) * slacks;
            end 
        end

        function initCostFunction(obj)
            % stages cost
            for i = 1:obj.config.N
                control = obj.U(:,i);

                % objective function
                obj.objective = obj.objective+obj.stageCostWOControl(i)+control'*obj.R*control;
            end

            % terminal cost
            obj.objective = obj.objective+obj.stageCostWOControl(obj.config.N+1);
        end

        function cost = stageCostWOControl(obj,i)
            state = obj.X(:,i);
            params = obj.P(obj.config.NX+4*(i-1)+1:obj.config.NX+4*i);

            x = state(1);
            y = state(2);
            s = state(7);
            vs = state(11);

            xTrack = params(1);
            yTrack = params(2);
            phiTrack = params(3);
            s0 = params(4);

            xRef = xTrack + cos(phiTrack)*(s-s0);
            yRef = yTrack + sin(phiTrack)*(s-s0);
            
            % contouring error
            ec = -sin(phiTrack) * (xRef - x)...
                                + cos(phiTrack) * (yRef - y);
            % lag error
            el = cos(phiTrack) * (xRef - x)...
                                + sin(phiTrack) * (yRef - y);
            error = [ec;el];

            cost = error'*obj.Q*error - obj.parameters.costs.qVs * vs;
        end

        function initIpoptSolver(obj)
            import casadi.*;

            % make the decision variable one column  vector
            OPT_variables = [reshape(obj.X,obj.config.NX*(obj.config.N+1),1);reshape(obj.U,obj.config.NU*obj.config.N,1);reshape(obj.S,obj.config.NS*(obj.config.N+1),1)];
            nlpProb = struct('f', obj.objective, 'x', OPT_variables, 'g', obj.g, 'p', obj.P);

            obj.opts = struct;
            obj.opts.ipopt.max_iter = 200;
            obj.opts.ipopt.print_level = 3;%0,3
            obj.opts.print_time = 0;
            %obj.opts.compiler = 'shell';
            %obj.opts.jit = true;
            obj.opts.ipopt.acceptable_tol =1e-8;
            obj.opts.ipopt.acceptable_obj_change_tol = 1e-6;
            obj.opts.ipopt.linear_solver = 'ma27';
            
            obj.solver = nlpsol('solver','ipopt',nlpProb,obj.opts);
        end

        function setBounds(obj)
            obj.args.lbx = zeros(obj.config.NX * (obj.config.N+1) + obj.config.NU * (obj.config.N) + obj.config.NS*(obj.config.N+1),1); % lower bounds for states, inputs and slack vars
            obj.args.ubx = zeros(obj.config.NX * (obj.config.N+1) + obj.config.NU * (obj.config.N) + obj.config.NS*(obj.config.N+1),1); % upper bounds for states, inputs and slack vars

            obj.args.lbg = zeros(obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1),1); % lower bounds for equality and inequality constraints
            obj.args.ubg = zeros(obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1),1); % upper bounds for equality and inequality constraints

            % lower and upper bounds for states

            obj.args.lbx(1:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = -inf;
            obj.args.lbx(2:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = -inf;
            obj.args.lbx(3:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = -2*pi;
            obj.args.lbx(4:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = 1.0;
            obj.args.lbx(5:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = -inf;
            obj.args.lbx(6:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = -inf;
            obj.args.lbx(7:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = 0.0;
            obj.args.lbx(8:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = obj.parameters.bounds.lowerStateBounds.throttleL;
            obj.args.lbx(9:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = obj.parameters.bounds.lowerStateBounds.steeringAngleL;
            obj.args.lbx(10:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = obj.parameters.bounds.lowerStateBounds.brakesL;
            obj.args.lbx(11:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = 0.0;

            obj.args.ubx(1:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = inf;
            obj.args.ubx(2:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = inf;
            obj.args.ubx(3:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = 2*pi;
            obj.args.ubx(4:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = 15;
            obj.args.ubx(5:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = inf;
            obj.args.ubx(6:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = inf;
            obj.args.ubx(7:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = inf;
            obj.args.ubx(8:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = obj.parameters.bounds.upperStateBounds.throttleU;
            obj.args.ubx(9:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = obj.parameters.bounds.upperStateBounds.steeringAngleU;
            obj.args.ubx(10:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = obj.parameters.bounds.upperStateBounds.brakesU;
            obj.args.ubx(11:obj.config.NX:obj.config.NX*(obj.config.N+1),1) = inf;
           
            % lower and upper bounds for inputs
            
            obj.args.lbx(obj.config.NX*(obj.config.N+1)+1: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.lowerInputBounds.dThrottleL;

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+2: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.lowerInputBounds.dSteeringAngleL;

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+3: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.lowerInputBounds.dBrakesL;

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+4: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.lowerInputBounds.dVsL;


            obj.args.ubx(obj.config.NX*(obj.config.N+1)+1: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.upperInputBounds.dThrottleU;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+2: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.upperInputBounds.dSteeringAngleU;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+3: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.upperInputBounds.dBrakesU;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+4: ...
                                          obj.config.NU: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N,1) = obj.parameters.bounds.upperInputBounds.dVsU;

            % lower and upper bounds vals for states, inputs and slack variables

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+1: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = -inf;

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+2: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = -inf;

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+3: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = -inf;

            obj.args.lbx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+4: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = -inf;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+1: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = inf;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+2: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = inf;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+3: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = inf;

            obj.args.ubx(obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+4: ...
                                          obj.config.NS: ...
                                          obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N+obj.config.NS*(obj.config.N+1)) = inf;

            % lower and upper bounds for equality and inequality constraints
            % lower and upper bounds for the equality constraints
            obj.args.lbg(1:obj.config.NX*(obj.config.N+1)) = 0;
            obj.args.ubg(1:obj.config.NX*(obj.config.N+1)) = 0;
            % lower and upper bounds for the inequality constraints
            % lower and upper bounds for the linear track constraint
            %obj.args.lbg(obj.config.NX*(obj.config.N+1) + 1:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = -inf;
            %obj.args.ubg(obj.config.NX*(obj.config.N+1) + 1:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = 0;
            %obj.args.lbg(obj.config.NX*(obj.config.N+1) + 2:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = 0;
            %obj.args.ubg(obj.config.NX*(obj.config.N+1) + 2:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = inf;
            % lower and upper bounds for the track constraint
            obj.args.lbg(obj.config.NX*(obj.config.N+1) + 1:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = 0;
            obj.args.ubg(obj.config.NX*(obj.config.N+1) + 1:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = obj.parameters.mpcModel.rOut;
            % lower and upper bounds for the front slip angle
            obj.args.lbg(obj.config.NX*(obj.config.N+1) + 2:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = -obj.parameters.mpcModel.maxAlpha;
            obj.args.ubg(obj.config.NX*(obj.config.N+1) + 2:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = obj.parameters.mpcModel.maxAlpha;
            % lower and upper bounds for the rear slip angle
            obj.args.lbg(obj.config.NX*(obj.config.N+1) + 3:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = -obj.parameters.mpcModel.maxAlpha;
            obj.args.ubg(obj.config.NX*(obj.config.N+1) + 3:obj.config.NS:obj.config.NX*(obj.config.N+1) + obj.config.NS*(obj.config.N+1)) = obj.parameters.mpcModel.maxAlpha;
        end

        function ipoptReturn = runMPC(obj, x0)
            s0 = zeros(obj.config.NS,obj.config.N+1);
            x0(obj.config.siIndex.s) = obj.track.centerLine.projectOnSpline(vectorToState(x0));
            x0 = obj.unwrapState(x0);
            
            if obj.validInitialGuess
              obj.updateInitialGuess(x0);
            else
              obj.generateNewInitialGuess(x0);
            end

            obj.args.p(1:obj.config.NX,1) = x0;
          
            obj.args.x0 = [reshape(obj.initialStateGuess,obj.config.NX*(obj.config.N+1),1);
                           reshape(obj.initialControlGuess,obj.config.NU*(obj.config.N),1);
                           reshape(s0,obj.config.NS*(obj.config.N+1),1)];
            
            sol = obj.solver('x0', obj.args.x0, 'lbx', obj.args.lbx, 'ubx', obj.args.ubx,...
            'lbg', obj.args.lbg, 'ubg', obj.args.ubg,'p',obj.args.p);
            
            obj.initialStateGuess = reshape(full(sol.x(1:obj.config.NX*(obj.config.N+1)))', ...
                                   obj.config.NX,obj.config.N+1);
            obj.initialControlGuess = reshape(full(sol.x(obj.config.NX*(obj.config.N+1)+1:obj.config.NX*(obj.config.N+1)+obj.config.NU*obj.config.N))', ...
                                   obj.config.NU,obj.config.N);

            ipoptReturn = MpcReturn();

            ipoptReturn.x0 = obj.initialStateGuess(:,1);
            ipoptReturn.u0 = obj.initialControlGuess(:,1);
            ipoptReturn.mpcHorizon.states = obj.initialStateGuess;
            ipoptReturn.mpcHorizon.inputs = obj.initialControlGuess;
        end

        function x0 = unwrapState(obj,x0)
            lapLength = obj.track.centerLine.getLength()/2;
            if x0(obj.config.siIndex.yaw) > pi
              x0(obj.config.siIndex.yaw) = x0(obj.config.siIndex.yaw) - 2.0 * pi;
            end
            if x0(obj.config.siIndex.yaw) < -pi
              x0(obj.config.siIndex.yaw) = x0(obj.config.siIndex.yaw) + 2.0 * pi;
            end
            x0(obj.config.siIndex.s) = rem(x0(obj.config.siIndex.s),lapLength);
        end

        function updateInitialGuess(obj,x0)
            obj.initialControlGuess(:,1:obj.config.N-1) = obj.initialControlGuess(:,2:obj.config.N);
            obj.initialControlGuess(:,obj.config.N) = obj.initialControlGuess(:,obj.config.N-1);
            
            obj.initialStateGuess(:,1) = x0;
            obj.initialStateGuess(:,2:obj.config.N) = obj.initialStateGuess(:,3:obj.config.N+1);
            obj.initialStateGuess(:,obj.config.N+1) = full(obj.ode4(obj.initialStateGuess(:,obj.config.N),obj.initialControlGuess(:,obj.config.N)));

            obj.unwrapInitialGuess();
            obj.fillTrackCenterCoordinates();
        end

        function generateNewInitialGuess(obj,x0)
            obj.initialStateGuess = zeros(obj.config.NX,obj.config.N+1);
            obj.initialControlGuess = zeros(obj.config.NU,obj.config.N);

            obj.initialStateGuess(:,1) = x0;
            obj.initialStateGuess(obj.config.siIndex.vx,2:obj.config.N+1) = x0(4);
            obj.initialStateGuess(obj.config.siIndex.vs,1:obj.config.N+1) = x0(4);

            for i = 2:obj.config.N+1
              obj.initialStateGuess(obj.config.siIndex.s,i) =...
                obj.initialStateGuess(obj.config.siIndex.s,i - 1) + obj.ts * obj.initialStateGuess(obj.config.siIndex.vs,i - 1);
              trackPosI = obj.track.centerLine.getPosition(obj.initialStateGuess(obj.config.siIndex.s,i));
              trackdPosI = obj.track.centerLine.getDerivative(obj.initialStateGuess(obj.config.siIndex.s,i));
              obj.initialStateGuess(obj.config.siIndex.x,i) = trackPosI(1);
              obj.initialStateGuess(obj.config.siIndex.y,i) = trackPosI(2);
              obj.initialStateGuess(obj.config.siIndex.yaw,i) = atan2(trackdPosI(2), trackdPosI(1));
            end
            obj.unwrapInitialGuess();
            obj.fillTrackCenterCoordinates();
            obj.validInitialGuess = true;
        end

        function unwrapInitialGuess(obj)
            for i = 2:obj.config.N+1
              if (obj.initialStateGuess(obj.config.siIndex.yaw,i) - obj.initialStateGuess(obj.config.siIndex.yaw,i - 1)) < -pi
                obj.initialStateGuess(obj.config.siIndex.yaw,i) = obj.initialStateGuess(obj.config.siIndex.yaw,i) + 2.0 * pi;
              end
              if (obj.initialStateGuess(obj.config.siIndex.yaw,i) - obj.initialStateGuess(obj.config.siIndex.yaw,i - 1)) > pi
                obj.initialStateGuess(obj.config.siIndex.yaw,i) = obj.initialStateGuess(obj.config.siIndex.yaw,i) - 2.0 * pi;
               end
            end

            trackLength = obj.track.centerLine.getLength();
            lapLength = trackLength/2;
            if obj.initialStateGuess(obj.config.siIndex.s,1) > lapLength/2
                for i = 2:obj.config.N+1
                    obj.initialStateGuess(obj.config.siIndex.s,i) = rem(obj.initialStateGuess(obj.config.siIndex.s,i),trackLength);
                end
            else
                for i = 2:obj.config.N+1
                    obj.initialStateGuess(obj.config.siIndex.s,i) = rem(obj.initialStateGuess(obj.config.siIndex.s,i),lapLength);
                end
            end
        end

        function fillTrackCenterCoordinates(obj)
            for i = 1:obj.config.N+1
                
                s0 = obj.initialStateGuess(7,i);
                xTrack = full(obj.track.centerLineInterpolation.x(s0));
                yTrack = full(obj.track.centerLineInterpolation.y(s0));
                phiTrack = full(atan2(obj.track.centerLineDerivativesInterpolation.y(s0),obj.track.centerLineDerivativesInterpolation.x(s0)));
     
                obj.args.p(obj.config.NX+4*i-3,1) = xTrack;
                obj.args.p(obj.config.NX+4*i-2,1) = yTrack;
                obj.args.p(obj.config.NX+4*i-1,1) = phiTrack;
                obj.args.p(obj.config.NX+4*i,1) = s0;
            end            
        end

        function xNext = ode4(obj,state,input)
            % 4th order Runge Kutta (RK4) implementation
            % 4 evaluation points of continuous dynamics
            % evaluating the 4 points
            k1 = obj.f(state, input);
            k2 = obj.f(state + obj.ts / 2.0 * k1, input);
            k3 = obj.f(state + obj.ts / 2.0 * k2, input);
            k4 = obj.f(state + obj.ts * k3, input);
            % combining to give output
            xNext = state + obj.ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
        end
    end
end

