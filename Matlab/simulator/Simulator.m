classdef Simulator < handle    
    properties (Access = private)
        car;
        tire;
        f;
        model;
        config;
    end
    
    methods (Access = public)

        function obj = Simulator(config,car,tire)
            import casadi.*;

            obj.car = car;
            obj.tire = tire;
            obj.model = Model(car,tire);
            obj.config = config;

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
            wf = SX.sym('wf');
            wr = SX.sym('wr');


            state = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs];

            dThrottle = SX.sym('dThrottle');
            dSteeringAngle = SX.sym('dSteeringAngle');
            dBrakes = SX.sym('dBrakes');
            dVs = SX.sym('dVs');

            input = [dThrottle,dSteeringAngle,dBrakes,dVs];

            if strcmp(config.simulator,'kinematic')
                rhs = obj.model.initKinematicModel(state,input);
            elseif strcmp(config.simulator,'dynamic')
                rhs = obj.model.initDynamicModel(state,input);
            elseif strcmp(config.simulator,'simple_dynamic')
                rhs = obj.model.initSimpleDynamicModel(state,input);
            elseif strcmp(config.simulator,'combined_dynamic')
                state = [x;y;yaw;vx;vy;r;s;throttle;steeringAngle;brakes;vs;wf;wr];
                rhs = obj.model.initCombinedSlipDynamicModel(state,input);
            end

            obj.f = Function('f',{state,input},{rhs});
        end

        function xNext = ode4(obj,state,input,ts)
            % 4th order Runge Kutta (RK4) implementation
            % 4 evaluation points of continuous dynamics
            % evaluating the 4 points
            k1 = obj.f(state, input);
            k2 = obj.f(state + ts / 2.0 * k1, input);
            k3 = obj.f(state + ts / 2.0 * k2, input);
            k4 = obj.f(state + ts * k3, input);
            % combining to give output
            xNext = state + ts * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
        end

        function xNext = simTimeStep(obj,x,u,ts)
            xNext = x;
            integrationSteps = cast(ts/0.001,'int64');
            
            for i = 1:integrationSteps
                xNext = obj.ode4(xNext,u,0.001).full();
            end
            xNext = obj.unwrapState(xNext);
        end

        function x0 = unwrapState(obj,x0)
            if x0(3) > pi
              x0(obj.config.siIndex.yaw) = x0(obj.config.siIndex.yaw) - 2.0 * pi;
            end
            if x0(obj.config.siIndex.yaw) < -pi
              x0(obj.config.siIndex.yaw) = x0(obj.config.siIndex.yaw) + 2.0 * pi;
            end
        end
    end
end

