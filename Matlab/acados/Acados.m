classdef Acados < handle
    %ACADOS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        config
        parameters
        ts
        car

        carModel

        track

        ocpModel
        ocpOpts
        ocp

        p % parameters vector: [Xcen, Ycen, xRef, yRef, thetaRef];

        state
        input

        initialStateGuess
        initialControlGuess
        validInitialGuess
        n_non_solves_
        f
        paramVec
    end
    
    methods (Access = public)
        function obj = Acados(config,parameters)
            check_acados_requirements();

            obj.config = config;
            obj.parameters = parameters;
            obj.ts = parameters.config.ts;
            obj.car = parameters.car;

            obj.carModel = Model(obj.car,parameters.tire);

            obj.ocpModel = acados_ocp_model();
            obj.ocpOpts = acados_ocp_opts();

            obj.validInitialGuess = false;
            obj.n_non_solves_ = 0;
            obj.track.centerLine = ArcLengthSpline(config,parameters.mpcModel);
            obj.track.outerBorder = ArcLengthSpline(config,parameters.mpcModel);
            obj.track.innerBorder = ArcLengthSpline(config,parameters.mpcModel);

            obj.paramVec = zeros(11,obj.config.N+1);
        end

        function setTrack(obj,track)
            import casadi.*;

            obj.track.centerLine.gen2DSpline([track.x;track.x],[track.y;track.y]);
            obj.track.outerBorder.gen2DSpline([track.xOuter;track.xOuter],[track.yOuter;track.yOuter]);
            obj.track.innerBorder.gen2DSpline([track.xInner;track.xInner],[track.yInner;track.yInner]);

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

            %obj.initMPC();
        end

        function track = getTrack(obj)
            track = obj.track.centerLine;
        end

        function initMPC(obj)
            obj.initOcpModel(); 
            obj.setBounds();
            obj.setOCPOpts();
        end

        function initOcpModel(obj)
            import casadi.*;

            obj.ocpModel.set('name','acados_mpcc');
            obj.ocpModel.set('T', obj.config.N*obj.ts);

            model = getModel(obj.parameters);

            obj.ocpModel.set('sym_x',model.x);
            obj.ocpModel.set('sym_u',model.u);
            obj.ocpModel.set('sym_p',model.p);
            obj.ocpModel.set('sym_z',model.z);
            obj.ocpModel.set('sym_xdot',model.xdot);
            
            %obj.ocpModel.set('dyn_type','implicit');
            %obj.ocpModel.set('dyn_expr_f',model.f_impl_expr);

            obj.ocpModel.set('dyn_type','explicit');
            obj.ocpModel.set('dyn_expr_f',model.f_expl_expr);
            
            obj.ocpModel.set('cost_type','ext_cost');
            obj.ocpModel.set('cost_type_e','ext_cost');

            obj.ocpModel.set('cost_ext_fun_type','casadi');
            obj.ocpModel.set('cost_ext_fun_type_e','casadi');
            
            obj.ocpModel.set('constr_x0', [0,0,0,0,0,0,0,0,0,0,0]);
            obj.ocpModel.set('cost_expr_ext_cost',model.cost_expr_ext_cost);
            obj.ocpModel.set('cost_expr_ext_cost_e',model.cost_expr_ext_cost_e);

            obj.ocpModel.set('constr_expr_h',model.constr_expr_h);

            obj.f = model.f;
        end

        function setBounds(obj)
            nbx = 9;
            jbx = zeros(nbx,obj.config.NX);

            jbx(1,3) = 1;
            jbx(2,4) = 1;
            jbx(3,5) = 1;
            jbx(4,6) = 1;
            jbx(5,7) = 1;
            jbx(6,8) = 1;
            jbx(7,9) = 1;
            jbx(8,10) = 1;
            jbx(9,11) = 1;

            obj.ocpModel.set('constr_Jbx',jbx);

            obj.ocpModel.set('constr_lbx',[ ...
                                            obj.parameters.bounds.lowerStateBounds.yawL, ...
                                            obj.parameters.bounds.lowerStateBounds.vxL, ...
                                            obj.parameters.bounds.lowerStateBounds.vyL, ...
                                            obj.parameters.bounds.lowerStateBounds.rL, ...
                                            obj.parameters.bounds.lowerStateBounds.sL, ...
                                            obj.parameters.bounds.lowerStateBounds.throttleL, ...
                                            obj.parameters.bounds.lowerStateBounds.steeringAngleL, ...
                                            obj.parameters.bounds.lowerStateBounds.brakesL, ...
                                            obj.parameters.bounds.lowerStateBounds.vsL]);

            obj.ocpModel.set('constr_ubx',[ ...
                                            obj.parameters.bounds.upperStateBounds.yawU, ...
                                            obj.parameters.bounds.upperStateBounds.vxU, ...
                                            obj.parameters.bounds.upperStateBounds.vyU, ...
                                            obj.parameters.bounds.upperStateBounds.rU, ...
                                            obj.parameters.bounds.upperStateBounds.sU, ...
                                            obj.parameters.bounds.upperStateBounds.throttleU, ...
                                            obj.parameters.bounds.upperStateBounds.steeringAngleU, ...
                                            obj.parameters.bounds.upperStateBounds.brakesU, ...
                                            obj.parameters.bounds.upperStateBounds.vsU]);

            nbu = 4;
            jbu = zeros(nbu,obj.config.NU);

            jbu(1,1) = 1;
            jbu(2,2) = 1;
            jbu(3,3) = 1;
            jbu(4,4) = 1;

            obj.ocpModel.set('constr_Jbu',jbu);

            obj.ocpModel.set('constr_lbu',[ ...
                                            obj.parameters.bounds.lowerInputBounds.dThrottleL, ...
                                            obj.parameters.bounds.lowerInputBounds.dSteeringAngleL, ...
                                            obj.parameters.bounds.lowerInputBounds.dBrakesL, ...
                                            obj.parameters.bounds.lowerInputBounds.dVsL]);

            obj.ocpModel.set('constr_ubu',[ ...
                                            obj.parameters.bounds.upperInputBounds.dThrottleU, ...
                                            obj.parameters.bounds.upperInputBounds.dSteeringAngleU, ...
                                            obj.parameters.bounds.upperInputBounds.dBrakesU, ...
                                            obj.parameters.bounds.upperInputBounds.dVsU]);

            % track, front slip angle, rear slip angle constraints
            constr_lh = [];
            constr_uh = [];

            % front slip angle constraint
            constr_lh = [constr_lh,-obj.parameters.mpcModel.maxAlpha];
            %
            constr_uh = [constr_uh,obj.parameters.mpcModel.maxAlpha];

            % rear slip angle constraint
            constr_lh = [constr_lh,-obj.parameters.mpcModel.maxAlpha];
            %
            constr_uh = [constr_uh,obj.parameters.mpcModel.maxAlpha];

            % track constraint bounds
            constr_lh = [constr_lh,0];
            
            constr_uh = [constr_uh,obj.parameters.mpcModel.rOut^2];

            % friction ellipse constraint bounds
            constr_lh = [constr_lh,0,0];
            
            constr_uh = [constr_uh,1,1];
           
            obj.ocpModel.set('constr_lh',constr_lh);
            obj.ocpModel.set('constr_uh',constr_uh);

            % Coeffs for soft constraints penalization
            scQuadTrack = obj.parameters.costs.scQuadTrack;
            scQuadTire = obj.parameters.costs.scQuadTire;
            scQuadAlpha = obj.parameters.costs.scQuadAlpha;
            scLinTrack = obj.parameters.costs.scLinTrack;
            scLinTire = obj.parameters.costs.scLinTire;
            scLinAlpha = obj.parameters.costs.scLinAlpha;
                        
            % quadratic part
            Z = diag([scQuadAlpha, scQuadAlpha, scQuadTrack,scQuadTire,scQuadTire]);
            % linear part
            z = [scLinAlpha; scLinAlpha; scLinTrack;scLinTire;scLinTire];

            jsh = eye(obj.config.NS); % all constraints are softened
            
            % Coeffs for track onyl 
%             Z = diag([obj.parameters.costs.scQuadTrack]);
            % linear part
%             z = [obj.parameters.costs.scLinTrack];

%             jsh = eye(1); % all constraints are softened
            
            % Coeffs for slip angles only
%             Z = diag([obj.parameters.costs.scQuadAlpha, obj.parameters.costs.scQuadAlpha]);
            % linear part
%             z = [obj.parameters.costs.scLinAlpha; obj.parameters.costs.scLinAlpha];

%             jsh = eye(2); % all constraints are softened
            
            obj.ocpModel.set('constr_Jsh',jsh);
            obj.ocpModel.set('cost_Z',Z);
            obj.ocpModel.set('cost_z',z);
        end

        function setOCPOpts(obj)
            obj.ocpOpts.set('param_scheme_N', obj.config.N);
            obj.ocpOpts.set('nlp_solver', 'sqp_rti'); % sqp, sqp_rti 
            obj.ocpOpts.set('nlp_solver_exact_hessian', 'false'); % false=gauss_newton, true=exact
            obj.ocpOpts.set('sim_method', 'erk'); % erk, irk, irk_gnsf
            obj.ocpOpts.set('sim_method_num_stages', 4);
            obj.ocpOpts.set('sim_method_num_steps', 3);
            obj.ocpOpts.set('qp_solver', 'partial_condensing_hpipm');
            obj.ocpOpts.set('qp_solver_cond_N', 5);
            obj.ocpOpts.set('qp_solver_iter_max', 55); %51 for FSI;55 for FSG;
            obj.ocpOpts.set('nlp_solver_tol_stat', 1e-4);
            obj.ocpOpts.set('nlp_solver_tol_eq', 1e-4);
            obj.ocpOpts.set('nlp_solver_tol_ineq', 1e-4);
            obj.ocpOpts.set('nlp_solver_tol_comp', 1e-4);

            obj.ocp = acados_ocp(obj.ocpModel, obj.ocpOpts);
        end

        function sol = runMPC(obj,x0)
            x0(obj.config.siIndex.s) = obj.track.centerLine.projectOnSpline(vectorToState(x0));
            x0 = obj.unwrapState(x0);
            
            if obj.validInitialGuess
              obj.updateInitialGuess(x0);
            else
              obj.generateNewInitialGuess(x0);
            end
            
            n_non_solves_sqp_ = 0;
            for i = 1:obj.parameters.config.nSqp

                obj.fillParametersVector();
                
                obj.ocp.set('constr_x0', obj.initialStateGuess(:,1))
    
                obj.ocp.set('init_x', obj.initialStateGuess);
                obj.ocp.set('init_u', obj.initialControlGuess);
                obj.ocp.set('init_pi', zeros(obj.config.NX, obj.config.N));
    
                obj.ocp.solve();
    
                status = obj.ocp.get('status');
    
                obj.initialStateGuess = obj.ocp.get('x');
                obj.initialControlGuess = obj.ocp.get('u');
    
                sol = MpcReturn;
                if status ~= 0
                    n_non_solves_sqp_ = n_non_solves_sqp_+1;
                end
                if status == 0||status == 2||status == 3
                    sol.x0 = obj.initialStateGuess(:,1);
                    sol.u0 = obj.initialControlGuess(:,1);
                    sol.mpcHorizon.states = obj.initialStateGuess;
                    sol.mpcHorizon.inputs = obj.initialControlGuess;
                    sol.mpcHorizon.slacks = obj.getSlacks();
                    sol.solverStatus = status;
                    sol.cost = obj.ocp.get_cost;
                    sol.circlesCenters = obj.getConstraintsCirclesCenters();
                end
                max_error = max(obj.parameters.config.nSqp-1,1);
                if n_non_solves_sqp_ >= max_error
                    obj.n_non_solves_ = obj.n_non_solves_+1;
                else
                    obj.n_non_solves_ = 0;
                end
                if obj.n_non_solves_ >= obj.parameters.config.nReset
                    obj.validInitialGuess = false;
                end
            end
        end

        function centers = getConstraintsCirclesCenters(obj)
            centers = zeros(2,obj.config.N+1);
            newStateGuess = obj.ocp.get('x');
            for i = 1:obj.config.N+1
                
                xTrack = obj.paramVec(1,i);
                yTrack = obj.paramVec(2,i);
                %phiTrack = obj.paramVec(3,i);
                %s0 = obj.paramVec(4,i);

                %centers(1,i) = xTrack + (newStateGuess(7,i)-s0)*cos(phiTrack);
                %centers(2,i) = yTrack + (newStateGuess(7,i)-s0)*sin(phiTrack);

                centers(1,i) = xTrack;
                centers(2,i) = yTrack;
            end
        end

        function slacks = getSlacks(obj)
            slacks.upper = zeros(obj.config.NS,obj.config.N);
            slacks.lower = zeros(obj.config.NS,obj.config.N);
            for i = 1:obj.config.N
                slacks.upper(:,i) = obj.ocp.get('su',i-1);
                slacks.lower(:,i) = obj.ocp.get('sl',i-1);
            end
        end

        function fillParametersVector(obj)
            for i = 1:obj.config.N+1
                qC = obj.parameters.costs.qC;
                qL = obj.parameters.costs.qL;
                qVs = obj.parameters.costs.qVs;
                rThrottle = obj.parameters.costs.rThrottle;
                rSteeringAngle = obj.parameters.costs.rSteeringAngle;
                rBrakes = obj.parameters.costs.rBrakes;
                rVs = obj.parameters.costs.rVs;

                s0 = obj.initialStateGuess(7,i);
                
                xTrack = full(obj.track.centerLineInterpolation.x(s0));
                yTrack = full(obj.track.centerLineInterpolation.y(s0));

                phiTrack = full(atan2(obj.track.centerLineDerivativesInterpolation.y(s0),obj.track.centerLineDerivativesInterpolation.x(s0)));

                obj.ocp.set('p',[xTrack;yTrack;phiTrack;s0;qC;qL;qVs;rThrottle;rSteeringAngle;rBrakes;rVs],i-1);
                obj.paramVec(:,i) = [xTrack;yTrack;phiTrack;s0;qC;qL;qVs;rThrottle;rSteeringAngle;rBrakes;rVs];
                                                                    
            end            
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

            for i = 1:obj.config.N+1
                obj.initialStateGuess(4,i) = max(obj.initialStateGuess(4,i),obj.parameters.mpcModel.vxMin);
                obj.initialStateGuess(11,i) = max(obj.initialStateGuess(11,i),obj.parameters.mpcModel.vxMin);
            end

            obj.unwrapInitialGuess();
        end

        function generateNewInitialGuess(obj,x0)
            obj.initialStateGuess = zeros(obj.config.NX,obj.config.N+1);
            obj.initialControlGuess = zeros(obj.config.NU,obj.config.N);

            obj.initialStateGuess(:,1) = x0;
            obj.initialStateGuess(obj.config.siIndex.vx,1:obj.config.N+1) = max(x0(4),obj.parameters.mpcModel.vxMin);
            obj.initialStateGuess(obj.config.siIndex.vs,1:obj.config.N+1) = max(x0(4),obj.parameters.mpcModel.vxMin);

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

        function [cost_expr_ext_cost,cost_expr_ext_cost_e] = computeCost(obj)
            % Coeffs for control inputs penalization
            rThrottle = obj.paramVec(8,1);
            rSteeringAngle = obj.paramVec(9,1);
            rBrakes = obj.paramVec(10,1);
            rVs = obj.paramVec(11,1);

            R = diag([rThrottle, ...
                      rSteeringAngle, ...
                      rBrakes, ...
                      rVs]);
            
            cost_expr_ext_cost = 0;

            % stages cost
            for i = 1:obj.config.N
                inputVec = obj.initialControlGuess(:,i);
                cost_expr_ext_cost = cost_expr_ext_cost + obj.costWOControl(i) + inputVec'*R*inputVec;
            end

            % terminal cost
            cost_expr_ext_cost_e = obj.costWOControl(obj.config.N+1);
        end

        function cost = costWOControl(obj,i)
            % Coeffs for laf and contouring errors penallization

            xTrack = obj.paramVec(1,i);
            yTrack = obj.paramVec(2,i);
            phiTrack = obj.paramVec(3,i);
            s0 = obj.paramVec(4,i);                
            qC = obj.paramVec(5,i);
            qL = obj.paramVec(6,i);
            qVs = obj.paramVec(7,i);
           
            Q = diag([qC,qL]);

            x = obj.initialStateGuess(1,i);
            y = obj.initialStateGuess(2,i);
            s = obj.initialStateGuess(7,i);
            vs = obj.initialStateGuess(11,i);

            xRef = xTrack + (s-s0)*cos(phiTrack);
            yRef = yTrack + (s-s0)*sin(phiTrack);

            % contouring error
            ec = -cos(phiTrack)*(yRef-y)+sin(phiTrack)*(xRef-x);
            % lag error
            el = cos(phiTrack)*(xRef-x)+sin(phiTrack)*(yRef-y);
           
            error = [ec;el];

            cost = error'*Q*error - qVs*vs;
        end
    end
end

