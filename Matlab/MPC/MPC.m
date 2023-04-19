classdef MPC
    %MPC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        d_ts
        d_validInitialGuess
    
        d_stages
        d_initialGuess
        d_optimalSolution
    
        d_nSqp
        d_sqpMixing
        d_nNonSolves
        d_nNoSolvesSqp
        d_nReset
    
        d_model
        d_cost
        d_constraints
        d_track
        d_bounds
        d_normalization
        d_mpcModel
        d_car
        d_solverInterface
    end
    
    methods (Access = public)
        function obj = MPC(configPath,inputArg2)
            %MPC Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function mpcReturn = runMPC(obj,x0)
        end

        function setTrack(x,y)
        end
    
        function getTrack()
        end
    end

    methods (Access = private)

        function obj = setMPCProblem(obj)
            for i = 1:N
                obj = obj.setStage(obj.d_initialGuess(1,i).xk, obj.d_initialGuess(1,i).uk, obj.d_initialGuess(1,i + 1).xk, i);
            end
        end

        function obj = setStage(obj,xk,uk,xk1,timeStep)
              obj.d_stages(1,timeStep).nx = NX;
              obj.d_stages(1,timeStep).nu = NU;
            
              if timeStep == 0
                obj.d_stages(1,timeStep).ng = 0;
                obj.d_stages(1,timeStep).ns = 0;
              else
                obj.d_stages(1,timeStep).ng = NPC;
                obj.d_stages(1,timeStep).ns = NS;
              end
            
              vxZero = obj.d_mpcModel.vx_zero;
            
              xkNz = xk;
              xkNz.vxNonZero(vxZero);
            
              xk1Nz = xk1;
              xk1Nz.vxNonZero(vxZero);
            
              obj.d_stages(1,timeStep).costMat = obj.normalizeCost(obj.d_cost.getCost(obj.d_track, xkNz, uk, timeStep));
              obj.d_stages(1,timeStep).linModel = obj.normalizeDynamics(obj.d_model.getLinModel(xkNz, uk, xk1Nz));
              obj.d_stages(1,timeStep).constrainsMat =...
                obj.normalizeCon(obj.d_constraints.getConstraints(obj.d_track, xkNz));
            
              obj.d_stages(1,timeStep).lBoundsX =...
                obj.d_normalization.tXInv * obj.d_bounds.getBoundsLX(xkNz);
              obj.d_stages(1,timeStep).uBoundsX =...
                obj.d_normalization.tXInv * obj.d_bounds.getBoundsUX(xkNz);
              obj.d_stages(1,timeStep).lBoundsU = obj.d_normalization.tUInv * obj.d_bounds.getBoundsLU(uk);
              obj.d_stages(1,timeStep).uBoundsU = obj.d_normalization.tUInv * obj.d_bounds.getBoundsUU(uk);
              obj.d_stages(1,timeStep).lBoundsS = obj.d_normalization.tSInv * obj.d_bounds.getBoundsLS();
              obj.d_stages(1,timeStep).uBoundsS = obj.d_normalization.tSInv * obj.d_bounds.getBoundsUS();
            
              obj.d_stages(1,timeStep).lBoundsX(siIndex.s) =...
                obj.d_normalization.tXInv(siIndex.s, siIndex.s) *...
                (-obj.d_model.sTrustRegion);  %*d_initialGuess[timeStep].xk.vs;
              obj.d_stages(1,timeStep).uBoundsX(siIndex.s) =...
                obj.d_normalization.tXInv(siIndex.s, siIndex.s) *...
                (obj.d_model.sTrustRegion);  %*d_initialGuess[timeStep].xk.vs;
        end


        function costMat = normalizeCost(obj,costMat)
              Q = obj.d_normalization.tX * costMat.Q * obj.d_normalization.tX;
              R = obj.d_normalization.tU * costMat.R * obj.d_normalization.tU;
              q = obj.d_normalization.tX * costMat.q;
              r = obj.d_normalization.tU * costMat.r;
              Z = obj.d_normalization.tS * costMat.Z * obj.d_normalization.tS;
              z = obj.d_normalization.tS * costMat.z;
              costMat = struct(Q, R, zeros(11,4), q, r, Z, z);
        end

        function linModelMat = normalizeDynamics(linModel)
              a =...
              obj.d_normalization.tXInv * linModel.a * d_normalizationParametersPtr->tX;
              b =...
              obj.d_normalization.tXInv * linModel.b * d_normalizationParametersPtr->tU;
              g = obj.d_normalization.tXInv * linModel.g;
              linModelMat = struct(a, b, g);
        end


        function conMat = normalizeCon(conMat)
              c = conMat.c * d_normalizationParametersPtr->tX;
              d = conMat.d * d_normalizationParametersPtr->tU;
              dl = conMat.dl;
              du = conMat.du;
              return {c, d, dl, du};
        end

        function deNormalizeSolution(solution)
        end

        function updateInitialGuess(x0)
        end

        function generateNewInitialGuess(const State &x0)
        end

        function unwrapInitialGuess()
        end

        function sqpSolutionUpdate(lastSolution,currentSolution)
        end
    end
end

