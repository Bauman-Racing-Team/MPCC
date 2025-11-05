classdef MpcReturn
    properties (Access = public)
        x0
        u0
        mpcHorizon
        solverStatus
        cost
        circlesCenters
        bordersCoordinates
    end

    methods (Access = public)
        function obj = MpcReturn(x0,u0,mpcHorizon,solverStatus,cost,circlesCenters,bordersCoordinates)
            if nargin > 0
                obj.x0 = x0;
                obj.u0 = u0;
                obj.mpcHorizon = mpcHorizon;
                obj.solverStatus = solverStatus;
                obj.cost = cost;
                obj.circlesCenters = circlesCenters;
                obj.bordersCoordinates = bordersCoordinates;
            end
        end
    end

end

