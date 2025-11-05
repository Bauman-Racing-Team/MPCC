classdef MpcReturn
    properties (Access = public)
        x0
        u0
        mpcHorizon
        solverStatus
        cost
        circlesCenters
        minDistsFromBorderToCarCenter
        bordersCoordinates
    end

    methods (Access = public)
        function obj = MpcReturn(x0,u0,mpcHorizon,solverStatus,cost,circlesCenters,minDistsFromBorderToCarCenter,bordersCoordinates)
            if nargin > 0
                obj.x0 = x0;
                obj.u0 = u0;
                obj.mpcHorizon = mpcHorizon;
                obj.solverStatus = solverStatus;
                obj.cost = cost;
                obj.circlesCenters = circlesCenters;
                obj.minDistsFromBorderToCarCenter = minDistsFromBorderToCarCenter;
                obj.bordersCoordinates = bordersCoordinates;
            end
        end
    end

end

