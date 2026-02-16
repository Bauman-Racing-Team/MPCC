classdef CubicSpline < handle
    %CUBIC_SPLINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        d_dataSet
        d_splineParameters
        d_splineData
    end

    methods (Access = public)
        function obj = CubicSpline()
            obj.d_splineParameters = SplineParams();
            obj.d_splineData = SplineData();
        end

        function genSpline(obj,xIn,yIn,isRegular)
        % given x and y data generate spline
        % special case for regular or irregular spaced data points in x
        % if regular the spacing in x is given by deltaX
        
        % store data in data struct
          if isRegular
            deltaX = xIn(2) - xIn(1);
            obj.setRegularData(xIn, yIn, deltaX);
          else
            obj.setData(xIn, yIn);
          end
          % given data compute spline parameters
        
          % TODO if success is false call exception
        
          % bool success = compSplineParams();
        
          obj.compSplineParams();
        end
        
        function point = getPoint(obj, x)
        % evaluation of spline a x
          % wrape input to data -> x data needs start at 0 and contain end point!!!
          x = obj.unwrapInput(x);
          % compute index
          index = obj.getIndex(x);
          % access previous points
          xI = obj.d_splineData.xData(index);
          % compute diff to point and it's powers
          dx = x - xI;
          dx2 = dx * dx;
          dx3 = dx * dx2;
          % return spline value y = a + b dx + c dx^2 + d dx^3
          point = obj.d_splineParameters.a(index) + obj.d_splineParameters.b(index) * dx...
                 + obj.d_splineParameters.c(index) * dx2 + obj.d_splineParameters.d(index) * dx3;
        end

        function derivative = getDerivative(obj,x)
        % evaluate first derivative of spline
          % identical to noram spline with y' = b + 2 c dx + 3 d dx^2
          x = obj.unwrapInput(x);
          index = obj.getIndex(x);
          xI = obj.d_splineData.xData(index);
        
          dx = x - xI;
          dx2 = dx * dx;
          % y' = b + 2 c dx + 3 d dx^2
          derivative = obj.d_splineParameters.b(index) + 2.0 * obj.d_splineParameters.c(index) * dx...
                 + 3.0 * obj.d_splineParameters.d(index) * dx2;
        end

        function secondDerivative = getSecondDerivative(obj,x)
          % evaluate second derivative of spline
          % identical to noram spline with y' = 2 c + 6 d dx
          x = obj.unwrapInput(x);
          index = obj.getIndex(x);
          xI = obj.d_splineData.xData(index);
        
          dx = x - xI;
          % y' = 2 c + 6 d dx
          secondDerivative = 2.0 * obj.d_splineParameters.c(index) + 6.0 * obj.d_splineParameters.d(index) * dx;
        end
    end

    methods (Access = private)
        function setRegularData(obj,xIn,yIn,deltaX)
            % if x and y have same length, stare given data in spline data struct
              if size(xIn,1) == size(yIn,1)
                obj.d_splineData.xData = xIn;
                obj.d_splineData.yData = yIn;
                obj.d_splineData.nPoints = size(xIn,1);
                obj.d_splineData.isRegular = true;
                obj.d_splineData.deltaX = deltaX;
            
                obj.d_dataSet = true;
              else
                disp('input data does not have the same length');
              end
        end
        
        function setData(obj,xIn,yIn)
            % if x and y have same length, stare given data in spline data struct
              if size(xIn,1) == size(yIn,1)
                obj.d_splineData.xData = xIn;
                obj.d_splineData.yData = yIn;
                obj.d_splineData.nPoints = size(xIn,1);
                obj.d_splineData.isRegular = false;
                obj.d_splineData.deltaX = 0;
                xMap = zeros(size(xIn,1),2);
                for i = 1:size(xIn,1)
                  xMap(i,1:2) = [xIn(i), i];
                end
                xMap = sortrows(xMap);
                obj.d_splineData.xMap = array2table(xMap);
            
                obj.d_dataSet = true;
              else
                disp('input data does not have the same length');
              end
        end

        function compSplineParams(obj)
            % compute spline parameters parameters
            % code is a replica of the wiki code
              % spline parameters from parameter struct initialized to zero
              obj.d_splineParameters.a = zeros(obj.d_splineData.nPoints,1);
              obj.d_splineParameters.b = zeros(obj.d_splineData.nPoints - 1,1);
              obj.d_splineParameters.c = zeros(obj.d_splineData.nPoints,1);
              obj.d_splineParameters.d = zeros(obj.d_splineData.nPoints - 1,1);
            
              % additional variables used to compute a,b,c,d
              mu = zeros(obj.d_splineData.nPoints - 1,1);
              h = zeros(obj.d_splineData.nPoints - 1,1);
              alpha = zeros(obj.d_splineData.nPoints - 1,1);
              l = zeros(obj.d_splineData.nPoints,1);
              z = zeros(obj.d_splineData.nPoints,1);
            
              % a is equal to y data
              obj.d_splineParameters.a = obj.d_splineData.yData;
              % compute h as diff of x data
              for i = 1:(obj.d_splineData.nPoints - 1)
                h(i) = obj.d_splineData.xData(i + 1) - obj.d_splineData.xData(i);
              end
              % compute alpha
              for i = 2:(obj.d_splineData.nPoints - 1)
                alpha(i) = 3.0 / h(i) * (obj.d_splineParameters.a(i + 1) - obj.d_splineParameters.a(i))...
                           -3.0 / h(i - 1) * (obj.d_splineParameters.a(i) - obj.d_splineParameters.a(i - 1));
              end
            
              % compute l, mu, and z
              l(1) = 1.0;
              mu(1) = 0.0;
              z(1) = 0.0;
              for i = 2:(obj.d_splineData.nPoints - 1)
                l(i) = 2.0 * (obj.d_splineData.xData(i + 1) - obj.d_splineData.xData(i - 1)) - h(i - 1) * mu(i - 1);
                mu(i) = h(i) / l(i);
                z(i) = (alpha(i) - h(i - 1) * z(i - 1)) / l(i);
              end
              %l(obj.d_splineData.nPoints - 1) = 1.0;
              z(obj.d_splineData.nPoints) = 0.0;
            
              % compute b,c,d data given the previous work
              obj.d_splineParameters.c(obj.d_splineData.nPoints) = 0.0;
            
              for i = (obj.d_splineData.nPoints - 1):-1:1
                obj.d_splineParameters.c(i) = z(i) - mu(i) * obj.d_splineParameters.c(i + 1);
                obj.d_splineParameters.b(i) =...
                  (obj.d_splineParameters.a(i + 1) - obj.d_splineParameters.a(i)) / h(i)...
                  -(h(i) * (obj.d_splineParameters.c(i + 1) + 2.0 * obj.d_splineParameters.c(i))) / 3.0;
                obj.d_splineParameters.d(i) =...
                  (obj.d_splineParameters.c(i + 1) - obj.d_splineParameters.c(i)) / (3.0 * h(i));
              end
        end
        
        function index = getIndex(obj,x)
              % given a x value find the closest point in the spline to evaluate it
              % special case if x is regularly space
              % assumes wrapped data!
            
              % if special case of end points
              if x == obj.d_splineData.xData(obj.d_splineData.nPoints)
                index = obj.d_splineData.nPoints;
                return;
              end
              % if regular index can be found by rounding
              if obj.d_splineData.isRegular
                index = cast(1 + floor(x / obj.d_splineData.deltaX), "int64");
                return;
              % if irregular index need to be searched
              else
                idx = find(obj.d_splineData.xMap.xMap1 > x,1);
                if isempty(idx)
                  index  = -1;
                  return
                else
                  val = obj.d_splineData.xMap.xMap2(idx);
                  index = val - 1;
                  return;
                end
             end
        end

        function input = unwrapInput(obj,x)
            xMax = obj.d_splineData.xData(obj.d_splineData.nPoints);
            input = x - xMax * floor(x / xMax);
        end
    end
end

