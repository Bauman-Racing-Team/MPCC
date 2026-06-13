classdef Track  < handle  
    properties (Access = public)
        x
        y

        xInner
        yInner

        xOuter
        yOuter
    end
    
    methods (Access = public)
        function obj = Track(conesBlue,conesYellow,centerLine)
            obj.xOuter = [conesBlue(:,1);conesBlue(1,1)];
            obj.yOuter = [conesBlue(:,2);conesBlue(1,2)];
            obj.xInner = [conesYellow(:,1);conesYellow(1,1)];
            obj.yInner = [conesYellow(:,2);conesYellow(1,2)];
            if nargin > 2
                obj.x = centerLine(:,1);
                obj.y = centerLine(:,2);
            else
                obj.genCenterLine();
            end
        end
        
        function genCenterLine(obj)
            oSize = size(obj.xOuter,1);
            obj.x = zeros(oSize,1);
            obj.y = zeros(oSize,1);
            
            for i = 1:oSize
                innerConeIndex = obj.getInnerConeIndex(i);
                obj.x(i) = (obj.xOuter(i) + obj.xInner(innerConeIndex)) / 2.0;
                obj.y(i) = (obj.yOuter(i) + obj.yInner(innerConeIndex)) / 2.0;
            end
        end

        function innerConeIndex = getInnerConeIndex(obj,coneBlueIndex)
            ySize = size(obj.xInner,1);
            d = inf;
            for i = 1:ySize
                newD = hypot(obj.xOuter(coneBlueIndex) - obj.xInner(i),obj.yOuter(coneBlueIndex) - obj.yInner(i));
                if newD < d
                    d = newD;
                    innerConeIndex = i;
                end
            end
        end
    end
end

