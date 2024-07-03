classdef Parameters
    %PARAMETERS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        bounds
        boundsTRO
        mpcModel
        costs
        car
        tire
        normalization
        config
    end

    properties (Access = private)
        d_config
    end
    
    methods
        function obj = Parameters(config)
            obj.d_config = config;
            %load bounds
            fname = 'bounds.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.bounds = jsondecode(str);

            obj.d_config = config;
            %load boundsTRO
            fname = 'boundsTRO.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.boundsTRO = jsondecode(str);

            %load mpcmodelparameters
            fname = 'model.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.mpcModel = jsondecode(str);

            %load costs
            fname = 'cost.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.costs = jsondecode(str);

            %load car parameters
            fname = 'car.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.car = jsondecode(str);

            %load tire coefficients
            fname = 'tire.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.tire = jsondecode(str);

            %load noermalization parameters
            fname = 'normalization.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            decoded = jsondecode(str);

            normStateVec = [decoded.x, decoded.y, decoded.yaw, decoded.vx, decoded.vy,...
                decoded.r, decoded.s, decoded.throttle, decoded.steeringAngle,...
                decoded.brakes, decoded.vs];

            obj.normalization.tX = diag(normStateVec);

            for i = 1:obj.d_config.NX
                obj.normalization.tXInv(i,i) = 1/obj.normalization.tX(i,i);
            end

            normInputVec = [decoded.dThrottle, decoded.dSteeringAngle,...
                decoded.dBrakes, decoded.dVs];

            obj.normalization.tU = diag(normInputVec);

            for i = 1:obj.d_config.NU
                obj.normalization.tUInv(i,i) = 1/obj.normalization.tU(i,i);
            end

            obj.normalization.tS = eye(2);
            obj.normalization.tSInv = eye(2);

            %load config
            fname = 'config.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.config = jsondecode(str);
        end
    end
end

