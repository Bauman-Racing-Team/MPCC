classdef Parameters
    %PARAMETERS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = public)
        bounds
        mpcModel
        costs
        car
        tire
        config
    end

    properties (Access = private)
        d_config
    end
    
    methods
        function obj = Parameters(vehicle, config)
            obj.d_config = config;
            %load bounds
            fname = 'bounds.json';
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.bounds = jsondecode(str);

            %load mpcmodelparameters
            fname = vehicle + "/model.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.mpcModel = jsondecode(str);

            %load costs
            fname = vehicle + "/cost.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.costs = jsondecode(str);

            %load car parameters
            fname = vehicle + "/car.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.car = jsondecode(str);

            %load tire coefficients
            fname = vehicle + "/tire.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.tire = jsondecode(str);

            %load config
            fname = vehicle + "/config.json";
            fid = fopen(fname);
            raw = fread(fid,inf);
            str = char(raw');
            fclose(fid);
            obj.config = jsondecode(str);
        end
    end
end

