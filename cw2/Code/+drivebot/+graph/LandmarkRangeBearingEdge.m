% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % Q2b:
            % Complete implementation
            %warning('landmarkrangebearingedge:initialize:unimplemented', ...
                %'Implement the rest of this method for Q1b.');

            % Get the vehicle state vertex
            x = this.edgeVertices{1}.x;
            % Landmark vertex
            lm = zeros(2, 1);
            lm(1) = x(1) + this.z(1) * cos(this.z(2) + x(3));
            lm(2) = x(2) + this.z(1) * sin(this.z(2) + x(3));
            % Define the landmark vertex
            this.edgeVertices{2}.setEstimate(lm);
        end
        
        function computeError(this)

            % Q2b:
            % Complete implementation
            %warning('landmarkrangebearingedge:computeerror:unimplemented', ...
                %'Implement the rest of this method for Q1b.');

            % Get the vehicle vertex
            X_pre = this.edgeVertices{1}.estimate();
            % Get the landmark vertex
            X_post = this.edgeVertices{2}.estimate();
            % Define distance between vertices
            dx =  X_post - X_pre(1:2);
            
            % Calculate the error
            this.errorZ(1) = norm(dx) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - X_pre(3) - this.z(2));

        end
        
        function linearizeOplus(this)
            % Q2b:
            % Complete implementation
            %warning('landmarkrangebearingedge:linearizeoplus:unimplemented', ...
                %'Implement the rest of this method for Q1b.');
            x = this.edgeVertices{1}.estimate();
            X_I = this.edgeVertices{2}.estimate();
            dx = X_I - x(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            
            this.J{2} = - this.J{1}(1:2, 1:2);
        end        
    end
end