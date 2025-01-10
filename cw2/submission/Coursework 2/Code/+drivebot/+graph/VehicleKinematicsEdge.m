% This class uses a slightly simpler model for the vehicle kinematics used
% in the lectures. This is the more standard built in type for estimate.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
%
% M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
%
% The process model has the form:
%
% x = x + M * [vx;vy;theta]
%
% where vx, vy and vtheta are the velocities.
%
% The error model 
% eTheta = 

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function this = VehicleKinematicsEdge(dT)
            assert(dT >= 0);
            this = this@g2o.core.BaseBinaryEdge(3);            
            this.dT = dT;
        end
       
        function initialize(this)
            
                        
            priorX = this.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            M = this.dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Compute the posterior assming no noise
            this.edgeVertices{2}.x = this.edgeVertices{1}.x + M * this.z;

            % Wrap the heading to -pi to pi
            this.edgeVertices{2}.x(3) = g2o.stuff.normalize_theta(this.edgeVertices{2}.x(3));

        end
        
        function computeError(this)
    
            % Q1b:
            % Complete implementation

            % Get the prior vertex
            X_pre = this.edgeVertices{1}.x;

            % Define inverse transformation matrix
            Mi = [cos(X_pre(3)) sin(X_pre(3)) 0;
                 -sin(X_pre(3)) cos(X_pre(3)) 0;
                  0             0             1] / this.dT;

            % Compare between prior and posterior vertices
            dx = this.edgeVertices{2}.x - X_pre;
            dx(3) = g2o.stuff.normalize_theta(dx(3));

            % Calculate the errors
            this.errorZ = Mi * (dx) - this.z;
            
            % warning('vehiclekinematicsedge:computeerror:unimplemented', ...
            %         'Implement the rest of this method for Q1b.');
        end
        
        % Compute the Jacobians
        function linearizeOplus(this)

            % Q1b:
            % Complete implementation
            X_pre = this.edgeVertices{1}.x;

            dx = this.edgeVertices{2}.x - X_pre;
            Mi = [cos(X_pre(3)) sin(X_pre(3)) 0;
                 -sin(X_pre(3)) cos(X_pre(3)) 0;
                  0 0 1] / this.dT;
            % Define Jacobians
            % Get partial derivatives
            this.J{2} = Mi;
            this.J{1}(1, 1) = - cos(X_pre(3)); 
            this.J{1}(1, 2) = - sin(X_pre(3));
            this.J{1}(1, 3) = (-dx(1) * sin(X_pre(3)) + dx(2) * cos(X_pre(3)));
            this.J{1}(2, 1) = sin(X_pre(3));
            this.J{1}(2, 2) = - cos(X_pre(3));
            this.J{1}(2, 3) = (-dx(1) * cos(X_pre(3)) - dx(2) * sin(X_pre(3)));
            this.J{1}(3, 3) = -1;
            this.J{1} = this.J{1}/ this.dT;
            % warning('vehiclekinematicsedge:linearizeoplus:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');

        end
    end    
end