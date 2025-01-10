classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	    % Q1d:
        % Implement the code
%         warning('gpsmeasurementedge:computeerror:unimplemented', ...
%                 'Implement the rest of this method for Q1d.');
        
        % Get the state (position and heading)
        x = this.edgeVertices{1}.estimate();
        % Define transformation matrix
        M = [cos(x(3)) -sin(x(3));
             sin(x(3)) cos(x(3))];
        % Calculate the errors
        this.errorZ = x(1:2) + M*this.xyOffset - this.z;
        end
        
        function linearizeOplus(this)

	    % Q1d:
        % Implement the code
%         warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
%                 'Implement the rest of this method for Q1d.');
        % Define Jacobians for derivatives
        this.J{1} = [1 0 0;
                     0 1 0];
        end
    end
end
