classdef Brick < handle
    %   Class creates a handle to display brick and handles position
    
    properties
        handle;
        vertexCount;
        pose;
        possession = 0;
        placementPose = eye(3);
        
        
    end
    
    properties(Constant)
        PLACED = 1;
        MARKED = 2;
        UR5_POSSESSION = 5;
        UR3_POSSESSION = 3;
        FREE = 0;
    end
    
    methods
        function self = Brick(position,TABLE_OFFSET)
            % read ply file for vertex data
            [f,v,data] = plyread('brick.ply','tri');
            self.vertexCount = size(v,1);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            %set current pose
            self.pose = transl(position(1),position(2),TABLE_OFFSET);
            %plot brick
            self.handle = trisurf(f ...
                , v(:,1)+position(1), v(:,2)+position(2), v(:,3)+TABLE_OFFSET ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
        end
        
        %update current brick possessor
        function SetPossession(self,possessor)
            self.possession = possessor;
        end
        
        %set new brick pose
        function updatePose(self, newPose)
            %invert transform to return to zero and then move in global frame
            updatedPoints = [self.pose \ [self.handle.Vertices,ones(self.vertexCount,1)]']';
            self.handle.Vertices = updatedPoints(:,1:3);
            self.pose = newPose;
            updatedPoints = [self.pose * [self.handle.Vertices,ones(self.vertexCount,1)]']';
            self.handle.Vertices = updatedPoints(:,1:3);
        end
    end
end

