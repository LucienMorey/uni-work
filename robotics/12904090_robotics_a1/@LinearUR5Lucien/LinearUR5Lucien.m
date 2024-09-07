classdef LinearUR5Lucien < handle & RobotLucien
    
    properties
        workspace = [-2 2 -2 2 -0.3 2];
        pointCloud;   
    end

    properties(Constant)
        Q_HOME = zeros(1,7);
        
    end

    methods%% Class for UR5 robot simulation
        function self = LinearUR5Lucien(basePose, modelPrefix, useGripper)
            self.useGripper = useGripper;
            
            self.GetUR5Robot(basePose); 
            self.PlotAndColourRobot(modelPrefix);
            self.DetermineWorkspaceVolume();
            self.DetermineXYReach();
            self.DetermineXZReach();

        end

        %% GetUR5Robot
        % Given a name (optional), create and return a UR5 robot model
        function GetUR5Robot(self, basePose)
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = '5';

            % Create the UR5 model mounted on a linear rail
            L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            L(2) = Link([0      0.1599  0       -pi/2   0]);
            L(3) = Link([0      0.1357  0.425   -pi     0]);
            L(4) = Link([0      0.1197  0.39243 pi      0]);
            L(5) = Link([0      0.093   0       -pi/2   0]);
            L(6) = Link([0      0.093   0       -pi/2	0]);
            L(7) = Link([0      0.082       0       0       0]);
            
            % Incorporate joint limits
            L(1).qlim = [-0.8 0];
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-90 90]*pi/180;
            L(4).qlim = [-170 170]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(7).qlim = [-360 360]*pi/180;

            L(3).offset = -pi/2;
            L(5).offset = -pi/2;
            
            self.model = SerialLink(L,'name',name);
            
            % Rotate robot to the correct orientation
            self.model.base = basePose * trotx(pi/2);
        end

        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self,modelPrexix)%robot,workspace)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([modelPrefix,num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([modelPrexix,num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            t1 = self.model.base * ...
            trotz(self.model.theta(1)) * ...
            transl(self.model.a(1), 0, self.model.d(1)) * ...
            trotx(self.model.alpha(1)) * ...
            trotz(self.model.theta(2)) * ...
            transl(self.model.a(2), 0, self.model.d(2)) * ...
            trotx(self.model.alpha(2));

            t2 = self.model.fkine([0, 0, 0, 0, 0, 0, 0]);


            td = t1 \ t2;
            self.maxReach = sqrt(td(1, 4)^2 + td(2, 4)^2 + td(3, 4)^2);
            
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                , plyData{linkIndex+1}.vertex.green ...
                                                                , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

        function q = GenerateWaypointQ(self, desired_pose)
            waypointT = desired_pose * transl(0,0,-0.2);
            q = self.model.ikcon(waypointT, self.model.getpos);
        end

        % repeat getting xy workspace at different elevations and combine in surface mesh for volume
        function DetermineWorkspaceVolume(self)

            stepRads = deg2rad(30);
            qlim = self.model.qlim;
            % Don't need to worry about joint 6
            self.workspacePointcloud = [];
            counter = 1;
            q0 = 0;
            q1 = 0;
            q2 = 0;
            q3 = degtorad(90);
            q4 = 0; 
            q5 = 0; 
            q6 = 0; 
            q7 = 0;
            for q3 = deg2rad(180):-self.ANGULAR_RESOLUTION:deg2rad(0)
                for q1= 0:-self.LINEAR_RESOLUTION:-0.8
                    q = [q1,q2,q3,q4,q5,q6,q7];                        
                    tr = self.model.fkine(q);                                                
                    self.workspacePointcloud(counter,:) = tr(1:3,4)';
                    counter = counter+1;
                end
                q1 = -0.8;
                for q2 = deg2rad(0):-self.ANGULAR_RESOLUTION:deg2rad(-180)
                    q = [q1,q2,q3,q4,q5,q6,q7];                        
                    tr = self.model.fkine(q);                                                
                    self.workspacePointcloud(counter,:) = tr(1:3,4)';
                    counter = counter+1;
                end
                q2 = deg2rad(-180);
                for q01 = -0.8:self.LINEAR_RESOLUTION:0
                    q = [q1,q2,q3,q4,q5,q6,q7];                        
                    tr = self.model.fkine(q);                                                
                    self.workspacePointcloud(counter,:) = tr(1:3,4)';
                    counter = counter+1;
                end
                q1=0;
                for q2 = deg2rad(-180):-self.ANGULAR_RESOLUTION:deg2rad(-360)
                    q = [q1,q2,q3,q4,q5,q6,q7];                        
                    tr = self.model.fkine(q);                                                
                    self.workspacePointcloud(counter,:) = tr(1:3,4)';
                    counter = counter+1;
                end
            end

            self.tri = delaunay(self.workspacePointcloud(:,1), self.workspacePointcloud(:,2), self.workspacePointcloud(:,3));
            self.volumeSurfaceHandle = trisurf(self.tri, self.workspacePointcloud(:,1), self.workspacePointcloud(:,2), self.workspacePointcloud(:,3), 'Visible', false);
            [self.workspaceVolume,] = self.stlVolume(self.workspacePointcloud', self.tri');
        end

        % determine xy reach
        function DetermineXYReach(self)
            q0 = 0;
            q1 = 0;
            q2 = 0;
            q3 = degtorad(90);
            q4 = 0; 
            q5 = 0; 
            q6 = 0; 
            q7 = 0;
            self.XYReachPoints = [];
            counter = 1;
            for q1= 0:-self.LINEAR_RESOLUTION:-0.8
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            q1 = -0.8;
            for q2 = deg2rad(0):-self.ANGULAR_RESOLUTION:deg2rad(-180)
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            q2 = deg2rad(-180);
            for q01 = -0.8:self.LINEAR_RESOLUTION:0
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            q1=0;
            for q2 = deg2rad(-180):-self.ANGULAR_RESOLUTION:deg2rad(-360)
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            self.XYReachHandle = plot3(self.XYReachPoints(:,1), self.XYReachPoints(:,2), self.XYReachPoints(:,3), 'Visible', false);
        end

        % TODO determine YZ Reach
        function DetermineYZReach(self)
            q0 = 0;
            q1 = 0;
            q2 = 0;
            q3 = degtorad(90);
            q4 = 0; 
            q5 = 0; 
            q6 = 0; 
            q7 = 0;
            self.XYReachPoints = [];
            counter = 1;
            for q1= 0:-self.LINEAR_RESOLUTION:-0.8
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            q1 = -0.8;
            for q2 = deg2rad(0):-self.ANGULAR_RESOLUTION:deg2rad(-180)
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            q2 = deg2rad(-180);
            for q01 = -0.8:self.LINEAR_RESOLUTION:0
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            q1=0;
            for q2 = deg2rad(-180):-self.ANGULAR_RESOLUTION:deg2rad(-360)
                q = [q1,q2,q3,q4,q5,q6,q7];                        
                tr = self.model.fkine(q);                                                
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            self.XYReachHandle = plot3(self.XYReachPoints(:,1), self.XYReachPoints(:,2), self.XYReachPoints(:,3), 'Visible', false);
        end

        % TODO determine XZ Reach
        function DetermineXZReach(self)
            
            qlim = self.model.qlim;
            q1 = 0;
            q2 = deg2rad(90);
            q3 = 0;
            q4 = 0; 
            q5 = 0; 
            q6 = 0; 
            q7 = 0;
            self.XZReachPoints = [0,0,0];
            counter = 1;
            for q1= qlim(1,1):self.LINEAR_RESOLUTION:qlim(1,2)
                 
                for q3 =qlim(3,1):self.ANGULAR_RESOLUTION:qlim(3,1)
                    for q4 = qlim(4,1):self.ANGULAR_RESOLUTION:qlim(4,2)
                        q2 = deg2rad(-90);   
                        q = [q1,q2,q3,q4,q5,q6,q7];                        
                        tr = self.model.fkine(q);                                                
                        self.XZReachPoints(counter,:) = tr(1:3,4)';
                        counter = counter + 1;
                        q2 = deg2rad(90);
                        q = [q1,q2,q3,q4,q5,q6,q7];                        
                        tr = self.model.fkine(q);                                                
                        self.XZReachPoints(counter,:) = tr(1:3,4)';
                        counter = counter + 1;
                    end
                end
            end

            k = convhull(self.XZReachPoints);
            self.XZReachHandle = plot3(self.XZReachPoints(:,1), self.XZReachPoints(:,2), self.XZReachPoints(:,3), 'Visible', false);
        end

        % Check brick in range. do this buy projecting brick point to line segment of linear rail and checking distance from projection ot real point
        function inRange = BrickInRange(self, brickPose)
            segmentStart = self.model.base * ...
            trotz(self.model.theta(1)) * ...
            transl(self.model.a(1), 0, self.model.qlim(1,1)) * ...
            trotx(self.model.alpha(1));
            
            segmentEnd = self.model.base * ...
            trotz(self.model.theta(1)) * ...
            transl(self.model.a(1), 0, self.model.qlim(1,2)) * ...
            trotx(self.model.alpha(1));

            p = brickPose(1:3,4);

            segment=segmentEnd(1:3,4)-segmentStart(1:3,4);
            displacement = p -segmentStart(1:3,4);
            scale = dot(displacement,segment)/dot(segment,segment);
            projected_point = segmentStart(1:3,4) + scale * segment;

            if scale > 1.0
                projected_point = segmentEnd(1:3,4);
            elseif scale < 0.0
                projected_point = segmentStart(1:3,4);
            end
            projectionToBrick = p - projected_point;
            inRange = sqrt(projectionToBrick(1,1)^2 + projectionToBrick(2,1)^2 + projectionToBrick(3,1)^2) < self.maxReach;



        end

    end
end