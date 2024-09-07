classdef UR3 < handle & RobotLucien
    properties(Constant)
        Q_HOME = [0,-pi/2,0,-pi/2,0,0];
    end
    
    methods%% Class for UR3 robot simulation
        function self = UR3(base_pose, modelPrefix, useGripper)
            self.useGripper = useGripper;
            
            self.GetUR3Robot(base_pose);

            self.PlotAndColourRobot(modelPrefix);
            self.DetermineXYReach();
            self.DetermineXZReach();
            self.DetermineYZReach();
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self, base_pose)

            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = '3';


            % Create the UR3 model
            L(1) = Link([0      0.172   0       -pi/2   0]);
            L(2) = Link([0      0.1235  0.249   0     0]);
            L(3) = Link([0      -0.1    0.2185  0      0]);
            L(4) = Link([0      0.091   0       -pi/2   0]);
            L(5) = Link([0      0.0915  0       pi/2	0]);
            L(6) = Link([0      0.075   0       0       0]);
            
            % Incorporate joint limits
            L(1).qlim = [-360 360]*pi/180;
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-360 360]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;

            self.model = SerialLink(L,'name',name);
            
            % Rotate robot to the correct orientation
            self.model.base = base_pose;
        end

        %generate waypoint pose for UR3. itis beneficial to face the required pose in an elbow up position
        function q = GenerateWaypointQ(self, desired_pose)
            q = self.facePose(desired_pose);
        end

        % calculate joint angle to face robot
        function q = facePose(self, pose)
            dt = self.model.base \ pose;
            q1 = atan2( dt(2, 4), dt(1, 4));
            q = [q1, -pi/4, 0, -pi/3, -pi/2, 0];
        end

        % determine reach of robot in local xy plane
        function DetermineXYReach(self)
            q1 = 0;
            q2 = 0;
            q3 = 0;
            q4 = deg2rad(-90);
            q5 = 0;
            q6 = 0;
            counter = 1;
            for q1=0:self.ANGULAR_RESOLUTION:deg2rad(360)
                q = [q1,q2,q3,q4,q5,q6];
                tr = self.model.fkine(q);
                self.XYReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            self.XYReachHandle = plot3(self.XYReachPoints(:,1), self.XYReachPoints(:,2), self.XYReachPoints(:,3), 'Visible', false);
        end
        
        % determine reach of robot in local xz plane
        function DetermineXZReach(self)
            %transform to first shoulder frame for calcs
            t1 = self.model.base * ...
                trotz(self.model.theta(1)) * ...
                transl(self.model.a(1), 0, self.model.d(1)) * ...
                trotx(self.model.alpha(1));
            t2 = self.model.base;

            td = t2\t1;
            startingAngle = asin(td(3,4)/self.maxReach);
            
            q1 = 0;
            q2 = 0;
            q3 = 0;
            q4 = deg2rad(-90);
            q5 = 0;
            q6 = 0;
            counter = 1;
            for q2= 0:self.ANGULAR_RESOLUTION:deg2rad(360);
                q = [q1,q2,q3,q4,q5,q6];
                tr = self.model.fkine(q);
                self.XZReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            self.XZReachHandle = plot3(self.XZReachPoints(:,1), self.XZReachPoints(:,2), self.XZReachPoints(:,3), 'Visible', false);
        end 

        % determine reach of robot in local yz plane
        function DetermineYZReach(self)
            %transform to first shoulder frame for calcs
            t1 = self.model.base * ...
                trotz(self.model.theta(1)) * ...
                transl(self.model.a(1), 0, self.model.d(1)) * ...
                trotx(self.model.alpha(1));
            t2 = self.model.base;

            td = t2\t1;
            startingAngle = asin(td(3,4)/self.maxReach);
            
            q1 = deg2rad(pi/2);
            q2 = 0;
            q3 = 0;
            q4 = deg2rad(-90);
            q5 = 0;
            q6 = 0;
            counter = 1;
            for q2= 0:self.ANGULAR_RESOLUTION:deg2rad(360)
                q = [q1,q2,q3,q4,q5,q6];
                tr = self.model.fkine(q);
                self.YZReachPoints(counter,:) = tr(1:3,4)';
                counter = counter+1;
            end
            self.XZReachHandle = plot3(self.YZReachPoints(:,1), self.YZReachPoints(:,2), self.YZReachPoints(:,3), 'Visible', false);
        end

        %check if brick is in range
        function inRange = BrickInRange(self, brickPose)
            %transform to first shoulder frame for calcs
            shoulderOrigin = self.model.base * ...
            trotz(self.model.theta(1)) * ...
            transl(self.model.a(1), 0, self.model.d(1)) * ...
            trotx(self.model.alpha(1));

            displacementToBrick = brickPose(1:3,4) - shoulderOrigin(1:3,4);
            inRange = sqrt(displacementToBrick(1,1)^2 + displacementToBrick(2,1)^2 + displacementToBrick(3,1)^2) < self.maxReach;


        end
    end
end