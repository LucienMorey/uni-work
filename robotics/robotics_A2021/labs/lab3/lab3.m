% 0 = 3 Link robot
% 1 = AAN-Robot - UR10
% 2 = SPIR
% 3 = Sawyer Robot

robot = 4;

switch robot
    case 0
        L1 = Link('d', 1, 'a', 0, 'alpha', pi/2);
        L2 = Link('d', 0, 'a', 1, 'alpha', 0);
        L3 = Link('d', 0, 'a', 1, 'alpha', -pi/2);
        robot = SerialLink([L1,L2,L3], 'name', '3 Link Robot');
        q = zeros(1,3);
        workspace = [-4, 4, -4, 4, -4, 4];
        scale = 1;
        robot.plot(q, 'workspace', workspace, 'scale', scale);
        robot.teach();
        %J = robot.jacob0(q)
        %inv(J)
        %robot.vellipse(q)
        
    
    case 1
        L1 = Link('d', 0.1293, 'a', 0, 'alpha', -pi/2);
        L2 = Link('d', 0, 'a', 0.612, 'alpha', 0);
        L3 = Link('d', 0, 'a', 0.57203, 'alpha', 0);
        L4 = Link('d', 0.16394, 'a', 0, 'alpha', pi/2);
        L5 = Link('d', -0.1157, 'a', 0, 'alpha', -pi/2);
        L6 = Link('d', 0.09202, 'a', 0, 'alpha', pi/2);
        robot = SerialLink([L1, L2, L3, L4, L5, L6], 'name', 'AANRobot');
        q = zeros(1,6);
        workspace = [-2, 2, -2, 2, -2, 2];
        scale = 1;
        robot.plot(q, 'workspace', workspace, 'scale', scale);
        robot.teach();
        %J = robot.jacob0(q)
        %inv(J)
        %robot.vellipse(q)
        
    case 2
        L1 = Link('d', 0.09625, 'a', 0, 'alpha', pi/2);
        L2 = Link('d', 0, 'a', 0.2773, 'alpha', 0);
        L3 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
        L4 = Link('d', 0.23601, 'a', 0, 'alpha', pi/2);
        L5 = Link('d', -0, 'a', 0, 'alpha', -pi/2);
        L6 = Link('d', 0.13435, 'a', 0, 'alpha', 0);
        robot = SerialLink([L1, L2, L3, L4, L5, L6], 'name', 'SPIR');
        q = zeros(1,6);
        workspace = [-1, 1, -1, 1, -1, 1];
        scale = 1;
        robot.plot(q, 'workspace', workspace, 'scale', scale);
        robot.teach();
        %J = robot.jacob0(q)
        %inv(J)
        %robot.vellipse(q)
        
    case 3
        L1 = Link('d',0.09625,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
        L2 = Link('d',0,'a',0.27813,'alpha',0,'offset',1.2981,'qlim',[deg2rad(-74.3575),deg2rad(105.6425)]);
        L3 = Link('d',0,'a',0,'alpha',-pi/2,'offset',-2.8689,'qlim',[deg2rad(-90),deg2rad(90)]);
        L4 = Link('d',0.23601,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
        L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
        L6 = Link('d',0.13435,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]);
        
        robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','igus');
        
        q = zeros(1,6);
        workspace = [-1, 1, -1, 1, -1, 1];
        scale = 1;
        robot.plot(q, 'workspace', workspace, 'scale', scale);
        robot.teach();
        %J = robot.jacob0(q)
        %inv(J)
        %robot.vellipse(q)
        
    case 4
        % 2.2 Determine the D&H parameters based upon the link measurements on the PDF
        % & 2.3 Determine and include the joint limits in your model
        L1=Link('alpha',-pi/2,'a',0.180, 'd',0.475, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
        L2=Link('alpha',0,'a',0.385, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
        L3=Link('alpha',pi/2,'a',-0.100, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
        L4=Link('alpha',-pi/2,'a',0, 'd',0.329+0.116, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
        L5=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
        L6=Link('alpha',0,'a',0, 'd',0.09, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);

        densoRobot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Denso VM6083G');
        densoRobot.name = 'Denso VM6083G';
        % Use glyphs to draw robot, don't display the name
        densoRobot.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'};
        qlim = densoRobot.qlim;
        
        
        angle_steps = deg2rad(30);
        pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/angle_steps + 1));
        point_cloud = zeros(pointCloudeSize,3);
        counter = 1;
        
        for q1 = qlim(1,1):angle_steps:qlim(1,2)
            for q2 = qlim(1,1):angle_steps:qlim(1,2)
                for q3 = qlim(1,1):angle_steps:qlim(1,2)
                    for q4 = qlim(1,1):angle_steps:qlim(1,2)
                        for q5 = qlim(1,1):angle_steps:qlim(1,2)
                            tr = densoRobot.fkine([q1, q2, q3, q4, q5, 0]);
                            point_cloud(counter, :) = tr(1:3, 4)';
                            counter = counter+1;
                        end
                    end
                end
            end
        end
        
        plot3(point_cloud(:,1),point_cloud(:,2),point_cloud(:,3),'r.');
                            
    otherwise
        
end

