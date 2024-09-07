function animated_inv_pend(x,th,m,M,L)
%Based on the animation created by Steve Brunton

% kinematics
% x % cart position
% th % pendulum angle 

% dimensions
% L = 2;  % pendulum length
W = 1*sqrt(M/5);  % cart width
H = .5*sqrt(M/5); % cart height
wr = .2; % wheel radius
mr = .3*sqrt(m); % mass radius

% positions
% y = wr/2; % cart vertical position
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px = x + L*sin(th);
py = y + L*cos(th);

%% Colors
grey = [0.3 0.3 0.3];
black = [0 0 0];
white = [1 1 1];
red = [1 0.1 0.1];
blue = [.3 0.3 1];

plot([-10 10],[0 0],'k','LineWidth',2)  %ground line
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',red,'EdgeColor',black)
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',grey,'EdgeColor',black)
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',grey,'EdgeColor',black)

plot([x px],[y py],'k','LineWidth',2)

rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',blue,'EdgeColor',black)

% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-2 10]);
ylim([-2.5 2.5]);
%set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Position',[10 900 800 400])
%set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   

% box off
drawnow
hold off