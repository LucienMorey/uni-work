% variable init
centre_x = 313;
centre_y = 313;
rotation_speed = 0.01;
x = zeros(0);
y = zeros(0);

%create tiled layout
t = tiledlayout(2,1);

%display track
nexttile(1,[1,1]);
imshow('Lab1_CircularRaceTrack.jpg'); 
axis on

%create red car
T_car_red = se2(300,550,0);
hold on
h_T_car_red = trplot2(T_car_red,'frame', 'car_red', 'color', 'r', 'length', 50);

red_radius = sqrt(((T_car_red(1,3)-centre_x)^2 + (T_car_red(2,3)-centre_y)^2));

%create the blue car
T_car_blue = se2(300, 125, 0);
h_T_car_blue = trplot2(T_car_blue, 'frame', 'car_blue', 'color', 'b', 'length', 50);
blue_radius = sqrt(((T_car_blue(1,3)-centre_x)^2 + (T_car_blue(2,3)-centre_y)^2));

%create static rotations to move cars

% Global frame: only have one of these uncommented
%T_rotation_red = se2(centre_x,centre_y)* se2(0,0,-rotation_speed)*se2(-centre_x,-centre_y,0);
%T_rotation_blue = se2(centre_x,centre_y)* se2(0,0,rotation_speed)*se2(-centre_x,-centre_y,0);

%angle increment in degrees
angle_of_increment = 1;
%Local Frame
T_rotation_red = se2(sqrt(2*red_radius^2 - 2*(red_radius^2)*cos(angle_of_increment * (pi/180))),0,0) * se2(0,0,-angle_of_increment*(pi/180));
T_rotation_blue = se2(sqrt(2*blue_radius^2 - 2*(blue_radius^2)*cos(angle_of_increment * (pi/180))),0,0) * se2(0,0,angle_of_increment*(pi/180));

% create transform text for overlay on image
message = sprintf('%0.2e  %0.2e  %0.2e\n%0.2e  %0.2e  %0.2e\n%0.2e  %0.2e  %0.2e',T_car_red(1,1),T_car_red(1,2),T_car_red(1,3),T_car_red(2,1),T_car_red(2,2),T_car_red(2,3),T_car_red(3,1),T_car_red(3,2),T_car_red(3,3));
tf_text= text(10, 50, message, 'FontSize', 10, 'Color', [.6 .2 .6]);

i = 0;
while i < 360
    % update plot 1
    nexttile(1,[1,1]);
    
    %transform car positions
    T_car_red = T_car_red*T_rotation_red;
    T_car_blue = T_car_blue*T_rotation_blue;
    
    %update graph handles for cars
    delete(h_T_car_red);
    delete(h_T_car_blue);
    h_T_car_red = trplot2(T_car_red, 'frame', 'car_red', 'color', 'r', 'length', 50);
    h_T_car_blue = trplot2(T_car_blue, 'frame', 'car_blue', 'color', 'b', 'length', 50);

    %update transform message on plot
    message = sprintf('%0.2e  %0.2e  %0.2e\n%0.2e  %0.2e  %0.2e\n%0.2e  %0.2e  %0.2e',T_car_red(1,1),T_car_red(1,2),T_car_red(1,3),T_car_red(2,1),T_car_red(2,2),T_car_red(2,3),T_car_red(3,1),T_car_red(3,2),T_car_red(3,3));
    set(tf_text,'string', message);
    
    % update plot 2
    nexttile(2,[1,1]);
    
    %calculate transform from red car to blue car
    T_red_to_blue = T_car_red\T_car_blue;
    
    % calculate distance between cars based on translation component of
    % transform
    distance = sqrt(T_red_to_blue(1,3)^2 + T_red_to_blue(2,3)^2);
    
    % add distance and time to end of arrays
    x = [x, i];
    y = [y, distance];
    
    %plot distance vs time
    plot(x,y)
    
    % flush graph
    i= i+1;
    drawnow();
end
