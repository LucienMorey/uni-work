function[collided] = CheckCollision(robot, sphere_centre, radius)
    tr = robot.fkine(robot.getpos);
    distance_to_centre = sqrt(sum((tr(1:3,4) - sphere_centre)^2));
    
    if distance_to_centre < radius
        disp('cooked it');
        collided = 1;
    else 
        disp('all g');
        collided = 0;
    end
end