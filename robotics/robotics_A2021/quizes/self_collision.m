number_of_joints = 3;

robot = mdl_hyper2d(number_of_joints);

robot.teach();

pause(0.01);

for i = 0:number_of_joints-2
   l1_p1 = robot.base();
   l1_p2 = robot.base();
   q = robot.getpos();
   
   if (i > 0)
       for j = 1 : i
       T_theta = trotz(q(j));
       T_d = transl(0,0,robot.d(j));
       T_a = transl(robot.a(j),0,0);
       T_alpha = trotx(robot.alpha(j));
       l1_p1 = l1_p1 * T_theta * T_d * T_a * T_alpha;
       end
   end

       for j = 1 : i+1
       T_theta = trotz(q(j));
       T_d = transl(0,0,robot.d(j));
       T_a = transl(robot.a(j),0,0);
       T_alpha = trotx(robot.alpha(j));
       l1_p2 = l1_p2 * T_theta * T_d * T_a * T_alpha;
       end
   
   
   for j = i+1:number_of_joints-1
       l2_p1 = robot.base();
       l2_p2 = robot.base();

       for k = 1 : j
       T_theta = trotz(q(k));
       T_d = transl(0,0,robot.d(k));
       T_a = transl(robot.a(k),0,0);
       T_alpha = trotx(robot.alpha(k));
       l2_p1 = l2_p1 * T_theta * T_d * T_a * T_alpha;
       end

       for k = 1 : j+1
       T_theta = trotz(q(k));
       T_d = transl(0,0,robot.d(k));
       T_a = transl(robot.a(k),0,0);
       T_alpha = trotx(robot.alpha(k));
       l2_p2 = l2_p2 * T_theta * T_d * T_a * T_alpha;
       end
       l1_p1;
       l1_p2;
       l2_p1;
       l2_p2;
       denominator = ((l2_p2(2,4) - l2_p1(2,4)) * (l1_p2(1,4) - l1_p1(1,4)) - (l2_p2(1,4) - l2_p1(1,4)) * (l1_p2(2,4) - l1_p1(2,4)));
       alpha_numerator = ((l2_p2(1,4) - l2_p1(1,4)) * (l1_p1(2,4) - l2_p1(2,4)) - (l2_p2(2,4) - l2_p1(1,4)) * (l1_p1(1,4) - l2_p1(1,4)));
       beta_numerator = ((l1_p2(1,4) - l1_p1(1,4)) * (l1_p1(2,4) - l2_p1(2,4)) - (l1_p2(2,4) - l1_p1(2,4)) * (l1_p1(1,4) - l2_p1(1,4)));
       
       s_l1 = alpha_numerator/denominator
       s_l2 = beta_numerator/denominator
       
       if(s_l1 > 0.0)&&(s_l1 < 1.0)&&(s_l2 > 0.0)&&(s_l2 < 0.0)
           p1 + s_l1*(p2-p1)
           
           t = zeros(4,4);
           x = zeros(4,4);
           anss = t * inv(x)
       end
   end
   
end