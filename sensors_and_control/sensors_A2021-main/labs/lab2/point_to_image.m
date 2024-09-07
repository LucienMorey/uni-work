function [image_point] = point_to_image(point, principal_point, focal_length)

image_u = focal_length(1) * point(1)/point(3) + principal_point(1);
image_v = focal_length(1) * point(2)/point(3) + principal_point(2);

image_point = [image_u, image_v];

end
