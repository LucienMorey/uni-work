image = imread('IMG_5730.jpg');

image_gs = convertRGBtoGrayscale_student(image);

imshow(image_gs);

image_bw = convertGStoBW_student(image_gs, 128);
imshow(image_bw)