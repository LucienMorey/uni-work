#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    if ( argc != 3 ) {
        printf("usage: ./access_pixels <Image_Path> <Saved_Image>\n");
        return -1;
    }

    Mat image;
    image = imread( argv[1], IMREAD_GRAYSCALE);

    if ( !image.data ) {
        printf("No image data \n");
        return -1;
    }
    
    int white_count = 0;
    unsigned char thresh = 200;
    for (int i = 0; i<image.rows; i++) {
        for (int j = 0; j <image.cols; j++) {
            unsigned char &pixel = image.at<unsigned char>(i, j);
            if (pixel > thresh) {
                white_count++;
                pixel = 0;
            }
        }
    }
    std::cout << "There are " << white_count << " white-ish pixels" << std::endl;
    namedWindow("Image", WINDOW_AUTOSIZE );
    imshow("Image", image);
    imwrite(argv[2], image);

    waitKey(0);

    return 0;
}
