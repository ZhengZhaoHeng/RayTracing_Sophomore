#include "Raytracer.h"
#include <ctime>

int main(int argc, char* argv[])
{
	Camera* cmr = new Camera();
	time_t start = clock();
	IplImage* img = cvCreateImage(cvSize(800, 600), IPL_DEPTH_8U, 3);
	cmr->setPhoto(img, img->width, img->height);
	cmr->initRender(Vector3D(1, 0, -5), Vector3D(2, 3, 0));
	cmr->render();
	time_t end = clock();
	std::cout << double(end - start) / CLOCKS_PER_SEC << std::endl;
	cvNamedWindow("myWindow");
	cvShowImage("myWindow", img);
	cvSaveImage("result.jpg", img);

	cvWaitKey(0);
	cvDestroyWindow("myWindow");
	return 0;
}