#include "Raytracer.h"

int main(int argc, char* argv[])
{
	Camera* cmr = new Camera();
	IplImage* img = cvCreateImage(cvSize(800, 600), IPL_DEPTH_8U, 3);
	cmr->setPhoto(img, img->width, img->height);
	cmr->initRender();
	cmr->render();

	cvNamedWindow("myWindow");
	cvShowImage("myWindow", img);

	cvWaitKey(0);
	cvDestroyWindow("myWindow");
	return 0;
}