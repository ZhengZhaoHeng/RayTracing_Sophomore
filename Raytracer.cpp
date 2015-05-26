

#include "Raytracer.h"
#include <cmath>

Camera::Camera()
{
	_scene = new Scene();
	_scene->initScene();
}

Camera::~Camera()
{
	delete _scene;
}

void Camera::setPhoto(IplImage* img, int width, int height)
{
	_img = img;
	_width = width;
	_height = height;
}

void Camera::initRender()
{
	_wx1 = -4;
	_wx2 = 4;
	_wy1 = 3;
	_wy2 = -3;
	_cx = _wx1;
	_cy = _wy1;
	_dx = (_wx2 - _wx1) / _width;
	_dy = (_wy2 - _wy1) / _height;
}

bool Camera::render()
{
	Vector3D src(0, 0, -5);
	for (int j = 0; j < _height; j++)
	{
		_cx = _wx1;
		for (int i = 0; i < _width; i++)
		{
			Color temp(0, 0, 0);
			Vector3D direction = Vector3D(_cx, _cy, 0) - src;
			direction.normalize();
			Ray ray(src, direction);
			double dist = 0;
			Primitive* prim = rayTracing(ray, temp, 1, 1, dist);
			int red = int(temp._x * 256);
			//if (temp._x > 0) std::cout << temp._x * 255 << ' ' << red <<std::endl;
			int green = int(temp._y * 256);
			int blue = int(temp._z * 256);
			red = std::min(red, 255);
			green = std::min(green, 255);
			blue = std::min(blue, 255);
			CvScalar color = CV_RGB(red, green, blue);
			for (int k = 0; k < 3; k++)
			{
				_img->imageData[j * _img->widthStep + i * 3 + k] = color.val[k];
			}
			_cx += _dx;
		}
		_cy += _dy;
	}
	return true;
}

Primitive* Camera::rayTracing(Ray& ray, Color& color, int depth, double r_index, double dist)
{
	if (depth > TRACEDEPTH)  return NULL;
	dist = 1e6;
	Primitive* prim = NULL;
	int result;
	//find the nearest primitive.
	for (int i = 0; i < _scene->get_num_prim(); i++)
	{
		//std::cout << i << std::endl;
		Primitive* temp = _scene->get_primitive(i);
	//	std::cout << temp->get_name() << std::endl;
		int reply = temp->intersect(ray, dist);
		if (reply != 0)
		{
			result = reply;
			prim = temp;
		}
	}

	if (prim == NULL) return NULL;
	if (prim->isLight())
	{
		color = Color(1, 1, 1);
	}
	else
	{
		Vector3D pnt = ray.get_origin() + ray.get_direction() * dist;	
		for (int i = 0; i < _scene->get_num_prim(); i++)
		{
			Primitive* temp = _scene->get_primitive(i);
			if (temp->isLight())
			{
				Primitive* light = temp;
				Vector3D l = ((Sphere*)light)->get_center() - pnt;
				l.normalize();
				Vector3D n = prim->get_normal(pnt);
				if (prim->get_material().get_diff() > 0)
				{
					double d = dot(n, l);
					if (d > 0)
					{
						double diffuse = d * prim->get_material().get_diff();
						//std::cout << diffuse <<std::endl;
						color = color + diffuse * prim->get_material().get_color() * light->get_material().get_color();
						/*std::cout << diffuse<<  ' ' << color._x << ' ' << 
							prim->get_material().get_color()._x << ' ' <<
							light->get_material().get_color()._x << std::endl;*/
					}
				}
			}
		}
	}
	return prim;
}

