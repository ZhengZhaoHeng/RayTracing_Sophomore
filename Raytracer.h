#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#include "common.h"
#include "scene.h"

class Ray
{
public:
	Ray(){}
	Ray(Vector3D& origin, Vector3D& direction):_origin(origin), _direction(direction){}
	~Ray(){}
	void set_origin(Vector3D& origin){ _origin = origin; }
	void set_direction(Vector3D& direction){ _direction = direction; }
	Vector3D& get_origin(){ return _origin; }
	Vector3D& get_direction(){ return _direction; }

private:
	Vector3D _origin;
	Vector3D _direction;
};

class Scene;
class Primitive;
class Camera
{
public:
	Camera();
	~Camera();
	void setPhoto(IplImage*, int, int);
	Scene* get_scene(){ return _scene;}
	Primitive* rayTracing(Ray& ray, Color& color, int depth, double r_index, double dist);
	void initRender();
	bool render();

protected:
	double _wx1, _wx2, _wy1, _wy2, _dx, _dy, _cx, _cy;
	Scene* _scene;
	IplImage* _img;
	int _width, _height;
};

#endif

