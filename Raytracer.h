#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#include "common.h"
#include "scene.h"

class Ray
{
public:
	Ray(){}
	Ray(Vector3D& origin, Vector3D& direction, int id):_origin(origin), _direction(direction), _id(id){}
	~Ray(){}
	void set_origin(Vector3D& origin){ _origin = origin; }
	void set_direction(Vector3D& direction){ _direction = direction; }
	Vector3D& get_origin(){ return _origin; }
	Vector3D& get_direction(){ return _direction; }
	void set_id(int id){ _id = id; }
	int get_id(){ return _id; }

private:
	Vector3D _origin;
	Vector3D _direction;
	int _id;
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
	Primitive* renderRay(Vector3D& pos, Color& color);
	int findNest(Ray& ray, double& dist, Primitive*& prim);
	void initRender(Vector3D& pos, Vector3D& target);
	void renderPart(int x1, int x2, int y1, int y2);
	void render();
	double calculateShade(Primitive* prim, Vector3D& pos, Vector3D& l);

protected:
	Vector3D _origin, _p1, _p2, _p3, _p4, _dx, _dy;
	Scene* _scene;
	IplImage* _img;
	int _width, _height;
	Vector3D _sr, _cw;
	int _n_id;
};

#endif

