#ifndef __SCENE_H_
#define __SCENE_H_

#include "common.h"
#include "Raytracer.h"
#include <string>
#include <vector>

#define HIT 1
#define MISS 0
#define INPRIM -1;

#define MAXLIGHTS 10

class Ray;
//Material Definition
class Material
{
public:
	Material();
	~Material();
	void set_color(Color& color){ _color = color; }
	Color get_color(){ return _color; }
	void set_refl(double refl){ _refl = refl; }
	double get_refl(){ return _refl; }
	void set_diff(double diff){ _diff = diff; }
	double get_diff(){ return _diff; }
	void set_refr(double refr){ _refr = refr; }
	double get_refr(){ return _refr; }
	void set_refr_index(double refr_index){ _refr_index = refr_index; }
	double get_refr_index(){ return _refr_index; }
	void set_spec(double spec){ _spec = spec; }
	double get_spec(){ return _spec; }
	void set_diff_refl(double diff_refl){ _diff_refl = diff_refl; }
	double get_diff_refl(){ return _diff_refl; }
	void set_uv(double u, double v){ _uscale = u; _vscale = v; }
	void set_texture(const char* filename)
	{
		_textured = true;
		_texture = cvLoadImage(filename);
	}
	Vector3D get_texture(double u, double v)
	{
		int x = (int)((u * _texture->width) * _uscale) % _texture->width;
		int y = (int)((v * _texture->height) * _vscale) % _texture->height;
		return _color * Vector3D(double((unsigned char)(_texture->imageData[y * _texture->widthStep + x * 3 + 2])) / 255,
			double((unsigned char)(_texture->imageData[y * _texture->widthStep + x * 3 + 1])) / 255,
			double((unsigned char)(_texture->imageData[y * _texture->widthStep + x * 3])) / 255);
	}
	bool get_textured(){ return _textured; }

private:
	Color _color;
	double _refl;
	double _diff;
	double _refr;
	double _refr_index;
	double _spec;
	double _diff_refl;
	double _uscale;
	double _vscale;
	IplImage* _texture;
	bool _textured;
};

//Primitive Definition
class Primitive
{
public:
	enum
	{
		SPHERE = 1,
		PLANE,
		BOX,
		TRIANGLE
	};
	Primitive():_light(false), _ray_id(-1){}
	void set_material(Material material){ _material = material; }
	Material& get_material(){ return _material; }
	bool isLight(){ return _light; }
	void set_name(const char* name){ _name = name; }
	std::string get_name(){ return _name; }
	int get_last_ray_id(){ return _ray_id; }
	virtual void set_light(bool light){ _light = light; }
	virtual int get_type() = 0;
	virtual AABB get_box() = 0;
	virtual int intersect(Ray& ray, double& dist) = 0;
	virtual bool intersectBox(AABB& box) = 0;
	virtual Vector3D get_normal(Vector3D& pos) = 0;
	virtual Color get_color(Vector3D& pos){ return _material.get_color(); }

protected:
	Material _material;
	std::string _name;
	bool _light;
	int _ray_id;
};

class Sphere : public Primitive
{
public:
	Sphere(Vector3D& center, double radius):
		_center(center), _radius(radius),
		_sq_radius(sqr(radius)), _r_radius(1.0 / radius)
	{
		_vn = Vector3D(0, 1, 0);
		_ve = Vector3D(1, 0, 0);
		_vc = _vn.cross(_ve);
	}
	~Sphere(){}
	int get_type(){ return SPHERE; }
	Vector3D& get_center(){ return _center; }
	Vector3D get_normal(Vector3D& pos){ return (pos - _center) * _r_radius; }
	double get_sq_radius(){ return _sq_radius; }
	double get_radius(){ return _radius; }
	int intersect(Ray& ray, double& dist);
	bool intersectBox(AABB& box);
	AABB get_box();
	Color get_color(Vector3D& pos);

private:
	Vector3D _center;
	double _radius, _sq_radius, _r_radius;
	Vector3D _vn, _ve, _vc;
};

class PlaneForPrim : public Primitive
{
public:
	PlaneForPrim(Plane& plane):_plane(plane)
	{
		_vn = Vector3D(0, 1, 0);
		if (_vn == _plane._n) _vn = Vector3D(1, 0, 0);
		_vx = _plane._n.cross(_vn);
		_vy = _plane._n.cross(_vx);
		_vx.normalize();
		_vy.normalize();
	}
	~PlaneForPrim(){}
	int get_type(){ return PLANE; }
	Vector3D& get_normal(){ return _plane._n; }
	double get_d(){ return _plane._d; }
	int intersect(Ray& ray, double& dist);
	Vector3D get_normal(Vector3D& pos){ return _plane._n; }
	bool intersectBox(AABB& box);
	AABB get_box(){ return AABB(Vector3D(-10000, -10000, -10000), Vector3D(20000, 20000, 20000)); }
	Color get_color(Vector3D& pos);

private:
	Plane _plane;
	Vector3D _vn, _vx, _vy;
};


class Box : public Primitive
{
public:
	int get_type() { return BOX; }
	Box(): _box(Vector3D(0, 0, 0), Vector3D(0, 0, 0)), _grid(NULL){}
	Box(AABB& box): _box(box), _grid(NULL){}
	int intersect(Ray& _Ray, double& dist );
	bool intersectBox(AABB& box) { return _box.intersect( box ); }
	Vector3D get_normal(Vector3D&);
	bool contains(Vector3D& pos) { return _box.contains( pos ); }
	Vector3D& get_pos() { return _box.get_pos(); }
	Vector3D& get_size() { return _box.get_size(); }
	double get_grid_x(int idx) { return _grid[idx << 1]; }
	double get_grid_y(int idx) { return _grid[(idx << 1) + 1]; }
	void set_light(bool a_Light );
	double calculateShade(Vector3D& pos);
	AABB get_box() { return _box; }
	AABB _box;
	double* _grid;
};

class Triangle : public Primitive
{
public:
	Triangle(Vector3D p1, Vector3D p2, Vector3D p3);
	~Triangle(){}
	int get_type(){ return TRIANGLE;}
	Vector3D get_normal(Vector3D& pos);
	int intersect(Ray& ray, double& dist);
	bool intersectBox(AABB& box);
	Color get_color(Vector3D& pos);
	AABB get_box();
	Vector3D _p3, _p1, _p2, _vn1, _vn2, _vn3, _vt1, _vt2, _vt3;
	Vector3D _n;
};
class ObjList
{
public:
	ObjList(): _next(NULL){}
	~ObjList() { delete _next; }
	void set_primitive(Primitive* prim){ _primitive = prim; }
	void set_next(ObjList* next){ _next = next; }
	Primitive* get_primitive(){ return _primitive; }
	ObjList* get_next(){ return _next; }
private:
	Primitive* _primitive;
	ObjList* _next;
};

class Scene
{
public:
	Scene(): _primitives(0), _grid(0), _lights(0){}
	~Scene();
	void initScene();
	void buildGrid();
	ObjList** get_grid(){ return _grid; }
	int get_num_prim(){ return _n_primitives; }
	Primitive* get_primitive(int idx){ return _primitives[idx]; }
	int get_num_light(){ return _n_lights; }
	Primitive* get_light(int idx){ return _lights[idx]; }
	AABB& get_extends(){ return _extends; }
	void loadObj(Vector3D& axis, const char* file_name);
	void loadSketchUp(Vector3D& axis, std::string file_name);

private:
	Primitive** _primitives;
	Primitive** _lights;
	ObjList** _grid;
	int _n_primitives, _n_lights;
	AABB _extends;
};


#endif