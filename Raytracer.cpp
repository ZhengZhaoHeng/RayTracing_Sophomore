

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

void Camera::initRender(Vector3D& pos, Vector3D& target)
{
	_sr._x = GRIDSIZE / _scene->get_extends().get_size()._x;
	_sr._y = GRIDSIZE / _scene->get_extends().get_size()._y;
	_sr._z = GRIDSIZE / _scene->get_extends().get_size()._z;
	_cw = _scene->get_extends().get_size() *(1.0 / GRIDSIZE);
	_n_id = 0;
	_origin = Vector3D(0, 0, -5);
	_p1 = Vector3D(-4, 3, 0);
	_p2 = Vector3D(4, 3, 0);
	_p3 = Vector3D(4, -3, 0);
	_p4 = Vector3D(-4, -3, 0);
	Vector3D zaxis = target - pos;
	zaxis.normalize();
	Vector3D up(0, 1, 0);
	Vector3D xaxis = up.cross(zaxis);
	Vector3D yaxis = zaxis.cross(xaxis);
	Matrix m;
	m._cell[0] = xaxis._x;
	m._cell[1] = xaxis._y;
	m._cell[2] = xaxis._z;
	m._cell[4] = yaxis._x;
	m._cell[5] = yaxis._y;
	m._cell[6] = yaxis._z;
	m._cell[8] = zaxis._x;
	m._cell[9] = zaxis._y;
	m._cell[10] = zaxis._z;
	m.invert();
	m._cell[3] = pos._x;
	m._cell[7] = pos._y;
	m._cell[11] = pos._z;
	_origin = m.transform(_origin);
	_p1 = m.transform(_p1);
	_p2 = m.transform(_p2);
	_p3 = m.transform(_p3);
	_p4 = m.transform(_p4);
	_dx = (_p2 - _p1) / _width;
	_dy = (_p4 - _p1) / _height;
}

void Camera::renderPart(int x1, int x2, int y1, int y2)
{
	Vector3D src(0, 0, -5);
	Primitive* last_primitive = NULL;
	for (int j = y1; j < y2; j++)
	{
		Vector3D cur_pos = _p1 + j * _dy;
		std::cout << j << std::endl;
		for (int i = x1; i < x2; i++)
		{
			//std::cout << i << ' ' << j << std::endl;
			Color temp(0, 0, 0);
			double dist = 0;
			Primitive* prim = renderRay(cur_pos, temp);
			int red, green, blue;
			if (prim != last_primitive)
			{
				last_primitive = prim;
				Color temp(0, 0, 0);
				for (int tx = -1; tx < 2; tx++)
					for (int ty = -1; ty < 2; ty++)
					{
						Primitive* prim = renderRay(cur_pos + _dx * tx / 2 + _dy * ty / 2, temp);
					}
				red = (int)(temp._x * 256 / 9);
				green = (int)(temp._y * 256 / 9);
				blue = (int)(temp._z * 256 / 9);
			}
			else
			{
				red = (int)(temp._x * 256);
				green = (int)(temp._y * 256);
				blue = (int)(temp._z * 256);
			}
			red = std::min(red, 255);
			green = std::min(green, 255);
			blue = std::min(blue, 255);
			CvScalar color = CV_RGB(red, green, blue);
			for (int k = 0; k < 3; k++)
			{
				_img->imageData[j * _img->widthStep + i * 3 + k] = color.val[k];
			}
			cur_pos = cur_pos + _dx;
		}
	}
}

void Camera::render()
{
	const int n_thread = 1;
	int width_l[n_thread];
	int width_r[n_thread];
	int height_u[n_thread];
	int height_d[n_thread];
	for (int i = 0; i < n_thread; i++)
	{
		width_l[i] = i * _width / n_thread;
		width_r[i] = (i + 1) * _width / n_thread;
		height_u[i] = i * _height / n_thread;
		height_d[i] = (i + 1) * _height / n_thread;
	}
	std::thread t[n_thread * n_thread];
	for (int i = 0; i < n_thread; i++)
	{
		for (int j = 0; j < n_thread; j++)
		{
			t[i * n_thread + j] = std::thread(&Camera::renderPart, this, width_l[i], width_r[i], height_u[j], height_d[j]);
		}
	}

	for (int i = 0; i < n_thread * n_thread; i++)
	{
		t[i].join();
	}
}

double Camera::calculateShade(Primitive* prim, Vector3D& pos, Vector3D& l)
{
	int n_sample = 1;
	int cnt = 0;
	if (prim->get_type() == Primitive::SPHERE)
	{
		Sphere* sphere = (Sphere*)prim;
		l = sphere->get_center() - pos;
		l.normalize();
		for (int i = 0; i < n_sample; i++)
		{
			double theta = double(rand() % 2000) / 1000;
			double phi = double(rand() % 2001) / 2000;
			theta = theta * PI;
			phi = phi * PI;
			Vector3D pnt = Vector3D(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)) * sphere->get_radius() + sphere->get_center();
			Vector3D dir = pnt - pos;
			double dist = dir.length();
			dir.normalize();
			Primitive* temp  = NULL;
			_n_id++;
			if (findNest(Ray(pos + EPS * dir, dir, _n_id), dist, temp))
			{
				if (temp == prim) cnt++;
			}
		}
	}
	if (prim->get_type() == Primitive::BOX)
	{
		Box* box = (Box*)prim;
		double delta_x = box->get_size()._x * 0.25;
		double delta_y = box->get_size()._z * 0.25;
		l = box->get_pos() + box->get_size() * 0.5 - pos;
		l.normalize();
		for (int i = 0; i < n_sample; i++)
		{
			Vector3D lp(box->get_grid_x(i & 15) + (double(rand() % 2001) / 2000) * delta_x,
				        box->get_pos()._y,
						box->get_grid_y(i & 15) + (double(rand() % 2001) / 2000) * delta_y);
			Vector3D dir = lp - pos;
			double dist = dir.length();
			dir.normalize();
			Primitive* temp = NULL;
			_n_id++;
			if (findNest(Ray(pos + dir * EPS, dir, _n_id), dist, temp))
			{
				if (temp == box) cnt++;
			}
		}
	}
	return (double)cnt / n_sample;
}

Primitive* Camera::rayTracing(Ray& ray, Color& color, int depth, double r_index, double dist)
{
	if (depth > TRACEDEPTH)  return NULL;
	dist = 1e6;
	Primitive* prim = NULL;
	int result;
	//find the nearest primitive.
	if (!(result = findNest(ray, dist, prim))) return 0;
	if (prim->isLight())
	{
		color = prim->get_material().get_color();
	}
	else
	{
		Vector3D pnt = ray.get_origin() + ray.get_direction() * dist;
		//trace light
		for (int i = 0; i < _scene->get_num_light(); i++)
		{

			Primitive* light = _scene->get_light(i);
			Vector3D l;
			double shadow = calculateShade(light, pnt, l);
			/*double shadow = 1.0;
			//calculate the shadow
			if (light->get_type() == Primitive::SPHERE)
			{
				double l_dist = l.length();
				l.normalize();
				_n_id++;
				Ray l_ray(pnt + l * EPS, l, _n_id);
				Primitive* obstacle;
				findNest(l_ray, l_dist, obstacle);
				if (obstacle != NULL && obstacle != light)
				{
					shadow = 0;
				}
			}*/
			//std::cout << shadow << std::endl;
			if (shadow > EPS)
			{
				//calculate the diffuse component
				Vector3D n = prim->get_normal(pnt);
				if (prim->get_material().get_diff() > 0)
				{
					double d = dot(n, l);
					if (d > 0)
					{
						double diffuse = d * prim->get_material().get_diff() * shadow;
						color = color + diffuse * prim->get_color(pnt) * light->get_color(pnt);
					}
				}
				//calculate the specular component(high light)
				if (prim->get_material().get_spec() > EPS)
				{
					Vector3D v = ray.get_direction();
					Vector3D r = l - 2 * dot(l, n) * n;
					double d = dot(v, r);
					if (d > 0)
					{
						double spec = pow(d, 20) * prim->get_material().get_spec() * shadow;
						color = color + spec * light->get_color(pnt);
					}
				}
			}
		}
		//calculate the reflection
		double refl_rate = prim->get_material().get_refl();
		if (refl_rate > EPS && depth < TRACEDEPTH)
		{
			double drefl_rate = prim->get_material().get_diff_refl();
			if (drefl_rate > 0 && depth < 2)
			{
				int n_sample = 1;
				Vector3D n =  prim->get_normal(pnt);
				Vector3D rp = ray.get_direction() - 2 * dot(ray.get_direction(), n) * n;
				Vector3D rn1  = Vector3D(rp._z, rp._y ,-rp._x);
				Vector3D rn2 = rp.cross(rn1);
				refl_rate = refl_rate / n_sample;
				for (int i = 0; i < n_sample; i++)
				{
					double xoffs, yoffs;
					do
					{
						xoffs = (double)(rand() % 2001) / 2000 * drefl_rate;
						yoffs = (double)(rand() % 2001) / 2000 * drefl_rate;
					}
					while ((sqr(xoffs) + sqr(yoffs)) > sqr(drefl_rate));
					Vector3D r = rp + rn1 * xoffs + rn2 * yoffs * drefl_rate;
					r.normalize();
					double refl_dist = 0;
					Color temp(0, 0, 0);
					_n_id++;
					rayTracing(Ray(pnt + r * EPS, r, _n_id), temp, depth + 1, r_index, refl_dist);
					color = color + temp * refl_rate * prim->get_color(pnt);
				}
			}
			else
			{
				Vector3D n = prim->get_normal(pnt);
				Vector3D r = ray.get_direction() - 2 * dot(ray.get_direction(), n) * n;
				Color refl_color(0, 0, 0);
				double refl_dist = 0;
				_n_id++;
				rayTracing(Ray(pnt + r * EPS, r, _n_id), refl_color, depth + 1, r_index, refl_dist);
				color = color + refl_rate * refl_color * prim->get_color(pnt);
			}
		}
		//calculate the refraction
		double refr_rate = prim->get_material().get_refr();
		if (refr_rate > EPS && depth < TRACEDEPTH)
		{
			double r2_index = prim->get_material().get_refr_index();
			double n = r_index / r2_index;
			Vector3D v_n = prim->get_normal(pnt) * result;
			double cos_i = -dot(ray.get_direction(), v_n);
			double cos_r = sqrt(1.0 - sqr(n) * (1 - sqr(cos_i)));
			if (cos_r > EPS)
			{
				Vector3D t = n * ray.get_direction() + (n * cos_i - cos_r) * v_n;
				//std::cout << t.length() << std::endl;
				Color refr_color(0, 0, 0);
				double refr_dist = 0;
				_n_id ++;
				rayTracing(Ray(pnt + t * EPS, t, _n_id), refr_color, depth + 1, r2_index, dist);
				Color reduction = prim->get_color(pnt) * 0.15 * -dist;
				Color transparent = Color(exp(reduction._x), exp(reduction._y), exp(reduction._z));
				color = color + transparent * refr_color;
			}
		}

	}
	return prim;
}

int Camera::findNest(Ray& ray, double& dist, Primitive*& prim)
{
	int retval = MISS;
	Vector3D dir, src;
	Box e = _scene->get_extends();
	src = ray.get_origin();
	dir = ray.get_direction();
	Vector3D cb, tmax, tdelta, cell;
	cell = (src - e.get_pos()) * _sr;
	int step_x, out_x, x = (int)cell._x;
	int step_y, out_y, y = (int)cell._y;
	int step_z, out_z, z = (int)cell._z;
	if ((x < 0) || (x >= GRIDSIZE) || (y < 0) || (y >= GRIDSIZE) || (z < 0) || (z >= GRIDSIZE))
		return 0;
	if (dir._x > 0)
	{
		step_x = 1;
		out_x = GRIDSIZE;
		cb._x = e.get_pos()._x + (x + 1) * _cw._x;
	}
	else
	{
		step_x = -1;
		out_x = -1;
		cb._x = e.get_pos()._x + x * _cw._x;
	}
	if (dir._y > 0)
	{
		step_y = 1;
		out_y = GRIDSIZE;
		cb._y = e.get_pos()._y + (y + 1) * _cw._y;
	}
	else
	{
		step_y = -1;
		out_y = -1;
		cb._y = e.get_pos()._y + y * _cw._y;
	}
	if (dir._z > 0)
	{
		step_z = 1;
		out_z = GRIDSIZE;
		cb._z = e.get_pos()._z + (z + 1) * _cw._z;
	}
	else
	{
		step_z = -1;
		out_z = -1;
		cb._z = e.get_pos()._z + z * _cw._z;
	}
	double rxr, ryr, rzr;
	if ((dir._x < -EPS) || (dir._x > EPS))
	{
		rxr = 1.0 / dir._x;
		tmax._x = (cb._x - src._x) * rxr;
		tdelta._x = _cw._x * step_x * rxr;
	}
	else tmax._x = 1000000;
	if ((dir._y < -EPS) || (dir._y > EPS))
	{
		ryr = 1.0 / dir._y;
		tmax._y = (cb._y - src._y) * ryr;
		tdelta._y = _cw._y * step_y * ryr;
	}
	else tmax._y = 1000000;
	if ((dir._z < -EPS) || (dir._z > EPS))
	{
		rzr = 1.0 / dir._z;
		tmax._z = (cb._z - src._z) * rzr;
		tdelta._z = _cw._z * step_z * rzr;
	}
	else tmax._z = 1000000;
	//find the first hit
	ObjList* list = 0;
	ObjList** grid = _scene->get_grid();
	prim = 0;
	while (1)
	{
		bool flag = false;
		list = grid[x + (y << GRIDSHIFT) + (z << (GRIDSHIFT * 2))];
		int i  = 0;
		while (list)
		{
			//std::cout << x << ' ' << y << ' ' << z << (int)list << std::endl;
			Primitive* temp = list->get_primitive();
			int result;
			if (temp->get_last_ray_id() != ray.get_id())
			{
				result = temp->intersect(ray, dist);
				if (result != 0)
				{
					retval = result;
					prim = temp;
					flag = true;
					break;
				}
			}
			list = list->get_next();
		}
		if (flag) break;
		if (tmax._x < tmax._y)
		{
			if (tmax._x < tmax._z)
			{
				x += step_x;
				if (x == out_x) return MISS;
				tmax._x += tdelta._x;
			}
			else
			{
				z += step_z;
				if (z == out_z) return MISS;
				tmax._z += tdelta._z;
			}
		}
		else
		{
			if (tmax._y < tmax._z)
			{
				y += step_y;
				if (y == out_y) return MISS;
				tmax._y += tdelta._y;
			}
			else
			{
				z += step_z;
				if (z == out_z) return MISS;
				tmax._z += tdelta._z;
			}
		}
	}
	// test all
	while (1)
	{
		list = grid[x + (y << GRIDSHIFT) + (z << (GRIDSHIFT * 2))];
		while (list)
		{
			Primitive* temp = list->get_primitive();
			int result;
			if (temp->get_last_ray_id() != ray.get_id())
			{
				result = temp->intersect(ray, dist);
				if (result != 0)
				{
					prim = temp;
					retval = result;
				}
			}
			list = list->get_next();
		}
		if (tmax._x < tmax._y)
		{
			if (tmax._x < tmax._z)
			{
				if (dist < tmax._x) break;
				x += step_x;
				if (x == out_x) break;
				tmax._x += tdelta._x;
			}
			else
			{
				if (dist < tmax._z) break;
				z += step_z;
				if (z == out_z) break;
				tmax._z += tdelta._z;
			}
		}
		else
		{
			if (tmax._y < tmax._z)
			{
				if (dist < tmax._y) break;
				y += step_y;
				if (y == out_y) break;
				tmax._y += tdelta._y;

			}
			else
			{
				if (dist < tmax._z) break;
				z += step_z;
				if (z == out_z) break;
				tmax._z += tdelta._z;
			}
		}
	}
	return retval;
}

Primitive* Camera::renderRay(Vector3D& pos, Color& color)
{
	Box e = _scene->get_extends();
	Vector3D dir = pos - _origin;
	dir.normalize();
	_n_id++;
	Ray r(_origin, dir, _n_id);
	if (!e.contains(_origin))
	{
		double bdist = 100000;
		if (e.intersect(r, bdist))
		{
			r.set_origin(_origin + (bdist + EPS) * dir);
		}
	}
	double dist = 0;
	return rayTracing(r, color, 1, 1.0, dist);
}