#include "stdafx.h"
#include "triangle.h"
#include <fstream>
#include <cmath>
using namespace std;

ofstream ffout("box.txt");

float max(float a,float b)
{
	if(a>b) return a;
	else return b;
}

float min(float a,float b)
{
	if(a>b) return b;
	else return a;
}

float det(vector3& a,vector3& b,vector3& c)
{
	return a.x*(b.y*c.z-b.z*c.y)-a.y*(b.x*c.z-b.z*c.x)+a.z*(b.x*c.y-b.y*c.x);
}

bool check(float a_max,float a_min,float b_max,float b_min)
{
	if(b_min>a_max) return 1;
	if(a_min>b_max) return 1;
	return 0;
}

void chaji(vector3 a,vector3 b,vector3& out)
{
	out.x=a.y*b.z-a.z*b.y;
	out.y=a.z*b.x-a.x*b.z;
	out.z=a.x*b.y-a.y*b.x;
	NORMALIZE(out);
	out=-out;
}

void tri_find(vector3 p1,vector3 p2,vector3 p3,vector3 n,float& mi,float& ma)
{
	n.Normalize();
	float pp1,pp2,pp3;
	pp1=n.x*p1.x+n.y*p1.y+n.z*p1.z;
	pp2=n.x*p2.x+n.y*p2.y+n.z*p2.z;
	pp3=n.x*p3.x+n.y*p3.y+n.z*p3.z;
	mi=min(min(pp1,pp2),pp3);
	ma=max(pp1,max(pp2,pp3));
}

void box_find(vector3 p1,vector3 p2,vector3 n,float& mi,float& ma) 
{
	n.Normalize();
	float x[2],y[2],z[2];
	float mmin=100000,mmax=-100000;
	x[0]=p1.x;
	x[1]=p2.x;
	y[0]=p1.y;
	y[1]=p2.y;
	z[0]=p1.z;
	z[1]=p2.z;
	for(int i=0;i<2;i++)
		for(int j=0;j<2;j++)
			for(int k=0;k<2;k++)
			{
				float m1;
				m1=x[i]*n.x+y[j]*n.y+z[k]*n.z;
				if(m1>mmax) mmax=m1;
				if(m1<mmin) mmin=m1;
			}
	mi=mmin;
	ma=mmax;
}

aabb Triangle::Getaabb()
	{
		vector3 o;
		o.x=max(max(t_P1.x,t_P2.x),t_P3.x);
		o.y=max(max(t_P1.y,t_P2.y),t_P3.y);
		o.z=max(max(t_P1.z,t_P2.z),t_P3.z);
		vector3 s;
		s.x=min(min(t_P1.x,t_P2.x),t_P3.x);
		s.y=min(min(t_P1.y,t_P2.y),t_P3.y);
		s.z=min(min(t_P1.z,t_P2.z),t_P3.z);
		return aabb(s,o-s);
	}

int Triangle::Intersect(Ray& ray,float& dist)
{
	vector3 r0=ray.GetOrigin();
	vector3 rd=ray.GetDirection();
	rd.Normalize();
	vector3 s=t_P1-r0,e1=t_P1-t_P2,e2=t_P1-t_P3;
	float sim=det(rd,e1,e2);
	float t=det(s,e1,e2)/sim;
	float bb=det(rd,s,e2)/sim;
	float rr=det(rd,e1,s)/sim;
	if ((t>0) && (bb>=0) && (rr>=0) && (bb+rr<=1))
		if((dist >= t*rd.Length()) )
		{
			//cout << p_Name << endl;
			dist=t*rd.Length();
			//cout << dist << endl;
			return 1;
		}
		
	return 0;
}

bool Triangle::IntersectBox(aabb& box)
{
	///return 1;
	vector3 b_min=box.GetPos();
	vector3 b_max=b_min+box.GetSize();
	//ffout << b_min.x << ' ' << b_min.y << ' ' << b_min.z << endl;
	//ffout << b_max.x << ' ' << b_max.y << ' ' << b_max.z << endl;
	vector3 e_x=vector3(1,0,0),e_y=vector3(0,1,0),e_z=vector3(0,0,1);
	bool o[13];
	o[0]=check(max(max(t_P1.x,t_P2.x),t_P3.x),min(min(t_P1.x,t_P2.x),t_P3.x),b_max.x,b_min.x);
	if(o[0]) return 0;
	o[1]=check(max(max(t_P1.y,t_P2.y),t_P3.y),min(min(t_P1.y,t_P2.y),t_P3.y),b_max.y,b_min.y);
	if(o[1]) return 0;
	o[2]=check(max(max(t_P1.z,t_P2.z),t_P3.z),min(min(t_P1.z,t_P2.z),t_P3.z),b_max.z,b_min.z);
	if(o[2]) return 0;
	float t_b_max,t_b_min;
	float t_t_max,t_t_min;
	box_find(b_min,b_max,t_N,t_b_min,t_b_max);
	tri_find(t_P1,t_P2,t_P3,t_N,t_t_min,t_t_max);
	o[3]=check(t_t_max,t_t_min,t_b_max,t_b_min);
	if(o[3]) return 0;
	vector3 nor;

	chaji(e_x,t_P2-t_P1,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_x,t_P3-t_P2,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_x,t_P1-t_P3,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_y,t_P2-t_P1,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_y,t_P3-t_P2,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_y,t_P1-t_P3,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_z,t_P2-t_P1,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_z,t_P3-t_P2,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	chaji(e_z,t_P1-t_P3,nor);
	tri_find(t_P1,t_P2,t_P3,nor,t_t_min,t_t_max);
	box_find(b_min,b_max,nor,t_b_min,t_b_max);
	if(check(t_t_max,t_t_min,t_b_max,t_b_min)) return 0;

	return 1;
}

Color Triangle::GetColor(vector3& pos)
{
	if(t_Img!=NULL)
	{
		vector3 s=t_P1,e1=t_P1-t_P2,e2=t_P1-t_P3;
		float sim=det(pos,e1,e2);
		float t=det(s,e1,e2)/sim;
		float bb=det(pos,s,e2)/sim;
		float rr=det(pos,e1,s)/sim;
		float aa=1-bb-rr;
		//vector3 res=aa*t_P1+bb*t_P2+rr*t_P3;
		//ffout << pos.x << ' ' << pos.y << ' ' <<pos.z << "to" << res.x  << ' ' << res.y << ' ' <<res.z << endl;
		//cout << aa << ' ' << bb << ' ' << rr << endl;
		vector3 dot=aa*t_T1+bb*t_T2+rr*t_T3;
		//cout << dot.x << ' ' << dot.y << endl;
		//int x=t_Img->width-floor(dot.x*t_Img->width)-1;
		int y=t_Img->height-floor(dot.y*t_Img->height)-1;
		int x=floor(dot.x*t_Img->width)-1;
		//int y=floor(dot.y*t_Img->height);
		ffout << x << ' ' << y << endl;
		ffout << (t_Img->width) << ' ' << (t_Img->height) << endl << endl;
		CvScalar p;
		p = cvGet2D(t_Img,y,x);//j 是列，i是行，点在图像上的坐标
		float a = 1.0f*p.val[0]/255;//B
		float b = 1.0f*p.val[1]/255;//G
		float c = 1.0f*p.val[2]/255;//R
		//ffout << c << ' ' << b << ' ' << a << endl;
		return Color(c,b,a);
		return Color(c,b,a)*p_Material->GetColor();
	}
	else
	{
		ffout << 0 << endl;
		return p_Material->GetColor();
	}
}