#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

Vector3D ray_point(const Ray& r, double t) {
	return r.o + t * r.d;
}

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

	Vector3D tmin = Vector3D();
	Vector3D tmax = Vector3D();
	double temp;
	double new_t0;
	double new_t1;

	tmin.x = (min.x - r.o.x) / r.d.x;
	tmax.x = (max.x - r.o.x) / r.d.x;

	if (r.d.x < 0) {
		temp = tmin.x;
		tmin.x = tmax.x;
		tmax.x = temp;
	}

	new_t0 = tmin.x;
	new_t1 = tmax.x;

	tmin.y = (min.y - r.o.y) / r.d.y;
	tmax.y = (max.y - r.o.y) / r.d.y;

	if (r.d.y < 0) {
		temp = tmin.y;
		tmin.y = tmax.y;
		tmax.y = temp;
	}

	if (tmin.y > new_t0) {
		new_t0 = tmin.y;
	}

	if (tmax.y < new_t1) {
		new_t1 = tmax.y;
	}

	tmin.z = (min.z - r.o.z) / r.d.z;
	tmax.z = (max.z - r.o.z) / r.d.z;

	if (r.d.z < 0) {
		temp = tmin.z;
		tmin.z = tmax.z;
		tmax.z = temp;
	}

	if (tmin.z > new_t0) {
		new_t0 = tmin.z;
	}

	if (tmax.z < new_t1) {
		new_t1 = tmax.z;
	}

	if ((new_t0 > new_t1) || (new_t0 < t0) || (new_t1 > t1) || (new_t0 < r.min_t) || (new_t1 > r.max_t)) {
		return false;
	}

	t0 = new_t0;
	t1 = new_t1;

  return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
