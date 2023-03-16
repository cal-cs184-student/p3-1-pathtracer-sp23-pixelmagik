#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
	
	// Miller Trumbore implementation for ray-triangle intersection
	Vector3D E1 = p2 - p1;
	Vector3D E2 = p3 - p1;
	Vector3D S = r.o - p1;
	Vector3D S1 = cross(r.d, E2);
	Vector3D S2 = cross(S, E1);

	double s1e1 = dot(S1, E1);
	double t = dot(S2, E2) / s1e1;
	double b1 = dot(S1, S) / s1e1;
	double b2 = dot(S2, r.d) / s1e1;

	// Update max_t for ray to t for nearest intersection
	double t_min = r.min_t;
	double t_max = r.max_t;
	r.max_t = t;

  return (t <= t_max) && (t >= t_min) && (b1 <= 1) && (b1 >= 0) && (b2 <= 1) && (b2 >= 0) && (b1 + b2 <= 1);

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

    // Miller Trumbore implementation for ray-triangle intersection
    Vector3D E1 = p2 - p1;
    Vector3D E2 = p3 - p1;
    Vector3D S = r.o - p1;
    Vector3D S1 = cross(r.d, E2);
    Vector3D S2 = cross(S, E1);

    double s1e1 = dot(S1, E1);
    double t = dot(S2, E2) / s1e1;
    double b1 = dot(S1, S) / s1e1;
    double b2 = dot(S2, r.d) / s1e1;

    // Update max_t for ray to t for nearest intersection
    double t_min = r.min_t;
    double t_max = r.max_t;

    bool has_intersect = (t <= t_max) && (t >= t_min) && (b1 <= 1) && (b1 >= 0) && (b2 <= 1) && (b2 >= 0) && (b1 + b2 <= 1);

    if (has_intersect) {
        r.max_t = t;
        isect->t = t;
        isect->n = b1 * n1 + b2 * n2 + (1 - b1 - b2) * n3;
        isect->primitive = this;
        isect->bsdf = this->get_bsdf();
    }

    return has_intersect;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
