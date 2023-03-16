#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

	double a = dot(r.d, r.d);
	double b = dot(2 * (r.o - o), r.d);
	double c = dot(r.o - o, r.o - o) - r2;

	double D = b * b - 4 * a * c;
	if (D < 0) {
		return false;
	}
	t1 = (- b - sqrt(D)) / (2 * a);
	t2 = (- b + sqrt(D)) / (2 * a);

  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	double t1;
	double t2;
	bool test_intersection = test(r, t1, t2);
	if (test_intersection) {
		if ((t1 >= r.min_t) && (t1 <= r.max_t)) {
			return true;
		}
		else {
			return (t2 >= r.min_t) && (t2 <= r.max_t);
		}
	}
	else {
		return false;
	}
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	double t1;
	double t2;
	bool test_intersection = test(r, t1, t2);
	bool has_intersect = false;
	double t;

	if (test_intersection) {
		if ((t1 >= r.min_t) && (t1 <= r.max_t)) {
			has_intersect = true;
			t = t1;
		}
		else {
			has_intersect = (t2 >= r.min_t) && (t2 <= r.max_t);
			if (has_intersect) {
				t = t2;
			}
		}
	}
	else {
		has_intersect = false;
	}
	if (has_intersect) {
        r.max_t = t;
		i->t = t;
		i->n = r.o + t * r.d - o;
        i->n.normalize();
		i->primitive = this;
		i->bsdf = this->get_bsdf();
	}
	return has_intersect;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
