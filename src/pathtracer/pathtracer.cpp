#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();



  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  for (int i = 0; i < num_samples; i++) {

      Vector3D w_in = hemisphereSampler->get_sample();
      Vector3D ref = isect.bsdf->f(w_out, w_in);
      Ray in_ray = Ray(hit_p, o2w * w_in);
      Intersection isect2 = Intersection();
      Vector3D Li = Vector3D(0, 0, 0);

      in_ray.min_t = EPS_F;

      if (bvh->intersect(in_ray, &isect2)) {
           Li = isect2.bsdf->get_emission();
      }

      L_out += 2 * PI * Li *  ref * cos_theta(w_in);
  }
  L_out /= num_samples;

  return L_out;

  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading

}


Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D Li;

  for (auto light_src : scene->lights) {
    auto wi = Vector3D();
    double distToLight;
    double pdf;

    if (light_src->is_delta_light()) {
        // sample once because point src
        Li = light_src->sample_L(hit_p, &wi, &distToLight, &pdf);
        wi.normalize();
        Vector3D ref = isect.bsdf->f(w_out, w2o * wi);
        Intersection isect2 = Intersection();
        Ray in_ray = Ray(hit_p,  wi);

        in_ray.min_t = EPS_F;
        in_ray.max_t = distToLight - EPS_F;

        if ((cos_theta(w2o * wi) < 0) || bvh->intersect(in_ray, &isect2)) {
            Li = 0;
        }
        // calculate output L
        L_out += ns_area_light * Li * ref * cos_theta(w2o * wi) / pdf;
    } else {
        Vector3D L_out_ext;     // average irradiance from light
        for (int i = 0; i < ns_area_light; i++) {

            // sample one location in the light
            Li = light_src->sample_L(hit_p, &wi, &distToLight, &pdf);
            wi.normalize();
            // create ray in the appropriate direction
            Ray in_ray = Ray(hit_p,  wi);
            Intersection isect2 = Intersection();

            // calculate reflectance
            Vector3D ref = isect.bsdf->f(w_out, w2o * wi);

            in_ray.min_t = EPS_F;
            in_ray.max_t = distToLight - EPS_F;

            float rand_val = ((float) rand() / (RAND_MAX));
            float extinct_dist = -(::log(rand_val) / (sig_a + sig_s));

            // shadow ray: ensure there is no blocking object between light and hit_p
            if ((cos_theta(w2o * wi) < 0) || bvh->intersect(in_ray, &isect2)) {
                Li = 0;
            } else if (extinct_dist < distToLight) {

                float scatter_r = ((float) rand() / (RAND_MAX));
                float scatter_prob = sig_s / (sig_s + sig_a);

                if (scatter_r > scatter_prob) {
                    auto scatter_sampler = SchlickWeightedSphereSampler3D();
                    float scatter_pdf;
                    Vector3D scatter_dir = scatter_sampler.get_sample(w2o * wi, k, &scatter_pdf);
                    scatter_dir.normalize();

                }
            }

            // calculate total reflectance
            L_out_ext += Li * ref * cos_theta(w2o * wi) / pdf;
        }

        // add light to total irradiance
        L_out += L_out_ext;
    }
  }

  return L_out / num_samples;


}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();


}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  Vector3D L;

  if (direct_hemisphere_sample) {
      L = estimate_direct_lighting_hemisphere(r, isect);
  } else {
      L = estimate_direct_lighting_importance(r, isect);
  }

  return L;


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out = Vector3D(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.

  L_out += one_bounce_radiance(r, isect);
  Vector3D w_in;
  double pdf;
  double prob = 0;

  Vector3D ref = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  Ray in_ray = Ray(hit_p, o2w * w_in);
  in_ray.depth = r.depth - 1;

//  cout << r.depth;
//  cout << "\n ";
//  cout << L_out.x;
//  cout << "\n ";

  if (coin_flip(prob) || (in_ray.depth <= 0)) {
      return L_out;
  }

  in_ray.min_t = EPS_F;
  Intersection isect2;
  if (bvh->intersect(in_ray, &isect2)) {
      Vector3D fLcos = ref * at_least_one_bounce_radiance(in_ray, isect2) * abs_cos_theta(w_in);
      L_out += fLcos / pdf / (1 - prob);
  }

  return L_out;
}


Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return L_out;

  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  //L_out = zero_bounce_radiance(r, isect);

  return L_out;
}

    void PathTracer::raytrace_pixel(size_t x, size_t y) {
        // TODO (Part 1.2):
        // Make a loop that generates num_samples camera rays and traces them
        // through the scene. Return the average Vector3D.
        // You should call est_radiance_global_illumination in this function.

        // TODO (Part 5):
        // Modify your implementation to include adaptive sampling.
        // Use the command line parameters "samplesPerBatch" and "maxTolerance"

        Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
        Vector3D total = Vector3D();
        int num_samples = 0;              // actual number of samples used

        double width = sampleBuffer.w;
        double height = sampleBuffer.h;

        double s_1 = 0;
        double s_2 = 0;

        for (int i = 0; i < ns_aa; i++) {
            if (num_samples % 32 == 0) {
                double mean = s_1 / (double)num_samples;
                double variance = (1.0 / (num_samples - 1)) * (s_2 - (pow(s_1, 2) / num_samples));
                double standard_dev = sqrt(variance);

                double I = 1.96 * (standard_dev / sqrt(num_samples));
                if (I <= maxTolerance * mean) {
                    break;
                }
            }
            Vector2D sample = gridSampler->get_sample();
            Ray ray = camera->generate_ray((origin.x + sample.x) / width, (origin.y + sample.y) / height);
            ray.depth = max_ray_depth;
            Vector3D radiance = PathTracer::est_radiance_global_illumination(ray);

            s_1 += radiance.illum();
            s_2 += pow(radiance.illum(), 2);
            num_samples++;

            total += radiance;
        }

        sampleBuffer.update_pixel(total/(double)num_samples, x, y);
        sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
    }


void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
