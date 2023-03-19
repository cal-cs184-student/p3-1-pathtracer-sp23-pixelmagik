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
        Vector3D trans_wi = w2o * wi;
        trans_wi.normalize();
        Vector3D ref = isect.bsdf->f(w_out, trans_wi);
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
            Vector3D trans_wi = w2o * wi;
            trans_wi.normalize();

            // create ray in the appropriate direction
            Ray in_ray = Ray(hit_p,  wi);
            Intersection isect2 = Intersection();

            // calculate reflectance
            Vector3D ref = isect.bsdf->f(w_out, trans_wi);

            in_ray.min_t = EPS_F;
            in_ray.max_t = distToLight - EPS_F;

            // shadow ray: ensure there is no blocking object between light and hit_p
            if ((cos_theta(trans_wi) < 0) || bvh->intersect(in_ray, &isect2)) {
                Li = 0;
            }

            // calculate total reflectance
            L_out_ext += Li * ref * cos_theta(trans_wi) / pdf;
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
  double prob = 0.3;

  Vector3D ref = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  w_in.normalize();
  Vector3D in_ray_dir = (w2o * w_in);
  in_ray_dir.normalize();
  Ray in_ray = Ray(hit_p, in_ray_dir);

  in_ray.depth = r.depth - 1;

  //  cout << r.depth;
  //  cout << "\n ";
  //  cout << L_out.x;
  //  cout << "\n ";

  if ((in_ray.depth <= 0) || coin_flip(prob)) {
      return L_out;
  }

  in_ray.min_t = EPS_F;
  Intersection isect2;
  if (bvh->intersect(in_ray, &isect2)) {
      Vector3D fLcos = ref * at_least_one_bounce_radiance(in_ray, isect2) * abs_cos_theta(in_ray_dir);
      L_out += fLcos / pdf / (1 - prob);
  }

  return L_out;
}


Vector3D PathTracer::est_radiance_global_illumination(const Ray& r) {
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

    int total_num_samples = (int) ns_aa;          // total samples to evaluate
    Vector2D origin = Vector2D((double) x, (double) y); // bottom left corner of the pixel
    bool adaptive = true;

    UniformGridSampler2D sampler2D = UniformGridSampler2D();

    Vector3D sum_illum = Vector3D(0, 0, 0);
    float s1 = 0.0;
    float s2 = 0.0;
    int num_samples = 0;
    float mean;
    float variance;

    while (num_samples < total_num_samples) {
        Vector2D rand_vec = sampler2D.get_sample() + origin;
        Ray single_ray = camera->generate_ray(rand_vec.x / (double) sampleBuffer.w,
                                              rand_vec.y / (double) sampleBuffer.h);
        single_ray.depth = max_ray_depth;
        Vector3D illumination = est_radiance_global_illumination(single_ray);
        sum_illum += illumination;
        num_samples++;

        if (adaptive && (num_samples % samplesPerBatch == 0)) {
            float pixel_illum = illumination.illum();
            s1 += pixel_illum;
            s2 += (pixel_illum * pixel_illum);
            mean = s1 / (float) num_samples;
            variance = (s2 - (s1 * s1 / (float) num_samples)) / (float)(num_samples - 1);
            if ((1.96 * sqrt(variance / (float) num_samples)) <= maxTolerance * mean) {
                break;
            }
        }
    }

    sampleBuffer.update_pixel(sum_illum / (double) num_samples, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

//
//    void PathTracer::raytrace_pixel(size_t x, size_t y) {
//        // TODO (Part 1.2):
//        // Make a loop that generates num_samples camera rays and traces them
//        // through the scene. Return the average Vector3D.
//        // You should call est_radiance_global_illumination in this function.
//        Vector3D average = {0.0,0.0,0.0};
//        double s1 = 0;
//        double s2 = 0;
//        int count = 0;
//        for (int i = 0; i < ns_aa; i++) {
//            count++;
//            Vector2D sample = gridSampler->get_sample();
//            Ray ray = camera->generate_ray((x + sample[0])/sampleBuffer.w, (y + sample[1])/sampleBuffer.h);
//            ray.depth = max_ray_depth;
//            Vector3D radiance = est_radiance_global_illumination(ray);
//            average += radiance;
//            s1 += radiance.illum();
//            s2 += radiance.illum() * radiance.illum();
//            if (i % samplesPerBatch == 0 && i != 0) {
//                double variance_2 = (1.0 / (i - 1.0)) * (s2 - (s1 * s1/i));
//                //cout<<(s2 - (s1 * s1/i))<<endl;
//                double mean = s1/i;
//                double bigI = 1.96 * std::sqrt(variance_2) / std::sqrt(i);
//                if (bigI <= (maxTolerance * mean)) {
//                    //cout<<s2 - (s1*s1/i)<<endl;
//                    //cout<<variance_2<<endl;
//                    //cout<<" exit"<<endl;
//                    break;
//                } else if (i == 64) {
//                    //cout <<"didn't break" <<endl;
//                }
//            }
//        }
//        average = average / count;
//        sampleBuffer.update_pixel(average, x, y);
//        sampleCountBuffer[x + y * sampleBuffer.w] = count;
//
//        // TODO (Part 5):
//        // Modify your implementation to include adaptive sampling.
//        // Use the command line parameters "samplesPerBatch" and "maxTolerance"
//        /*
//        int num_samples = ns_aa;          // total samples to evaluate
//        Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
//        average.illum();
//        sampleBuffer.update_pixel(Vector3D(0.2, 1.0, 0.8), x, y); */
//
//
//
//    }


void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
