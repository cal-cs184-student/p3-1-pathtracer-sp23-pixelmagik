#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;
  int box_count = 0;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    box_count++;
  }

  if (box_count == 0) {
      return NULL;
  }

  BVHNode* node = new BVHNode(bbox);

  if (box_count <= max_leaf_size) {
      node->start = start;
      node->end = end;
  }
  else {
      Vector3D big_centroid = bbox.centroid();
      int best_axis = -1;
      int best_cost = box_count;
      for (int i = 0; i < 3; i++) {
          int left_count = 0;
          for (auto p = start; p != end; p++) {
              BBox bb = (*p)->get_bbox();
              if (bb.centroid()[i] < bbox.min[i] + big_centroid[i]) {
                  left_count++;
              }
          }
          int cost = abs(2 * left_count - box_count);
          if (cost < best_cost) {
              best_axis = i;
              best_cost = cost;
          }
      }
      std::vector<Primitive*> *left = new std::vector<Primitive *>;
      std::vector<Primitive*> *right = new std::vector<Primitive *>;
      for (auto p = start; p != end; p++) {
          BBox bb = (*p)->get_bbox();
          if (bb.centroid()[best_axis] < big_centroid[best_axis]) {
              left->push_back(*p);
          }
          else {
              right->push_back(*p);
          }
      }
      node->l = construct_bvh(left->begin(), left->end(), max_leaf_size);
      node->r = construct_bvh(right->begin(), right->end(), max_leaf_size);
  }

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
      return false;
  }


  if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
          total_isects++;
          if ((*p)->has_intersection(ray)) {
              return true;
          }
      }
      return false;
  }

  total_isects++;

  return (has_intersection(ray, node->l) || has_intersection(ray, node->r));

}



bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

    bool hit = false;

    if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
        return false;
    }

    if (node->isLeaf()) {
        total_isects++;
        for (auto p = node->start; p != node->end; p++) {
            hit = ((*p)->intersect(ray, i)) || hit;
        }

        return hit;
    }

    total_isects++;
    bool left_isect = intersect(ray, i, node->l);
    bool right_isect = intersect(ray, i, node->r);

    return (left_isect || right_isect);
}

} // namespace SceneObjects
} // namespace CGL
