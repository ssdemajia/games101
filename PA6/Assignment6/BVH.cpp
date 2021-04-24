#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

void BVHAccel::SplitSAH(std::vector<Object*>& objects, std::vector<Object*>& left, std::vector<Object*>& right)
{
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
    {
        centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
    }
    
    int dim = centroidBounds.maxExtent();  // 选最长边来划分
    int minAxis = 1000000;
    int maxAxis = -minAxis;
    switch (dim) {
    case 0:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().x <
                   f2->getBounds().Centroid().x;
        });
        minAxis = objects[0]->getBounds().Centroid().x;
        maxAxis = objects.back()->getBounds().Centroid().x;
        break;
    case 1:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().y <
                   f2->getBounds().Centroid().y;
        });
        minAxis = objects[0]->getBounds().Centroid().y;
        maxAxis = objects.back()->getBounds().Centroid().y;
        break;
    case 2:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().z <
                   f2->getBounds().Centroid().z;
        });
        minAxis = objects[0]->getBounds().Centroid().z;
        maxAxis = objects.back()->getBounds().Centroid().z;
        break;
    }
    minAxis -= 1;
    maxAxis += 1;
    struct Bucket {
        int count=0;
        Bounds3 bound;
    };

    int axisLen = maxAxis - minAxis;
    int splitCount = 100;
    std::vector<Bucket> buckets(100);
    for (int cur = 0; cur < objects.size(); cur++) {
        auto obj = objects[cur];
        int i = -1;
        Vector3f center = obj->getBounds().Centroid();
        switch (dim) {
            case 0:
                i = splitCount * (center.x - minAxis) / axisLen;
                break;
            case 1:
                i = splitCount * (center.y - minAxis) / axisLen;
                break;
            case 2:
                i = splitCount * (center.z - minAxis) / axisLen;
                break;
        }
        
        assert(i != -1);
        if (buckets[i].count == 0)
        {
            buckets[i].bound = obj->getBounds();
        }
        else
        {
            buckets[i].bound = Union(buckets[i].bound, obj->getBounds());
        }
        buckets[i].count += 1;
    }

    float cost = std::numeric_limits<float>::max();
    float index = -1;
    for (int i = 1; i < splitCount; i++)
    {
        Bounds3 boundL;
        int countL = 0;
        for (int j = 0; j < i; j++) {
            if (buckets[j].count == 0)
                continue;
            if (countL == 0) {
                boundL = buckets[j].bound;
                countL = buckets[j].count;
            } else {
                boundL = Union(buckets[j].bound, boundL);
                countL += buckets[j].count;
            }
        }
        float areaL = boundL.SurfaceArea();
        float costL = countL * areaL;
        
        Bounds3 boundR;
        int countR = 0;
        for (int j = i; j < splitCount; j++) {
            if (buckets[j].count == 0)
                continue;
            if (countR == 0) {
                boundR = buckets[j].bound;
                countR = buckets[j].count;
            } else {
                boundR = Union(buckets[j].bound, boundR);
                countR += buckets[j].count;
            }
        }
        float areaR = boundR.SurfaceArea();
        float costR = countR * areaR;
        if (costR + costL < cost) {
            cost = costR + costL;
            index = countL;
        }
    }

    left = std::vector<Object*>(objects.begin(), objects.begin()+index);
    right = std::vector<Object*>(objects.begin()+index, objects.end());
}

void BVHAccel::SplitBVH(std::vector<Object*>& objects, std::vector<Object*>& left, std::vector<Object*>& right)
{
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
        centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();  // 选最长边来划分
    switch (dim) {
    case 0:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().x <
                   f2->getBounds().Centroid().x;
        });
        break;
    case 1:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().y <
                   f2->getBounds().Centroid().y;
        });
        break;
    case 2:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().z <
                   f2->getBounds().Centroid().z;
        });
        break;
    }

    auto beginning = objects.begin();
    auto middling = objects.begin() + (objects.size() / 2);
    auto ending = objects.end();

    left = std::vector<Object*>(beginning, middling);
    right = std::vector<Object*>(middling, ending);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        std::vector<Object*> leftshapes;
        std::vector<Object*> rightshapes;
        switch (splitMethod)
        {
            case SplitMethod::NAIVE:
                SplitBVH(objects, leftshapes, rightshapes);
                break;
        
            case SplitMethod::SAH:
                SplitSAH(objects, leftshapes, rightshapes);
                break;
        }
        
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    if (!node->bounds.IntersectP(ray))
        return inter;
    if (node->left == nullptr && node->right == nullptr) {  // 叶子节点
        return node->object->getIntersection(ray);
    }
    
    Intersection interLeft = getIntersection(node->left, ray);
        
    Intersection interRight = getIntersection(node->right, ray);

    return interLeft.distance < interRight.distance? interLeft: interRight;
}
