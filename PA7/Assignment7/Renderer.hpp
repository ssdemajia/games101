//
// Created by goksu on 2/25/20.
//
#include <unordered_map>
#include <deque>
#include "Scene.hpp"
#include "Semaphore.hpp"

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    Renderer();
    void Render(const Scene& scene);
    void PathTrace(int id);
    void doPathTrace(int i, int j, int spp);
private:
    float imageAspectRatio;
    float scale;
    int spp;
    Vector3f eye_pos;
    const Scene* scene;

    std::vector<std::pair<int, int>> mJobs;
    std::mutex mJobsMutex;
    std::vector<int> mThreadRound;
    std::vector<std::thread> mAllThreads;

    int mFramebufferIndex;
    std::mutex mFramebufferIndexMutex;
    std::vector<std::vector<Vector3f>> mFramebuffer;
    std::mutex mFrameBufferMutex;
    int mThreadNums;
    Semaphore mThreadSema;
};
