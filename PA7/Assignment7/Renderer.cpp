//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <chrono>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

Renderer::Renderer()
{
    const auto processor_count = std::thread::hardware_concurrency();
    mThreadNums = processor_count > 1? processor_count-1: 1;
    mThreadRound = std::vector<int>(mThreadNums, 0);
    std::cout << "thread nums:" << mThreadNums << std::endl;
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    mFramebuffer = std::vector<std::vector<Vector3f>>(scene.width, std::vector<Vector3f>(scene.height));

    std::vector<std::thread> curThreads;
    scale = tan(deg2rad(scene.fov * 0.5));
    imageAspectRatio = scene.width / (float)scene.height;
    eye_pos = Vector3f(278, 273, -800);
    mFramebufferIndex = 0;
    this->scene = &scene;
    
    mJobs.resize(scene.width * scene.height);
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            mJobs.push_back(std::make_pair(i, j));
            // doPathTrace(i, j, 1);
        }
    }
   std::cout << "job nums:" << mJobs.size() << std::endl;
   for (int i = 0; i < mThreadNums; i++)
   {
       mAllThreads.emplace_back(std::bind(&Renderer::PathTrace, this, std::placeholders::_1), i);
   }
   for (auto& t : mAllThreads)
   {
       t.join();
   }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            static unsigned char color[3];
            color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, mFramebuffer[i][j].x), 0.6f));
            color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, mFramebuffer[i][j].y), 0.6f));
            color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, mFramebuffer[i][j].z), 0.6f));
            fwrite(color, 1, 3, fp);
        }
    }

    fclose(fp);    
}

void Renderer::PathTrace(int id)
{
    int spp = 17;
    std::cout << id << " start" << std::endl;
    // change the spp value to change sample ammount
    std::cout << "SPP: " << spp << "\n";
    while (true)
    {
        int index = id + mThreadNums * mThreadRound[id];
        
        mThreadRound[id] += 1;
        if (index >= mJobs.size())
        {
            return;
        }
            
        auto& job = mJobs[index];
        int i = job.first;
        int j = job.second;
        doPathTrace(i, j, spp);
    }
}

void Renderer::doPathTrace(int i, int j, int spp)
{
    for (int k = 0; k < spp; k++){
        // generate primary ray direction
        float x = (2 * (i + 0.5) / (float)scene->width - 1) *
                imageAspectRatio * scale;
        float y = (1 - 2 * (j + 0.5) / (float)scene->height) * scale;

        Vector3f dir = normalize(Vector3f(-x, y, 1));
        mFramebuffer[i][j] += scene->castRay(Ray(eye_pos, dir), 0) / spp;
    }
   {
       std::lock_guard<std::mutex> g(mFramebufferIndexMutex);
       mFramebufferIndex++;
       UpdateProgress(mFramebufferIndex / (float)mJobs.size());
   }
}
