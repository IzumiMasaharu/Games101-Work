#include <fstream>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include "Scene.hpp"
#include "Renderer.hpp"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// TODO MISSION
void applyAntiAliasing(std::vector<Vector3f>& buffer, int width, int height) {
    std::vector<Vector3f> newBuffer = buffer;
    
    // 应用5x5均值滤波
    for (int y = 2; y < height - 2; ++y) {
        for (int x = 2; x < width - 2; ++x) {
            int index = y * width + x;
            Vector3f sum(0.0f);
            int count = 0;
            
            // 5x5 kernel
            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    int neighborIndex = (y + dy) * width + (x + dx);
                    if (neighborIndex >= 0 && neighborIndex < width * height) {
                        sum += buffer[neighborIndex];
                        ++count;
                    }
                }
            }
            
            // 计算平均值
            newBuffer[index] = sum / count;
        }
    }
    
    buffer = newBuffer;
    std::cout << "Anti-aliasing filter (5x5) applied to current render pass.\n";
}
// TODO MISSION
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int spp = 128; // Samples per pixel
    
    // 添加多次渲染的参数
    int num_renders = 16; // 渲染次数
    std::cout << "SPP per render: " << spp << "\n";
    std::cout << "Number of renders: " << num_renders << "\n";
    std::cout << "Total effective SPP: " << spp * num_renders << "\n";
    
    // 初始化累积缓冲区，用于存储所有渲染结果的总和
    std::vector<Vector3f> accumBuffer(scene.width * scene.height, Vector3f(0.0f));
    
    for (int render_idx = 0; render_idx < num_renders; render_idx++) {
        std::cout << "Rendering pass " << (render_idx + 1) << " of " << num_renders << "...\n";
        
        // 每次渲染清空当前帧缓冲区
        std::fill(framebuffer.begin(), framebuffer.end(), Vector3f(0.0f));
        
        int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        std::mutex mutex;

        auto render_chunk = [&](int start, int end) {
            for (int j = start; j < end; ++j) {
                for (int i = 0; i < scene.width; ++i) {
                    float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
                    float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
                    Vector3f dir = normalize(Vector3f(-x, y, 1));
                    Vector3f pixel_color(0.0f);
                    
                    for (int k = 0; k < spp; k++) {
                        pixel_color += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                    }
                    
                    std::lock_guard<std::mutex> lock(mutex);
                    framebuffer[j * scene.width + i] = pixel_color;
                }
            }
        };

        int chunk_size = scene.height / num_threads;
        for (int t = 0; t < num_threads; ++t) {
            int start = t * chunk_size;
            int end = (t == num_threads - 1) ? scene.height : start + chunk_size;
            threads.emplace_back(render_chunk, start, end);
        }

        for (auto& thread : threads) {
            thread.join();
        }
        
        // 应用反锯齿滤波处理
        std::vector<Vector3f> beforeBuffer = framebuffer;
        applyAntiAliasing(framebuffer, scene.width, scene.height);
        
        // 将当前渲染结果添加到累积缓冲区
        for (size_t i = 0; i < framebuffer.size(); ++i) 
            accumBuffer[i] += framebuffer[i];
        
        // 保存每次的中间渲染结果
        if (num_renders > 1) 
        {
            std::string filename = "./Diffuse-WithAfter/render_pass_" + std::to_string(render_idx + 1) + ".ppm";
            FILE* fp = fopen(filename.c_str(), "wb");
            (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
            for (auto i = 0; i < scene.height * scene.width; ++i) {
                static unsigned char color[3];
                color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
                color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
                color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
                fwrite(color, 1, 3, fp);
            }
            fclose(fp);

            filename = "./Diffuse-WithAfter/render_pass_before_" + std::to_string(render_idx + 1) + ".ppm";
            fp = fopen(filename.c_str(), "wb");
            (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
            for (auto i = 0; i < scene.height * scene.width; ++i) {
                static unsigned char color[3];
                color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, beforeBuffer[i].x), 0.6f));
                color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, beforeBuffer[i].y), 0.6f));
                color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, beforeBuffer[i].z), 0.6f));
                fwrite(color, 1, 3, fp);
            }
            fclose(fp);
        }
    }
    
    // 计算最终的平均值
    for (size_t i = 0; i < accumBuffer.size(); ++i) 
        accumBuffer[i] = accumBuffer[i] / num_renders;

    // 保存最终的平均帧缓冲区到文件
    FILE* fp = fopen("./Diffuse-WithAfter/binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, accumBuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, accumBuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, accumBuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}