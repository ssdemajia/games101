// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


std::vector<std::array<float, 2>> offsets = {
    {-0.25, -0.35},
    {0.35, 0.35},
    {-0.35, 0.28},
    {0.25, 0.25}
};
// std::vector<std::array<float, 2>> offsets = {
//     {0, 0}
// };

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const std::array<Eigen::Vector4f, 3>& v)
{   
    Eigen::Vector3f p, a, b, c;
    p << x, y, 0;
    a << v[0].x(), v[0].y(), 0;
    b << v[1].x(), v[1].y(), 0;
    c << v[2].x(), v[2].y(), 0;
    // std::cout << a.x() << ", " << a.y() << std::endl;
    // std::cout << b.x() << ", " << b.y() << std::endl;
    // std::cout << c.x() << ", " << c.y() << std::endl;
    // std::cout << std::endl;
    Eigen::Vector3f ab = b - a, ap = p - a;
    Eigen::Vector3f bc = c - b, bp = p - b;
    Eigen::Vector3f ca = a - c, cp = p - c;

    bool flag1 = ab.cross(ap).z() >= 0;
    bool flag2 = bc.cross(bp).z() >= 0;
    bool flag3 = ca.cross(cp).z() >= 0;
    return flag1 == flag2 == flag3;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0; // far - near / 2
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        // std::cout << v[0].x() << ", " << v[0].y() << ", " << v[0].z() << std::endl;
        // std::cout << v[1].x() << ", " << v[1].y() << ", " << v[1].z() << std::endl;
        // std::cout << v[2].x() << ", " << v[2].y() << ", " << v[2].z() << std::endl;
        // std::cout << std::endl;
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    std::array<Eigen::Vector4f, 3> v = t.toVector4();

    auto minX = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    auto minY = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));

    auto maxX = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    auto maxY = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    int upperX = static_cast<int>(std::ceil(maxX));
    int upperY = static_cast<int>(std::ceil(maxY));

    for (int x = static_cast<int>(minX); x < upperX; x++) {
        for (int y = static_cast<int>(minY); y < upperY; y++) {
            // msaa偏移
            for (int o = 0; o < offsets.size(); o++) {
                auto offset = offsets[o];
                if (!insideTriangle(x+offset[0], y+offset[1], v))
                {
                    continue;
                }

                auto[alpha, beta, gamma] = computeBarycentric2D(x+offset[0], y+offset[1], t.v); // 计算得到重心系数
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();  // 插值后的深度值
                if (z_interpolated > depth_buf[get_index_with_offset(x, y, o)]) {
                    continue;
                }
                Eigen::Vector3f subColor = 255 * alpha * t.color[0] + 
                                           255 * beta * t.color[1] + 
                                           255 * gamma * t.color[2];
                msaa_buf[get_index_with_offset(x, y, o)] = subColor;
                depth_buf[get_index_with_offset(x, y, o)] = z_interpolated;
            }
        }
    }
    for (int x = static_cast<int>(minX); x < upperX; x++) {
        for (int y = static_cast<int>(minY); y < upperY; y++) {
            Eigen::Vector3f color(0, 0, 0);
            // msaa偏移
            for (int o = 0; o < offsets.size(); o++) {
                color += msaa_buf[get_index_with_offset(x, y, o)];
            }
            // std::cout << color.x() << "," << color.y() << "," << color.z() << std::endl;
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, color/4.0f);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    std::fill(msaa_buf.begin(), msaa_buf.end(), Eigen::Vector3f{0, 0, 0});
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h * offsets.size());
    msaa_buf.resize(w * h * offsets.size());
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_with_offset(int x, int y, int offset)
{
    return ((height-1-y)*width + x) * offsets.size() + offset;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on