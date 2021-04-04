#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float radian = rotation_angle * MY_PI / 180;
    model << std::cos(radian), -std::sin(radian), 0, 0,
             std::sin(radian), std::cos(radian), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float radian = angle * MY_PI / 180;
    float x = axis.x();
    float y = axis.y();
    float z = axis.z();
    float cos_theta = std::cos(radian);
    float sin_theta = std::sin(radian);
    Eigen::Matrix4f result;
    result << x*x*(1-cos_theta)+cos_theta, x*y*(1-cos_theta)-z*cos_theta, x*z*(1-cos_theta)+y*sin_theta, 0,
              x*y*(1-cos_theta)+z*sin_theta, y*y*(1-cos_theta)+cos_theta, y*z*(1-cos_theta)-x*sin_theta, 0,
              x*z*(1-cos_theta)-y*sin_theta, y*z*(1-cos_theta)+x*sin_theta, z*z*(1-cos_theta)+cos_theta, 0,
              0, 0, 0, 1;
    return result;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float radianFovHalf = eye_fov * MY_PI / 360;
    float t = std::abs(zNear) * std::tan(radianFovHalf);
    float r = t * aspect_ratio;
    projection << zNear/r, 0, 0, 0,
                  0, zNear/t, 0, 0,
                  0, 0, (zNear+zFar)/(zNear-zFar), -2*zNear*zFar/(zNear-zFar),
                  0, 0, 1, 0;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }
    int eye_z = 10;
    rst::rasterizer r(700, 700);

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        Eigen::Vector3f eye_pos = {0, 0, eye_z};
        r.set_model(get_rotation(Eigen::Vector3f(0, 1, 0), angle));
        // r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'w') {
            eye_z -= 1;
        }
        else if (key == 's') {
            eye_z += 1;
        }
    }

    return 0;
}
