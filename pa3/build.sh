c++ /opt/homebrew/Cellar/opencv/4.5.1_3/lib/libopencv_core.dylib \
    /opt/homebrew/Cellar/opencv/4.5.1_3/lib/libopencv_imgproc.dylib \
    /opt/homebrew/Cellar/opencv/4.5.1_3/lib/libopencv_imgcodecs.dylib \
    /opt/homebrew/Cellar/opencv/4.5.1_3/lib//libopencv_highgui.dylib \
    -I/opt/homebrew/Cellar/eigen/3.3.9/include \
    -I/usr/local/include  \
    -I/opt/homebrew/Cellar/opencv/4.5.1_3/include/opencv4 \
    -std=c++17 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h