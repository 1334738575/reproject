# reproject
这只是一个非常简单的cmake架构，主要实现了图片的虚拟视点转换，并进行填补空洞的功能。 
为了方便，代码中将rgb图像转换为灰度图，相机、帧、虚拟视点间的联系不强，只是对两张图片进行了简单变换。
一共有三类：camera、frame、V_viewpoint。 
头文件中Save_point为二维图像位置对应的点信息。
终端运行示例：
./output ../test_img/color-cam0-f000.jpg ../test_img/depth-cam0-f000.png ../test_img/color-cam1-f000.jpg ../test_img/depth-cam1-f000.png
