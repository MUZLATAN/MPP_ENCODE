//
// Created by z on 2020/7/17.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>


// #include <opencv2/core/core.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>

#include "MppEncoder.h"
using namespace std;

// int main1(){
//     whale::vision::MppEncoder mppenc;
//     mppenc.MppEncdoerInit(1920, 1080, 30);

//     cout<<"main test 1"<<endl;
//     cv::VideoCapture cap;
//     cap.open("rtsp://admin:Buzhongyao123@192.168.14.175:554/h264/ch30/main/av_stream");
//     cv::Mat rgbImg;
//     cv::Mat yuvImg;
//     cv::Mat resize_img;
//     int count = 0;

//     int length = 0;

//     cout<<"main test 4"<<endl;
//     char dst[1024*1024*4];
//     char *pdst = dst;
//     FILE* fp = fopen("I4201.h264", "wb+");

//     FILE* fy = fopen("I420.yuv", "wb+");

//     while (count++ < 500)
//     {
//         std::cout<<count++<<std::endl;
//         cap.read(rgbImg);
//         // cv::resize(rgbImg, resize_img, cv::Size(1920,1080));

//     //    mppenc.encode(resize_img, fp);

//         mppenc.encode(rgbImg, pdst, &length);
//         fwrite(dst, length,1,  fp);
//     }


//     cout<<"main test 6"<<endl;
//     fclose(fp);
//     return 0;
// }

std::vector<char> ReadFile(const std::string filename)
{
	std::vector<char> buffer;
    std::ifstream is(filename, std::ios::binary | std::ios::ate);
	if (!is.is_open())
		return buffer; 
    is.unsetf(std::ios::skipws);

    std::streampos size;
    is.seekg(0, std::ios::end);
    size = is.tellg();
    is.seekg(0, std::ios::beg);

    
    buffer.reserve(size);

    buffer.insert(buffer.begin(), std::istream_iterator<char>(is), std::istream_iterator<char>());


    return buffer;
}
int main(int argc, char* argv[]){
    std::cout<<"hello"<<std::endl;
    whale::vision::MppEncoder mppenc;
    mppenc.MppEncdoerInit(640, 360, 30);


    int count = 0;
    int length = 0;

    cout<<"main test 4"<<endl;
    char dst[1024*1024*4];
    char img[1024*1024*4];
    char *pdst = dst;
    FILE* fp = fopen("I4201.h264", "wb+");

    std::vector<char> file = ReadFile("1.yuv");
    // FILE* yuv = fopen("/home/nvidia/embed/mppencode/build/1.yuv", "rb");
    // if (yuv != NULL )
    // {
    //     fread(img,1, 345600, yuv );
    // }
    // else{
    //     std::cout<<"read file failed"<<std::endl;
    //     return 0;
    // }
    

    while (count++ < 500)
    {
        std::cout<<count++<<std::endl;

        mppenc.encode(file.data(), file.size(), pdst, &length);
        // mppenc.encode(img, 345600, pdst, &length);
        fwrite(dst, length,1,  fp);
    }

    cout<<"main test 6"<<endl;
    fclose(fp);
    return 0;
}
