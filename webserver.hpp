#ifndef WEBSERVER_HPP
#define WEBSERVER_HPP

#include <mutex> 
#include <atomic>
#include "thermo_img.hpp"
#include <opencv2/core/core.hpp>
#include <thread>

class Webserver
{
private:
    std::mutex lock;
    cv::Mat img;
    cv::Mat laplacian_img = {cv::Size(1, 1), CV_8U, 255};
    cv::Mat hs_img = {cv::Size(1, 1), CV_8U, 255};
    std::vector<poi> POI;
    std::vector<poi> heat_sources;
    std::vector<std::pair<std::string,double>> cameraComponentTemps;

public:
    std::atomic<bool> finished{ false };

    Webserver();
    void terminate();
    
    void setImg(cv::Mat img)
    {   
        std::lock_guard<std::mutex> lk(lock);
        this->img = img;
    }   
    
    void setHSImg(cv::Mat img)
    {
        std::lock_guard<std::mutex> lk(lock);
        this->hs_img = img;
    }

    void setLaplacian(cv::Mat img)
    {
        std::lock_guard<std::mutex> lk(lock);
        this->laplacian_img = img;
    }

    void setPOI(std::vector<poi> POI)
    {   
        std::lock_guard<std::mutex> lk(lock);
        this->POI = POI;
    }   

    void setHeatSources(std::vector<poi> hs)
    {
        std::lock_guard<std::mutex> lk(lock);
        this->heat_sources = hs;
    }
    
    void setCameraComponentTemps(std::vector<std::pair<std::string,double>> cct)
    {
        std::lock_guard<std::mutex> lk(lock);
        this->cameraComponentTemps = cct;
    }

private:
    std::thread web_thread;

    void start();
};

#endif
