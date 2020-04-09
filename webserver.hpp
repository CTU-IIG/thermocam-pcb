#ifndef WEBSERVER_HPP
#define WEBSERVER_HPP

#include <mutex> 
#include <atomic>
#include "thermocam-pcb.hpp"
#include <opencv2/core/core.hpp>

class Webserver
{
private:
    std::mutex lock;
    cv::Mat img;
    std::vector<poi> POI;

public:
    std::atomic<bool> finished{ false };
    
    void setImg(cv::Mat img)
    {   
        std::lock_guard<std::mutex> lk(lock);
        this->img = img;
    }   
    
    void setPOI(std::vector<poi> POI)
    {   
        std::lock_guard<std::mutex> lk(lock);
        this->POI = POI;
    }   
    
    void *start(void*);
};

#endif
