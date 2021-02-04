#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <iostream>

#include <thread>       // std::thread
#include <future>       // std::future

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class cvSharedMemory{
   public:
    cvSharedMemory(){}

    void open(int rows, int cols, int channels, int elementsize, int cvtype,
              std::string map_file = "opencv_unity_buffer",std::string mutex_file = "opencv_unity_mutex"){
        this->map_file_name = std::wstring(map_file.begin(), map_file.end());
        this->mutex_file_name = std::wstring(mutex_file.begin(), mutex_file.end());
        this->rows = rows;
        this->cols = cols;
        this->channels = channels;
        this->elementsize = elementsize;
        this->cvtype = cvtype;
        initMemory(rows * cols * elementsize);
        ready = true;
    }

    void close(){
        ::UnmapViewOfFile(data_buf);
        ::CloseHandle(shmem);
        ::ReleaseMutex(mutex);
        ready = false;
        future_init_ = false;
    }

    bool write_threaded(cv::Mat image){
        if (!isSendThreadBusy()){
            future_ = std::async(&cvSharedMemory::write, this, image);
            future_init_ = true;
            return true;
        } else {
            std::cerr << "Shared memory thread is busy" << std::endl;
            return false;
        }
    }

    void write(cv::Mat image){
        if (ready){
            int img_data_size = image.total() * image.elemSize();
            if (img_data_size == (rows * cols * elementsize)){
                /* Copy to Shared memory */
                WaitForSingleObject(mutex, INFINITE);
                memcpy(data_buf, image.ptr(), (rows * cols * elementsize));
                ::ReleaseMutex(mutex);
            } else {
                std::cerr << "Invalid image size for shared memory. Re-initalise share memory for new image size." << std::endl;
            }
        } else {
            std::cerr << "Shared memory not initalised. Run 'open' before calling 'write'" << std::endl;
        }
    }

    void read(cv::Mat &image){
        if (ready){
            WaitForSingleObject(mutex, INFINITE);
            image = cv::Mat(cv::Size(cols, rows), cvtype, data_buf, channels * cols);
            ::ReleaseMutex(mutex);
        } else {
            std::cerr << "Shared memory not initalised. Run 'open' before calling 'read'" << std::endl;
        }
    }

    bool isOpen(){
        return ready;
    }

    bool isSendThreadBusy(){
        if (future_init_){
            using namespace std::chrono_literals;
            auto status = future_.wait_for(0ms);
            bool send_complete = (status == std::future_status::ready);
            return !send_complete;
        } else {
            return false;
        }
    }

   private:
    HANDLE mutex = INVALID_HANDLE_VALUE;
    HANDLE shmem = INVALID_HANDLE_VALUE;
    int rows,cols,channels,cvtype,elementsize;
    unsigned char *data_buf;
    std::wstring map_file_name;
    std::wstring mutex_file_name;
    bool ready = false;

    std::future<void> future_;
    bool future_init_ = false;

    void initMemory(int mem_size){
        /* Create Shared Memory */
        mutex = ::CreateMutex(NULL, FALSE, mutex_file_name.c_str());
        shmem = ::CreateFileMapping(
            INVALID_HANDLE_VALUE,
            NULL,
            PAGE_READWRITE,
            0,
            mem_size,
            map_file_name.c_str()
        );
        data_buf = (unsigned char*)::MapViewOfFile(shmem, FILE_MAP_ALL_ACCESS, 0, 0, (rows * cols * elementsize));
    }
};
