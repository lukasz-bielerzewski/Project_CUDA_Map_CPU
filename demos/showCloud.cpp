#include "Defs/defs.h"
#include <tinyxml2.h>
#include "Visualizer/Qvisualizer.h"
#include "Mapping/elevationMap.h"
// simulation
#include "Defs/simulator_defs.h"
//planner
#include "include/Defs/planner_defs.h"
// Utilities
#include "Utilities/recorder.h"
// Filtering
#include <GL/glut.h>
#include <qapplication.h>
#include <iostream>
//Projekt
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <pcl/outofcore/visualization/camera.h>

#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <string>


void computeSpherePoints(const cv::Mat& depthImage, const cv::Mat& rgbImage,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::mutex &cloudMutex,
                              int startRow, int endRow, int startCol, int endCol)
{
        // Compute sphere points for the given range of pixels
        for (int y = startRow; y < endRow; y+=10) {
            for (int x = startCol; x < endCol; x+=10) {
                cv::Vec3b color = rgbImage.at<cv::Vec3b>(y, x);

                // Add sphere point to the point cloud
                pcl::PointXYZRGB spherePoint;
                spherePoint.x = static_cast<float>(x);
                spherePoint.y = static_cast<float>(y);
                spherePoint.z = static_cast<float>(depthImage.at<ushort>(y, x)) / 50.0f;
                spherePoint.r = color[2];
                spherePoint.g = color[1];
                spherePoint.b = color[0];

                // Add the sphere point to the point cloud (make sure to use thread-safe operations)
                {
                    std::lock_guard<std::mutex> lock(cloudMutex);
                    cloud->push_back(spherePoint);
                }
            }
        }
}

void addSpheresToViewer(pcl::visualization::PCLVisualizer& viewer, std::mutex &viewerMutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        float radius, size_t startIdx, size_t endIdx)
{
    for (size_t i = startIdx; i < endIdx; ++i) {
        std::lock_guard<std::mutex> lock(viewerMutex);
        pcl::PointXYZRGB& spherePoint = cloud->at(i);
        std::string sphereId = "sphere_" + std::to_string(i);
        viewer.addSphere(spherePoint, radius, spherePoint.r / 255.0, spherePoint.g / 255.0, spherePoint.b / 255.0, sphereId);
    }
}


int main(int argc, char** argv)
{
    try {
        setlocale(LC_NUMERIC,"C");
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load config file.\n";
            return 0;
        }


        setlocale(LC_NUMERIC,"C");
        glutInit(&argc, argv);



        std::string rgbImagePath = "/home/rezzec/office/rgb/1.png";
        std::string depthImagePath = "/home/rezzec/office/depth/1.png";


        // Wczytanie obrazu RGB
        cv::Mat rgbImage = cv::imread(rgbImagePath);
        if (rgbImage.empty()) {
            std::cout << "Błąd wczytywania obrazu RGB.\n";
            return 1;
        }

        // Wczytanie obrazu Depth
        cv::Mat depthImage = cv::imread(depthImagePath, cv::IMREAD_ANYDEPTH);
        if (depthImage.empty()) {
            std::cout << "Błąd wczytywania obrazu Depth.\n";
            return 1;
        }

        // Utworzenie chmury punktów
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::mutex cloudMutex;
        uint numThreads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;

        std::chrono::time_point<std::chrono::system_clock> start0, end0, start1, end1, start2, end2;
        float radius = 1.0f;

        // Divide the work across multiple threads
        uint rowsPerThread = static_cast<uint>(rgbImage.rows) / numThreads;
        uint remainingRows = static_cast<uint>(rgbImage.rows) % numThreads;
        uint startRow = 0;
        uint endRow = 0;

        start0 = std::chrono::system_clock::now();
        start1 = std::chrono::system_clock::now();

        for (uint i = 0; i < numThreads; i++) {
            startRow = endRow;
            endRow = startRow + rowsPerThread;

            // Adjust the end row for the last thread to account for the remaining rows
            if (i == numThreads - 1) {
                endRow += remainingRows;
            }

            // Launch a thread to compute sphere points for the given range of pixels
            threads.emplace_back(computeSpherePoints, std::ref(depthImage), std::ref(rgbImage),
                                 cloud, std::ref(cloudMutex),
                                 startRow, endRow, 0, rgbImage.cols);
        }

        // Wait for all threads to finish
        for (auto& thread : threads) {
            if(thread.joinable())
            {
                thread.join();
            }
        }

        end1 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time1 = end1 - start1;
        std::string tTime1 = std::to_string(elapsed_time1.count());

        // Inicjalizacja wizualizatora PCL
        pcl::visualization::PCLVisualizer viewer("Wyświetlanie sfer");
        viewer.setBackgroundColor(0, 0, 0);
        viewer.addCoordinateSystem(10.0);
        viewer.initCameraParameters();

        // Modify camera parameters for better view
        viewer.setCameraPosition(
            rgbImage.size().width - 0.000472546, 0.00897074, -2.5,
            rgbImage.size().width + 0.00101357, 0.000506464, -0.000268899,
            0.0, -1.0, 0.0
        );

        viewer.addText("Points generation time: " + tTime1, 10, 15, 15, 1, 1, 1, "tTime1");

        // Wyświetlanie chmury punktów
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

        std::vector<std::thread> threads2;
        size_t numSpheres = cloud->size();
        size_t spheresPerThread = numSpheres / numThreads;
        size_t remainingSpheres = numSpheres % numThreads;
        size_t startIdx = 0;
        size_t endIdx = 0;
        std::mutex viewerMutex;

        start2 = std::chrono::system_clock::now();

        for (uint i = 0; i < numThreads; i++) {
            startIdx = endIdx;
            endIdx = startIdx + spheresPerThread;

            // Adjust the end index for the last thread to account for the remaining spheres
            if (i == numThreads - 1) {
                endIdx += remainingSpheres;
            }

            // Launch a thread to add spheres to the viewer
            threads2.emplace_back(addSpheresToViewer, std::ref(viewer), std::ref(viewerMutex), cloud, radius, startIdx, endIdx);
        }

        // Wait for all threads to finish
        for (auto& thread : threads2) {
            if(thread.joinable())
            {
                thread.join();
            }
        }

        end2 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time2 = end2 - start2;
        std::string tTime2 = std::to_string(elapsed_time2.count());
        viewer.addText("Spheres generation time: " + tTime2, 10, 35, 15, 1, 1, 1, "tTime2");

        end0 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time0 = end0 - start0;
        std::string tTime0 = std::to_string(elapsed_time0.count());
        viewer.addText("Computing runtime: " + tTime0, 10, 55, 15, 1, 1, 1, "tTime0");

        // Uruchomienie pętli wyświetlającej
        while (!viewer.wasStopped()) {
            viewer.spinOnce();
        }


    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
