#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <opencv4/opencv2/opencv.hpp>
#include <crp_sdk_interfaces/srv/convert_pcd_to_costmap.hpp>
#include <fstream>
#include <string>
#include <filesystem>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

class PointCloudToCostmapService : public rclcpp::Node
{
public:
    PointCloudToCostmapService() : Node("pointcloud_to_costmap_service")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 初始化点云转Costmap服务...");
        
        // 创建服务
        convert_service_ = this->create_service<crp_sdk_interfaces::srv::ConvertPcdToCostmap>(
            "convert_pcd_to_costmap",
            std::bind(&PointCloudToCostmapService::convert_callback, this, _1, _2));
        
        // 默认处理参数
        declare_parameter<double>("default_resolution", 0.05);
        declare_parameter<int>("default_padding", 10);
        declare_parameter<double>("default_min_height", -5.0);
        declare_parameter<double>("default_max_height", 2.0);
        declare_parameter<double>("normal_radius", 0.3);
        declare_parameter<double>("wall_normal_threshold", 0.3);
        declare_parameter<int>("statistical_k", 20);
        declare_parameter<double>("statistical_stddev", 1.0);
        declare_parameter<double>("radius_outlier_radius", 0.2);
        declare_parameter<int>("radius_outlier_min_neighbors", 10);
        declare_parameter<bool>("enable_height_filter", true);
        declare_parameter<bool>("enable_statistical_filter", true);
        declare_parameter<bool>("enable_radius_filter", true);
        declare_parameter<bool>("enable_normal_filter", true);
        declare_parameter<bool>("save_intermediate_clouds", false);
        declare_parameter<bool>("verbose_logging", true);
        
        RCLCPP_INFO(this->get_logger(), "✅ 点云转Costmap服务已启动");
        RCLCPP_INFO(this->get_logger(), "📞 等待服务调用... 使用以下命令测试:");
        RCLCPP_INFO(this->get_logger(), "   ros2 service call /convert_pcd_to_costmap crp_sdk_interfaces/srv/ConvertPcdToCostmap \"{input_pcd_path: '/path/to/map.pcd', output_directory: '/tmp/costmap', map_name: 'my_map'}\"");
    }

private:
    rclcpp::Service<crp_sdk_interfaces::srv::ConvertPcdToCostmap>::SharedPtr convert_service_;

    void convert_callback(
        const std::shared_ptr<crp_sdk_interfaces::srv::ConvertPcdToCostmap::Request> request,
        std::shared_ptr<crp_sdk_interfaces::srv::ConvertPcdToCostmap::Response> response)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "==== 收到点云转换请求 ====");
        RCLCPP_INFO(this->get_logger(), "📁 输入PCD文件: %s", request->input_pcd_path.c_str());
        RCLCPP_INFO(this->get_logger(), "📁 输出目录: %s", request->output_directory.c_str());
        
        try {
            bool success = process_pointcloud(request, response);
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            response->processing_time = duration.count() / 1000.0;
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "✅ 点云转换成功完成 (耗时: %.2f秒)", response->processing_time);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 点云转换失败");
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "转换过程中发生异常: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "❌ %s", response->message.c_str());
        }
    }

    bool process_pointcloud(
        const std::shared_ptr<crp_sdk_interfaces::srv::ConvertPcdToCostmap::Request> request,
        std::shared_ptr<crp_sdk_interfaces::srv::ConvertPcdToCostmap::Response> response)
    {
        // 验证输入文件
        if (!std::filesystem::exists(request->input_pcd_path)) {
            response->success = false;
            response->message = "PCD文件不存在: " + request->input_pcd_path;
            return false;
        }
        
        // 验证必填参数
        if (request->map_name.empty()) {
            response->success = false;
            response->message = "地图名称不能为空";
            return false;
        }
        
        // 设置参数（使用请求中的参数或默认值）
        std::string map_name = request->map_name;
        double resolution = (request->resolution > 0) ? request->resolution : this->get_parameter("default_resolution").as_double();
        int padding = (request->padding > 0) ? request->padding : this->get_parameter("default_padding").as_int();
        double min_height = request->min_height;
        double max_height = request->max_height;
        
        // 如果请求中没有设置高度范围，使用默认值
        if (min_height == 0.0 && max_height == 0.0) {
            min_height = this->get_parameter("default_min_height").as_double();
            max_height = this->get_parameter("default_max_height").as_double();
        }
        
        double normal_radius = this->get_parameter("normal_radius").as_double();
        double wall_normal_threshold = this->get_parameter("wall_normal_threshold").as_double();
        int statistical_k = this->get_parameter("statistical_k").as_int();
        double statistical_stddev = this->get_parameter("statistical_stddev").as_double();
        double radius_outlier_radius = this->get_parameter("radius_outlier_radius").as_double();
        int radius_outlier_min_neighbors = this->get_parameter("radius_outlier_min_neighbors").as_int();
        
        bool enable_height_filter = this->get_parameter("enable_height_filter").as_bool();
        bool enable_statistical_filter = this->get_parameter("enable_statistical_filter").as_bool();
        bool enable_radius_filter = this->get_parameter("enable_radius_filter").as_bool();
        bool enable_normal_filter = this->get_parameter("enable_normal_filter").as_bool();
        bool save_intermediate_clouds = this->get_parameter("save_intermediate_clouds").as_bool();
        bool verbose_logging = this->get_parameter("verbose_logging").as_bool();

        // 创建输出目录
        try {
            std::filesystem::create_directories(request->output_directory);
            if (verbose_logging) {
                RCLCPP_INFO(this->get_logger(), "📁 输出目录: %s", request->output_directory.c_str());
            }
        } catch (const std::filesystem::filesystem_error& e) {
            response->success = false;
            response->message = "无法创建输出目录: " + std::string(e.what());
            return false;
        }

        // 加载PCD文件
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(request->input_pcd_path, *cloud) < 0)
        {
            response->success = false;
            response->message = "无法加载PCD文件: " + request->input_pcd_path;
            return false;
        }

        if (verbose_logging) {
            RCLCPP_INFO(this->get_logger(), "📊 原始点云大小: %zu", cloud->size());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = cloud;

        // 1. 高度过滤 - 去除地面和天花板
        if (enable_height_filter) {
            pcl::PassThrough<pcl::PointXYZ> pass_z;
            pass_z.setInputCloud(current_cloud);
            pass_z.setFilterFieldName("z");
            pass_z.setFilterLimits(min_height, max_height);
            pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pass_z.filter(*height_filtered);
            
            if (verbose_logging) {
                RCLCPP_INFO(this->get_logger(), "After height filtering: %zu", height_filtered->size());
            }
            
            if (save_intermediate_clouds) {
                std::string height_file = request->output_directory + "/" + map_name + "_height_filtered.pcd";
                pcl::io::savePCDFileBinary(height_file, *height_filtered);
                RCLCPP_INFO(this->get_logger(), "💾 保存高度过滤点云到: %s", height_file.c_str());
            }
            
            current_cloud = height_filtered;
        }

        // 2. 统计滤波 - 去除离群点
        if (enable_statistical_filter) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(current_cloud);
            sor.setMeanK(statistical_k);
            sor.setStddevMulThresh(statistical_stddev);
            pcl::PointCloud<pcl::PointXYZ>::Ptr stat_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            sor.filter(*stat_filtered);
            
            if (verbose_logging) {
                RCLCPP_INFO(this->get_logger(), "After statistical filtering: %zu", stat_filtered->size());
            }
            
            if (save_intermediate_clouds) {
                std::string stat_file = request->output_directory + "/" + map_name + "_statistical_filtered.pcd";
                pcl::io::savePCDFileBinary(stat_file, *stat_filtered);
                RCLCPP_INFO(this->get_logger(), "💾 保存统计过滤点云到: %s", stat_file.c_str());
            }
            
            current_cloud = stat_filtered;
        }

        // 3. 半径滤波 - 去除密度太低的点
        if (enable_radius_filter) {
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
            ror.setInputCloud(current_cloud);
            ror.setRadiusSearch(radius_outlier_radius);
            ror.setMinNeighborsInRadius(radius_outlier_min_neighbors);
            pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            ror.filter(*radius_filtered);
            
            if (verbose_logging) {
                RCLCPP_INFO(this->get_logger(), "After radius filtering: %zu", radius_filtered->size());
            }
            
            if (save_intermediate_clouds) {
                std::string radius_file = request->output_directory + "/" + map_name + "_radius_filtered.pcd";
                pcl::io::savePCDFileBinary(radius_file, *radius_filtered);
                RCLCPP_INFO(this->get_logger(), "💾 保存半径过滤点云到: %s", radius_file.c_str());
            }
            
            current_cloud = radius_filtered;
        }

        // 4. 法向量估计和墙体点提取
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud = current_cloud;
        
        if (enable_normal_filter) {
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(current_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_radius);
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            ne.compute(*normals);

            // 基于法向量的墙体点提取
            pcl::PointIndices::Ptr wall_indices(new pcl::PointIndices);
            for (size_t i = 0; i < normals->size(); ++i)
            {
                const auto& normal = normals->points[i];
                // 检查法向量是否有效
                if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z))
                    continue;
                    
                float nz = std::abs(normal.normal_z);
                float nx = std::abs(normal.normal_x);
                float ny = std::abs(normal.normal_y);
                
                // 更严格的墙体判断：z分量小，x或y分量大
                if (nz < wall_normal_threshold && (nx > 0.7 || ny > 0.7))
                    wall_indices->indices.push_back(i);
            }

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(current_cloud);
            extract.setIndices(wall_indices);
            pcl::PointCloud<pcl::PointXYZ>::Ptr wall_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*wall_filtered);
            
            if (verbose_logging) {
                RCLCPP_INFO(this->get_logger(), "After wall normal filtering: %zu", wall_filtered->size());
            }
            
            if (save_intermediate_clouds) {
                std::string wall_file = request->output_directory + "/" + map_name + "_wall_filtered.pcd";
                pcl::io::savePCDFileBinary(wall_file, *wall_filtered);
                RCLCPP_INFO(this->get_logger(), "💾 保存墙体过滤点云到: %s", wall_file.c_str());
            }
            
            final_cloud = wall_filtered;
        }

        // 5. 体素滤波 - 最终降采样
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(final_cloud);
        vg.setLeafSize(resolution, resolution, resolution);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*downsampled);
        
        if (verbose_logging) {
            RCLCPP_INFO(this->get_logger(), "Final point cloud size: %zu", downsampled->size());
        }
        
        if (save_intermediate_clouds) {
            std::string final_file = request->output_directory + "/" + map_name + "_final.pcd";
            pcl::io::savePCDFileBinary(final_file, *downsampled);
            RCLCPP_INFO(this->get_logger(), "💾 保存最终点云到: %s", final_file.c_str());
        }

        // 投影 + 构建栅格地图
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;

        for (auto& pt : downsampled->points)
        {
            if (pt.x < min_x) min_x = pt.x;
            if (pt.x > max_x) max_x = pt.x;
            if (pt.y < min_y) min_y = pt.y;
            if (pt.y > max_y) max_y = pt.y;
        }

        int width = static_cast<int>((max_x - min_x) / resolution) + 2 * padding;
        int height = static_cast<int>((max_y - min_y) / resolution) + 2 * padding;

        cv::Mat map = cv::Mat::ones(height, width, CV_8UC1) * 254;

        for (auto& pt : downsampled->points)
        {
            int x = static_cast<int>((pt.x - min_x) / resolution) + padding;
            int y = static_cast<int>((pt.y - min_y) / resolution) + padding;
            if (x >= 0 && x < width && y >= 0 && y < height)
                map.at<uchar>(height - y - 1, x) = 0;
        }

        std::string pgm_file = request->output_directory + "/" + map_name + ".pgm";
        std::string yaml_file = request->output_directory + "/" + map_name + ".yaml";

        cv::imwrite(pgm_file, map);

        std::ofstream yaml_out(yaml_file);
        yaml_out << "image: " << map_name + ".pgm" << "\n";
        yaml_out << "resolution: " << resolution << "\n";
        yaml_out << "origin: [" << min_x - padding * resolution << ", " << min_y - padding * resolution << ", 0.0]\n";
        yaml_out << "negate: 0\n";
        yaml_out << "occupied_thresh: 0.1\n";
        yaml_out << "free_thresh: 0.9\n";
        yaml_out.close();

        // 设置响应信息
        response->success = true;
        response->message = "点云转换成功完成";
        response->pgm_file_path = pgm_file;
        response->yaml_file_path = yaml_file;
        response->point_count = static_cast<int32_t>(downsampled->size());

        RCLCPP_INFO(this->get_logger(), "🗺️ 地图已保存到 %s 和 %s", pgm_file.c_str(), yaml_file.c_str());
        RCLCPP_INFO(this->get_logger(), "📊 处理点数: %d", response->point_count);
        
        return true;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PointCloudToCostmapService>();
    
    RCLCPP_INFO(node->get_logger(), "🔄 点云转Costmap服务运行中，等待服务调用...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
