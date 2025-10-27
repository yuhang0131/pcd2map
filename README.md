# pcd2map

ROS2 工作空间

## 构建工作空间

```bash
# 进入工作空间根目录
cd /home/yuhang/Code/pcd2map

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建工作空间
colcon build

# 源化环境
source install/setup.bash
```

## 项目结构

```
pcd2map/
├── src/              # ROS2 包源代码目录
├── build/            # 构建目录（自动生成）
├── install/          # 安装目录（自动生成）
└── log/              # 日志目录（自动生成）
```
