# Introduce some components on rviz2

marker.type:

```bash
visualization_msgs::msg::Marker::ARROW        // 箭头
visualization_msgs::msg::Marker::CUBE         // 立方体
visualization_msgs::msg::Marker::SPHERE       // 球体
visualization_msgs::msg::Marker::CYLINDER     // 圆柱体
visualization_msgs::msg::Marker::LINE_STRIP   // 线段（连接一系列点的折线）
visualization_msgs::msg::Marker::LINE_LIST    // 线列表（点对之间的线段）
visualization_msgs::msg::Marker::SPHERE_LIST  // 球体列表（多个球体）
visualization_msgs::msg::Marker::CUBE_LIST    // 立方体列表（多个立方体）
visualization_msgs::msg::Marker::POINTS       // 点集（多个点）
visualization_msgs::msg::Marker::TEXT_VIEW_FACING  // 面向相机的文本
```

marker.action:

```bash
visualization_msgs::msg::Marker::ADD        // 添加或修改标记
visualization_msgs::msg::Marker::DELETE     // 删除标记
visualization_msgs::msg::Marker::DELETEALL  // 删除所有标记
```

ColorRGBA 结构,这个消息类型包含四个浮点数（float32），每个字段对应颜色的一个分量：

```bash
float32 r   // 红色 (Red)
float32 g   // 绿色 (Green)
float32 b   // 蓝色 (Blue)
float32 a   // 透明度 (Alpha)
```

```bash
std_msgs::msg::ColorRGBA color;
color.r = 0.0;   // 没有红色
color.g = 0.0;   // 没有绿色
color.b = 1.0;   // 纯蓝色
color.a = 0.5;   // 半透明
```

```bash
visualization_msgs::msg::Marker marker;
marker.color.r = 0.0;
marker.color.g = 1.0;   // 绿色
marker.color.b = 0.0;
marker.color.a = 1.0;   // 完全不透明
```