# NavigationSystem 架构文档

本文档关注项目的整体结构、模块关系、运行流程和阅读路径，适合新成员先建立全局认知。

## 1. 项目定位

`NavigationSystem` 是一个基于 `C++17 + Qt5 Widgets + CMake` 的桌面导航与交通仿真系统。它同时具备：

- 地图生成能力
- 路径规划能力
- 动态交通仿真能力
- 空间查询能力
- 可视化交互能力
- 地图文件保存与加载能力

项目不是单纯的算法仓库，也不是单纯的 GUI 演示，而是一个把“图模型、算法、仿真、可视化”整合在一起的完整桌面应用。

## 2. 技术栈与工程组织

### 2.1 技术栈

- 语言：`C++17`
- GUI：`Qt5 Widgets`
- 构建：`CMake 3.16+`

### 2.2 编译目标

工程按三层目标组织：

```text
NavigationSystem (exe)
    └── NavigationGui (static library)
            ├── Qt5::Widgets
            └── NavigationCore (static library)
```

这代表：

- `NavigationCore` 负责核心逻辑，不依赖 GUI
- `NavigationGui` 负责界面和交互，依赖 `NavigationCore`
- `NavigationSystem` 只是最终启动程序

## 3. 目录结构

```text
src/
├── app/                    # 应用入口
├── core/                   # 核心逻辑
│   ├── algorithms/         # 路径规划
│   ├── generation/         # 地图生成
│   ├── graph/              # 图模型
│   ├── io/                 # 文件读写
│   ├── spatial/            # 空间索引
│   └── traffic/            # 交通模型与仿真
└── gui/                    # 界面与可视化
    ├── items/              # 图元
    ├── view/               # 场景与视图
    ├── widgets/            # 控件和对话框
    └── MainWindow.*        # 总编排
```

理解项目时，不要按文件数去看，而要按职责去看：

- `app`：程序从哪里启动
- `core`：系统真正做事的地方
- `gui`：系统如何被用户操作和看到

## 4. 分层架构

项目可以抽象成下面三层：

### 4.1 入口层

文件：`src/app/main.cpp`

职责很简单：

- 创建 `QApplication`
- 创建主窗口
- 显示主窗口
- 生成初始地图

也就是说，入口层不承担业务逻辑，只负责把应用跑起来。

### 4.2 编排与展示层

核心文件：

- `src/gui/MainWindow.*`
- `src/gui/view/MapScene.*`
- `src/gui/view/MapView.*`
- `src/gui/widgets/ControlPanel.*`

这层的职责是：

- 接收用户操作
- 调用核心模块
- 把结果显示出来
- 管理界面状态

这里最重要的类是 `MainWindow`。它几乎可以视为项目的“应用控制器”。

### 4.3 核心能力层

核心目录：

- `src/core/graph/`
- `src/core/algorithms/`
- `src/core/generation/`
- `src/core/traffic/`
- `src/core/spatial/`
- `src/core/io/`

这层负责：

- 维护基础数据模型
- 生成地图
- 计算路径
- 推进交通仿真
- 执行空间查询
- 读写文件

可以把它理解成“项目真正的业务核心”。

## 5. 系统主链路

从全局上看，项目的主链路如下：

```text
用户操作 / 程序启动
        ↓
main.cpp
        ↓
MainWindow
        ├── 地图生成 / 加载
        ├── PathFinder 选择
        ├── 交通仿真调度
        ├── 四叉树重建
        └── 控制面板事件处理
        ↓
core 模块
        ├── Graph
        ├── GridPerturbationGenerator
        ├── DijkstraPathFinder / DynamicPathFinder
        ├── TrafficSimulator / TrafficModel
        ├── QuadTree / BoundingBox
        └── GraphIO
        ↓
MapScene / MapView / 图元
        ↓
界面回显
```

几乎所有业务最终都要经过 `MainWindow`，所以阅读项目时优先理解它是最划算的。

## 6. 各核心模块在架构中的位置

### 6.1 图模型是地基

`Graph` 是所有模块共同依赖的数据中心：

- 地图生成向 `Graph` 写入节点和边
- 路径规划从 `Graph` 读取拓扑和边权
- 交通仿真修改边上的车辆数
- 空间索引从图中读取节点坐标
- 文件读写负责将 `Graph` 序列化和反序列化
- GUI 通过 `Graph` 把场景画出来

如果 `Graph` 没理解，后面所有模块都会变得零散。

### 6.2 路径规划是算法抽象层

项目通过 `PathFinder` 统一封装路径算法，这让：

- GUI 不必依赖具体算法实现
- 后续新增算法时能保持接入点稳定
- “最短距离”和“最快时间”两种策略能被统一调用

### 6.3 交通仿真让地图从静态变成动态

如果没有交通模块，这个项目就是一个静态地图可视化 + 路径搜索工具。

有了 `TrafficSimulator` 和 `TrafficModel` 之后：

- 边会有车辆数
- 边权会随交通状态变化
- 热力图会有实时更新
- 动态路径才有实际意义

### 6.4 空间索引解决查询效率问题

如果每次都在全图上暴力找最近节点或范围节点，查询会很慢。`QuadTree` 的意义就在于：

- 让近邻查询更快
- 让局部交通查看更高效
- 将图展示和空间检索解耦

### 6.5 GUI 负责把“结果”变成“体验”

GUI 层不是简单画图，而是负责：

- 交互输入
- 场景状态切换
- 路径与高亮展示
- 缩放与聚焦
- 拥堵颜色可视化

其中：

- `MapScene` 更偏场景状态与图元管理
- `MapView` 更偏视口行为
- `ControlPanel` 更偏命令输入

## 7. 典型运行流程

### 7.1 程序启动流程

1. `main.cpp` 创建 `QApplication`
2. 创建 `MainWindow`
3. `MainWindow` 初始化 UI、菜单、工具栏、控制面板和信号连接
4. `main.cpp` 调用 `generateNewMap()`
5. 地图生成后，创建 `TrafficSimulator`
6. 加载场景、重建四叉树、视图缩放适配

### 7.2 生成新地图流程

1. 用户触发生成地图
2. `GenerateMapDialog` 收集参数
3. `MainWindow` 创建新的 `Graph`
4. `GridPerturbationGenerator` 写入节点和边
5. `MainWindow` 启动交通仿真
6. `MainWindow` 调用 `loadGraph()` 将结果交给场景层

### 7.3 路径查询流程

有两种入口：

- 在场景里点击两个节点
- 在控制面板里按 ID 输入起点和终点

共同点是：

- 最终都要调用 `PathFinder`
- 最终都要返回 `PathResult`
- 最终都由 `MapScene` 负责画路径

### 7.4 空间查询流程

1. 用户输入坐标和参数
2. `ControlPanel` 发信号给 `MainWindow`
3. `MainWindow` 调用 `QuadTree`
4. 根据结果收集节点和边
5. `MapScene` 执行高亮与聚焦

### 7.5 保存和加载流程

保存：

1. 用户触发保存
2. `MainWindow` 调用 `GraphIO::save()`
3. `GraphIO` 将图写入 JSON 地图文件

加载：

1. 用户触发加载
2. `MainWindow` 调用 `GraphIO::load()`
3. 恢复 `Graph` 和地图尺寸
4. 停止旧仿真并启动新仿真
5. 重新加载场景并重建四叉树

## 8. 运行时的重要特点

这些特点不是实现细节，而是读代码时必须带着的前提：

- 生成地图或加载地图后，交通仿真会自动启动
- 热力图开关只控制“是否显示颜色”，不控制仿真是否运行
- 当前仿真器中，随机车辆的路线是按最短距离生成的
- 用户查询路径时，可以切换为交通感知的动态路由
- 地图保存不会保留瞬时交通状态
- 控制面板中的 `F1 / F2 / F4` 是功能标签，不是系统级快捷键

## 9. 推荐阅读顺序

如果是第一次接触代码，推荐按下面顺序阅读：

1. `src/gui/MainWindow.h` 与 `src/gui/MainWindow.cpp`
2. `src/core/graph/Graph.h` 与 `src/core/graph/Graph.cpp`
3. `src/core/algorithms/PathFinder.h`
4. `src/core/algorithms/DijkstraPathFinder.cpp`
5. `src/core/algorithms/DynamicPathFinder.cpp`
6. `src/core/traffic/TrafficSimulator.cpp`
7. `src/gui/view/MapScene.cpp`
8. `src/core/io/GraphIO.cpp`

这个顺序的好处是：

- 先看到总编排
- 再回到基础模型
- 然后理解算法与仿真
- 最后再进入界面细节

## 10. 一句话总结

这个项目可以概括为：

“一个以 `Graph` 为基础数据模型，以 `PathFinder` 为算法扩展点，以 `MainWindow + MapScene` 为应用编排核心的 Qt 导航仿真系统。”

只要先掌握下面四件事，项目就会很清楚：

- 图模型是什么
- 算法接口是什么
- 仿真如何改变边状态
- GUI 如何把 core 结果展示出来
