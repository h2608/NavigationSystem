# 导航系统

基于 C++ 和 Qt5 的高性能导航与路径规划系统，支持大规模图数据处理、动态交通仿真和可视化。

## 功能特性

- **大规模图生成**：支持生成 10,000+ 节点的道路网络
- **多种路径算法**：Dijkstra 最短路径、动态交通感知路径规划
- **实时交通仿真**：车辆代理仿真，动态拥堵计算
- **空间索引**：四叉树加速 k-最近邻查询
- **交互式可视化**：基于 Qt Graphics View 的地图渲染，支持缩放、平移、LOD
- **图文件 I/O**：保存和加载地图数据（JSON 格式）

## 系统要求

- C++17 或更高版本
- Qt 5.14 或更高版本
- CMake 3.16 或更高版本

## 构建说明

当前仓库在 Windows 上**已验证可用**的构建方式是：

- Qt：`E:/Qt/5.14.2/mingw73_64`
- 编译器：`E:/Qt/Tools/mingw730_64/bin/g++.exe`
- 生成器：`MinGW Makefiles`

不要将 `E:/Qt/5.14.2/mingw73_64` 和 `Visual Studio / MSVC` 混用，否则会出现 Qt 链接错误。

如果之前已经用别的工具链配置过 `build/`，先删除旧的 `build/` 目录，再重新配置。

### Bash

```bash
rm -rf build

cmake -S . -B build -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER="E:/Qt/Tools/mingw730_64/bin/gcc.exe" \
  -DCMAKE_CXX_COMPILER="E:/Qt/Tools/mingw730_64/bin/g++.exe" \
  -DCMAKE_MAKE_PROGRAM="E:/Qt/Tools/mingw730_64/bin/mingw32-make.exe" \
  -DQt5_DIR="E:/Qt/5.14.2/mingw73_64/lib/cmake/Qt5"

cmake --build build -j4
```

### PowerShell

```powershell
Remove-Item -Recurse -Force .\build -ErrorAction SilentlyContinue

cmake -S . -B build -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_C_COMPILER="E:/Qt/Tools/mingw730_64/bin/gcc.exe" `
  -DCMAKE_CXX_COMPILER="E:/Qt/Tools/mingw730_64/bin/g++.exe" `
  -DCMAKE_MAKE_PROGRAM="E:/Qt/Tools/mingw730_64/bin/mingw32-make.exe" `
  -DQt5_DIR="E:/Qt/5.14.2/mingw73_64/lib/cmake/Qt5"

cmake --build build -j4
```

## 运行

### Bash

```bash
./build/src/NavigationSystem.exe
```

### PowerShell

```powershell
.\build\src\NavigationSystem.exe
```

## 项目结构

```
src/
├── core/           # 核心算法和数据结构
│   ├── algorithms/ # 路径查找算法
│   ├── generation/ # 地图生成
│   ├── graph/      # 图数据结构
│   ├── io/         # 文件 I/O
│   ├── spatial/    # 空间索引
│   └── traffic/    # 交通仿真
├── gui/            # 图形界面
│   ├── items/      # 图形项（节点、边）
│   ├── view/       # 场景和视图
│   └── widgets/    # 控件
└── app/            # 应用程序入口
```

## 使用说明

1. **生成地图**：文件 → 生成地图，设置节点数量和地图尺寸
2. **查找路径**：点击两个节点查看最短路径
3. **交通仿真**：仿真 → 开始仿真，查看实时交通状态
4. **空间查询**：使用控制面板进行 k-最近邻查询或区域交通查询
5. **保存/加载**：文件 → 保存/打开地图

## 许可证

MIT License
