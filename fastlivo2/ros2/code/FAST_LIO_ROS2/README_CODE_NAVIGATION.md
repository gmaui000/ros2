# FAST_LIO_ROS2 代码阅读配置指南

## 概述
此配置专门为代码阅读和导航而设计，不涉及实际编译。配置了clangd作为主要的C++语言服务器，提供优秀的代码跳转功能。

## 已配置的功能

### 1. 代码跳转支持
- **定义跳转**: Ctrl+点击函数/类名跳转到定义
- **引用查找**: 右键点击函数/变量 → "Find All References"
- **悬停信息**: 鼠标悬停在代码上显示类型信息
- **自动补全**: 智能代码补全建议

### 2. 禁用功能
- 自动CMake配置
- 自动构建
- C/C++插件的IntelliSense（避免与clangd冲突）

### 3. 文件排除
- build/ 目录
- install/ 目录  
- log/ 目录

## 使用说明

### 重启VSCode
配置完成后，请重启VSCode以确保所有设置生效。

### 验证clangd状态
1. 打开任意C++文件（如 `src/preprocess.cpp`）
2. 查看VSCode右下角状态栏，应该显示"clangd"状态
3. 如果显示"⚠️"，点击查看详细错误信息

### 基本操作
1. **跳转到定义**: Ctrl+点击函数/类名
2. **查找引用**: 右键 → "Find All References"
3. **查看类型**: 鼠标悬停在变量/函数上
4. **代码补全**: 输入时按Ctrl+Space

### 常见问题解决

#### clangd无法启动
如果clangd无法启动，检查：
1. clangd是否已安装: `clangd --version`
2. VSCode的clangd扩展是否启用

#### 头文件找不到错误
由于在宿主机上缺少ROS2等依赖，外部头文件会显示错误，但这不影响项目内部代码的跳转功能。

#### 性能问题
如果clangd响应慢，可以：
1. 关闭大型文件
2. 重启clangd服务器（VSCode命令面板 → "clangd: Restart Language Server"）

## 配置详情

### VSCode设置 (.vscode/settings.json)
- 禁用CMake自动配置和构建
- 启用clangd作为主要C++语言服务器
- 配置clangd参数优化代码分析
- 排除构建相关目录的文件监视

### 编译数据库 (build/compile_commands.json)
包含项目的基本编译信息，帮助clangd理解项目结构。

## 扩展推荐
已配置的扩展：
- vscode-clangd (主要代码分析)
- C/C++ (备用)
- CMake Tools (禁用自动功能)

## 注意事项
1. 此配置仅用于代码阅读，不适用于编译
2. 外部依赖（如ROS2、PCL等）的头文件会显示错误，但项目内部代码跳转正常
3. 如果需要完整编译，请在虚拟机环境中进行
