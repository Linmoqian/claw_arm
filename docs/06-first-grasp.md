# 第一次抓取

本文档将带你完成第一次自动抓取，综合运用前面学到的知识。

## 准备工作

在开始之前，确保：

- [ ] 环境已配置完成
- [ ] 机械臂已连接并测试通过
- [ ] 摄像头已连接并测试通过
- [ ] 已完成相机标定（或使用默认参数）

## 抓取流程

完整的抓取流程包含以下步骤：

```
摄像头捕获 → 物体检测 → 坐标转换 → 路径规划 → 机械臂运动 → 抓取
```

## 快速体验

使用预设程序完成一次抓取：

```bash
python examples/basic/auto_grasp.py
```

程序会自动：
1. 打开摄像头
2. 检测红色物体
3. 计算世界坐标
4. 控制机械臂移动到目标位置
5. 闭合夹爪抓取

## 分步实践

### Step 1: 确认物体位置

首先用视觉程序确认物体位置：

```bash
python CV/4_world_coordinate.py
```

记下输出的世界坐标，例如 `(0.15, 0.05, 0.30)`。

### Step 2: 手动控制测试

使用交互模式测试机械臂：

```bash
python script/3.1so100_SDK_control.py --mode interactive
```

尝试手动控制机械臂移动到目标位置。

### Step 3: 编写抓取脚本

创建一个简单的抓取脚本：

```python
"""
simple_grasp.py - 简单抓取示例
"""
import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from SDK import PortHandler, PacketHandler

# 配置
SERIAL_PORT = "COM3"  # 修改为你的串口
BAUDRATE = 1000000

def main():
    # 连接机械臂
    port = PortHandler(SERIAL_PORT)
    port.setBaudRate(BAUDRATE)
    pkt = PacketHandler(0.0)

    if not port.openPort():
        print("无法打开串口")
        return

    try:
        # TODO: 实现抓取逻辑
        # 1. 读取视觉坐标
        # 2. 转换为关节角度
        # 3. 控制机械臂移动
        # 4. 闭合夹爪
        print("抓取完成！")

    finally:
        port.closePort()

if __name__ == "__main__":
    main()
```

### Step 4: 调试与优化

常见问题排查：

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 抓取位置偏移 | 坐标转换误差 | 重新标定相机 |
| 夹爪力度不够 | 夹爪参数未调 | 调整夹爪位置范围 |
| 运动不平滑 | 速度过快 | 降低运动速度 |

## 安全注意事项

⚠️ **重要安全提示**：

1. 首次运行时，保持手在急停按钮附近
2. 确保机械臂运动范围内无障碍物
3. 不要在机械臂运动时触碰
4. 使用较低的初始速度测试

## 下一步

完成第一次抓取后，你可以：

- [机械臂控制](04-arm-control.md) - 深入学习 SDK 使用
- [相机标定](05-calibration.md) - 提高定位精度
- [二次开发](07-advanced-dev.md) - 扩展自定义功能
