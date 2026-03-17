# 故障排除

本文档汇总常见问题及解决方案。

## 硬件问题

### 串口连接失败

**症状**: `无法打开串口 COMx`

**排查步骤**:

1. 确认串口号正确
   ```bash
   # Windows - 设备管理器
   # Linux
   ls /dev/ttyUSB*
   ```

2. 检查驱动
   - CH340: 下载 [官方驱动](http://www.wch.cn/download/CH341SER_EXE.html)
   - CP2102: 下载 [Silicon Labs 驱动](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

3. 检查占用
   ```bash
   # Windows - 检查端口占用
   netstat -ano | findstr COM3

   # Linux - 检查端口占用
   lsof /dev/ttyUSB0
   ```

4. 权限问题 (Linux)
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # 或添加用户到 dialout 组
   sudo usermod -a -G dialout $USER
   ```

### 找不到电机

**症状**: `电机无响应` 或 `读取位置失败`

**排查步骤**:

1. 检查电源
   - 确认 12V 电源已接通
   - 检查电源指示灯

2. 检查波特率
   - SO100 默认波特率: 1000000
   - 确保代码中波特率一致

3. 检查接线
   - TX/RX 是否交叉连接
   - 是否有松动

4. 单独测试
   ```python
   # 测试单个电机
   python script/1.3test_motors.py
   ```

### 摄像头问题

**症状**: 摄像头黑屏或打不开

**排查步骤**:

1. 尝试不同索引
   ```python
   # 测试不同摄像头索引
   for i in range(5):
       cap = cv2.VideoCapture(i)
       if cap.isOpened():
           print(f"摄像头 {i} 可用")
   ```

2. 检查占用
   - 关闭其他摄像头应用
   - 重启电脑

3. 检查权限 (Linux)
   ```bash
   sudo chmod 666 /dev/video0
   ```

## 软件问题

### 模块导入错误

**症状**: `ModuleNotFoundError: No module named 'cv2'`

**解决方案**:

1. 确认环境已激活
   ```bash
   conda activate claw_arm
   # 或
   venv\Scripts\activate
   ```

2. 重新安装依赖
   ```bash
   pip install -r requirements.txt
   ```

### 依赖冲突

**症状**: 版本冲突警告

**解决方案**:

1. 使用虚拟环境隔离
2. 指定版本安装
   ```bash
   pip install opencv-python==4.8.0
   ```

### 程序崩溃

**症状**: 程序意外退出

**排查步骤**:

1. 查看错误日志
2. 添加异常处理
   ```python
   try:
       # 可能出错的代码
   except Exception as e:
       print(f"错误: {e}")
       import traceback
       traceback.print_exc()
   ```

## 运动问题

### 运动抖动

**可能原因**:
- 电源电压不稳定
- 运动速度过快
- 机械连接松动

**解决方案**:
1. 使用稳定电源
2. 降低运动速度
3. 检查机械结构

### 位置不准确

**可能原因**:
- 标定不准确
- 机械间隙
- 零点偏移

**解决方案**:
1. 重新标定相机
2. 校准零点
3. 添加位置补偿

### 抓取失败

**可能原因**:
- 物体检测不准确
- 坐标转换误差
- 夹爪力度不够

**解决方案**:
1. 调整 HSV 阈值
2. 重新标定
3. 调整夹爪参数

## 性能问题

### 帧率低

**解决方案**:
1. 降低分辨率
2. 减少处理操作
3. 使用更快的摄像头

### 响应慢

**解决方案**:
1. 优化代码
2. 减少延迟
3. 使用多线程

## 获取帮助

1. 查看 [项目 Issues](https://github.com/your-username/claw_arm/issues)
2. 提交新 Issue（附上错误日志和环境信息）
3. 加入社区讨论

### Issue 模板

```markdown
**环境信息**:
- OS: Windows 11
- Python: 3.10
- OpenCV: 4.8.0

**问题描述**:
[描述遇到的问题]

**复现步骤**:
1. 运行 xxx 命令
2. 出现 xxx 错误

**日志输出**:
```
[粘贴错误日志]
```

**已尝试的解决方案**:
- [列出已尝试的方法]
```
