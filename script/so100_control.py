#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂底层控制（综合版）
直接使用 scservo_sdk 控制，包含电机检测、交互式控制、示教录制回放、校准等模式
"""

from scservo_sdk import PortHandler, PacketHandler
import time
import threading
import json
import os

# ───────── ANSI 颜色 ─────────
CLR  = "\033[0m"
BOLD = "\033[1m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
DIM    = "\033[2m"

def ok(msg):   return f"{GREEN}[OK]{CLR} {msg}"
def warn(msg): return f"{YELLOW}[!!]{CLR} {msg}"
def err(msg):  return f"{RED}[ERR]{CLR} {msg}"
def info(msg): return f"{CYAN}[>>]{CLR} {msg}"

BOX_W = 60

def box_title(title: str):
    pad = BOX_W - 4 - len(title)
    left = pad // 2
    right = pad - left
    print(f"\n{BOLD}╔{'═' * (BOX_W - 2)}╗")
    print(f"║{' ' * left} {title} {' ' * right}║")
    print(f"╚{'═' * (BOX_W - 2)}╝{CLR}")

def box_menu(items: list[tuple[str, str]]):
    """打印菜单项列表 [(key, desc), ...]"""
    for key, desc in items:
        print(f"  {CYAN}{key}{CLR}  {desc}")

def bar(pos, lo=0, hi=4095, width=20):
    """返回一个 ASCII 进度条"""
    ratio = max(0.0, min(1.0, (pos - lo) / max(hi - lo, 1)))
    filled = int(ratio * width)
    return f"[{'█' * filled}{'░' * (width - filled)}]"

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def pause(msg="按 Enter 继续..."):
    input(f"\n{DIM}{msg}{CLR}")


class SO100Controller:
    """SO100 机械臂控制器"""

    # STS3215 寄存器地址
    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56
    MAX_POSITION = 4095

    # 电机名称 & 缩写
    MOTOR_NAMES = {
        1: "shoulder_pan",
        2: "shoulder_lift",
        3: "elbow_flex",
        4: "wrist_flex",
        5: "wrist_roll",
        6: "gripper"
    }
    MOTOR_SHORT = {
        1: "肩-旋转",
        2: "肩-抬升",
        3: "肘-弯曲",
        4: "腕-弯曲",
        5: "腕-旋转",
        6: "夹  爪"
    }

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None

    def connect(self):
        """连接到机械臂"""
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)
        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")
        print(ok(f"已连接到 {self.port}"))

    def disconnect(self):
        """断开连接"""
        if self.port_handler:
            for motor_id in range(1, 7):
                self.disable_torque(motor_id)
            self.port_handler.closePort()
            print(ok("已断开连接"))

    def enable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.TORQUE_ENABLE, 0)

    def enable_all(self):
        for m in range(1, 7):
            self.enable_torque(m)

    def disable_all(self):
        for m in range(1, 7):
            self.disable_torque(m)

    def set_position(self, motor_id: int, position: int):
        position = max(0, min(self.MAX_POSITION, position))
        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, self.GOAL_POSITION, position)

    def get_position(self, motor_id: int) -> int:
        data, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.PRESENT_POSITION)
        return data if result == 0 else 0

    def get_all_positions(self) -> dict[int, int]:
        return {m: self.get_position(m) for m in range(1, 7)}

    def set_all_positions(self, positions: dict[int, int]):
        for m, p in positions.items():
            self.set_position(m, p)

    def move_smooth(self, target: dict[int, int], steps=30, dt=0.02):
        """平滑移动到目标位置"""
        start = self.get_all_positions()
        for i in range(1, steps + 1):
            ratio = i / steps
            for m in target:
                pos = int(start.get(m, 2047) + (target[m] - start.get(m, 2047)) * ratio)
                self.set_position(m, pos)
            time.sleep(dt)

    def ping(self, motor_id: int) -> bool:
        _, result, _ = self.packet_handler.ping(self.port_handler, motor_id)
        return result == 0

    def check_motors(self) -> list[int]:
        """检测并返回在线电机列表"""
        found = [m for m in range(1, 7) if self.ping(m)]
        return found


def mode_detection(controller: SO100Controller):
    """模式1：电机检测"""
    box_title("电机检测")

    print("\n  扫描电机中...\n")
    found = []
    for m in range(1, 7):
        name = controller.MOTOR_SHORT[m]
        eng  = controller.MOTOR_NAMES[m]
        if controller.ping(m):
            found.append(m)
            pos = controller.get_position(m)
            print(f"  {GREEN}●{CLR}  电机 {m}  {name:<8} ({eng:<14})  位置: {pos:>4}  {bar(pos)}")
        else:
            print(f"  {RED}○{CLR}  电机 {m}  {name:<8} ({eng:<14})  {DIM}--- 离线 ---{CLR}")

    total = len(found)
    print(f"\n  结果: {GREEN if total == 6 else YELLOW}{total}/6{CLR} 个电机在线", end="")
    if total == 6:
        print(f"  {GREEN}✓ 全部正常{CLR}")
    elif total > 0:
        print(f"  {YELLOW}⚠ 部分缺失{CLR}")
    else:
        print(f"  {RED}✗ 全部离线{CLR}")
    pause()


def mode_control_test(controller: SO100Controller):
    """模式2：预设动作测试"""
    box_title("预设动作测试")

    found = controller.check_motors()
    if len(found) < 6:
        print(err(f"只检测到 {len(found)}/6 个电机，无法测试"))
        pause(); return

    print(ok("6 个电机全部就绪"))

    controller.enable_all()
    print(ok("扭矩已启用"))

    sequences = [
        ("零点位置",   {1: 2047, 2: 2047, 3: 2047, 4: 2047, 5: 2047, 6: 2000}),
        ("伸展位置",   {1: 2500, 2: 2500, 3: 1500, 4: 1500, 5: 2047, 6: 3000}),
        ("收缩位置",   {1: 1500, 2: 1500, 3: 2500, 4: 2500, 5: 2500, 6: 1000}),
        ("回到零点",   {1: 2047, 2: 2047, 3: 2047, 4: 2047, 5: 2047, 6: 2047}),
    ]

    for i, (name, positions) in enumerate(sequences, 1):
        print(f"\n  {CYAN}[{i}/{len(sequences)}]{CLR} 移动到 → {BOLD}{name}{CLR}")
        controller.move_smooth(positions)
        # 显示到达位置
        for m in range(1, 7):
            pos = controller.get_position(m)
            print(f"    {controller.MOTOR_SHORT[m]}  {pos:>4}  {bar(pos)}")
        time.sleep(0.5)

    print(f"\n{ok('预设动作测试完成！')}")
    pause()


# ──────────────────────────────────────────
# 模式3：交互式关节控制（新增）
# ──────────────────────────────────────────

def mode_interactive(controller: SO100Controller):
    """模式3：交互式关节控制"""
    box_title("交互式关节控制")

    found = controller.check_motors()
    if len(found) < 6:
        print(err(f"只检测到 {len(found)}/6 个电机")); pause(); return

    controller.enable_all()
    print(ok("扭矩已启用\n"))

    STEP = 50  # 默认步长
    saved_poses = {}

    def show_help():
        print(f"""
  {BOLD}命令说明{CLR}
  ─────────────────────────────────────
  {CYAN}<电机ID> <位置>{CLR}     直接设置位置 (0-4095)
                        例: {DIM}1 2047{CLR}  (电机1 → 2047)
  {CYAN}<电机ID> +/-{CLR}        微调 ±{STEP}
                        例: {DIM}6 +{CLR}  (夹爪打开一点)
  {CYAN}all <位置>{CLR}          所有电机移到同一位置
  {CYAN}home{CLR}               回到中间位置 (2047)
  {CYAN}pos{CLR}                显示所有电机当前位置
  {CYAN}save <名称>{CLR}        保存当前姿态
  {CYAN}load <名称>{CLR}        加载已保存的姿态
  {CYAN}list{CLR}               列出已保存的姿态
  {CYAN}step <值>{CLR}          修改微调步长 (当前: {STEP})
  {CYAN}free{CLR}               释放扭矩 (自由拖动)
  {CYAN}lock{CLR}               启用扭矩 (锁定)
  {CYAN}h{CLR}                  显示帮助
  {CYAN}q{CLR}                  返回主菜单
""")

    def show_positions():
        print()
        for m in range(1, 7):
            pos = controller.get_position(m)
            print(f"  {CYAN}{m}{CLR}  {controller.MOTOR_SHORT[m]}  {pos:>4}  {bar(pos)}")
        print()

    show_help()
    show_positions()

    while True:
        try:
            raw = input(f"  {CYAN}arm>{CLR} ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        # ── 退出 ──
        if cmd == 'q':
            break

        # ── 帮助 ──
        elif cmd == 'h':
            show_help()

        # ── 显示位置 ──
        elif cmd == 'pos':
            show_positions()

        # ── 回零 ──
        elif cmd == 'home':
            print(info("平滑回零..."))
            controller.move_smooth({m: 2047 for m in range(1, 7)})
            show_positions()

        # ── 全部移动 ──
        elif cmd == 'all' and len(parts) == 2:
            try:
                val = int(parts[1])
                controller.move_smooth({m: val for m in range(1, 7)})
                show_positions()
            except ValueError:
                print(warn("请输入有效数字"))

        # ── 释放/锁定 ──
        elif cmd == 'free':
            controller.disable_all()
            print(ok("扭矩已释放，可以自由拖动机械臂"))

        elif cmd == 'lock':
            controller.enable_all()
            print(ok("扭矩已启用，机械臂已锁定"))

        # ── 修改步长 ──
        elif cmd == 'step' and len(parts) == 2:
            try:
                STEP = int(parts[1])
                print(ok(f"步长已设为 {STEP}"))
            except ValueError:
                print(warn("请输入有效数字"))

        # ── 保存姿态 ──
        elif cmd == 'save' and len(parts) >= 2:
            name = parts[1]
            saved_poses[name] = controller.get_all_positions()
            print(ok(f"姿态 \"{name}\" 已保存"))

        # ── 加载姿态 ──
        elif cmd == 'load' and len(parts) >= 2:
            name = parts[1]
            if name in saved_poses:
                controller.enable_all()
                print(info(f"加载姿态 \"{name}\"..."))
                controller.move_smooth(saved_poses[name])
                show_positions()
            else:
                print(warn(f"姿态 \"{name}\" 不存在"))

        # ── 列出姿态 ──
        elif cmd == 'list':
            if saved_poses:
                print(f"\n  {BOLD}已保存的姿态:{CLR}")
                for name, pose in saved_poses.items():
                    vals = " ".join(f"{v:>4}" for v in pose.values())
                    print(f"    {CYAN}{name:<12}{CLR} [{vals}]")
                print()
            else:
                print(warn("暂无保存的姿态"))

        # ── 单关节控制 ──
        elif cmd.isdigit():
            motor_id = int(cmd)
            if motor_id < 1 or motor_id > 6:
                print(warn("电机 ID 范围: 1-6"))
                continue
            if len(parts) == 1:
                pos = controller.get_position(motor_id)
                print(f"  {controller.MOTOR_SHORT[motor_id]}  {pos:>4}  {bar(pos)}")
            elif parts[1] == '+':
                cur = controller.get_position(motor_id)
                controller.set_position(motor_id, cur + STEP)
                time.sleep(0.3)
                pos = controller.get_position(motor_id)
                print(f"  {controller.MOTOR_SHORT[motor_id]}  {pos:>4}  {bar(pos)}")
            elif parts[1] == '-':
                cur = controller.get_position(motor_id)
                controller.set_position(motor_id, cur - STEP)
                time.sleep(0.3)
                pos = controller.get_position(motor_id)
                print(f"  {controller.MOTOR_SHORT[motor_id]}  {pos:>4}  {bar(pos)}")
            else:
                try:
                    val = int(parts[1])
                    controller.set_position(motor_id, val)
                    time.sleep(0.3)
                    pos = controller.get_position(motor_id)
                    print(f"  {controller.MOTOR_SHORT[motor_id]}  {pos:>4}  {bar(pos)}")
                except ValueError:
                    print(warn("请输入有效数字或 +/-"))

        else:
            print(warn(f"未知命令: {raw}  (输入 h 查看帮助)"))

    controller.enable_all()
    print()


# ──────────────────────────────────────────
# 模式4：示教录制与回放（新增）
# ──────────────────────────────────────────

def mode_teach(controller: SO100Controller):
    """模式4：示教录制与回放"""
    box_title("示教录制与回放")

    found = controller.check_motors()
    if len(found) < 6:
        print(err(f"只检测到 {len(found)}/6 个电机")); pause(); return

    waypoints = []
    FILE = "so100_teach.json"

    while True:
        print(f"""
  {BOLD}示教菜单{CLR}
  ─────────────────────────
  {CYAN}1{CLR}  开始录制 (释放扭矩 → 手动摆姿 → 按 Enter 记录)
  {CYAN}2{CLR}  回放轨迹
  {CYAN}3{CLR}  查看已录制的路点
  {CYAN}4{CLR}  清空路点
  {CYAN}5{CLR}  保存轨迹到文件
  {CYAN}6{CLR}  从文件加载轨迹
  {CYAN}q{CLR}  返回主菜单
""")
        choice = input(f"  {CYAN}teach>{CLR} ").strip()

        if choice == 'q':
            controller.enable_all()
            break

        elif choice == '1':
            # 录制
            controller.disable_all()
            print(ok("扭矩已释放, 请手动移动机械臂"))
            print(info("每按一次 Enter 记录一个路点, 输入 done 结束录制\n"))

            idx = len(waypoints)
            while True:
                cmd = input(f"  {DIM}[路点 {idx + 1}]{CLR} 按 Enter 记录 / 输入 done 结束: ").strip()
                if cmd.lower() == 'done':
                    break
                pos = controller.get_all_positions()
                waypoints.append(pos)
                idx += 1
                vals = " ".join(f"{v:>4}" for v in pos.values())
                print(f"  {GREEN}✓{CLR} 已记录路点 {idx}: [{vals}]")

            print(f"\n{ok(f'录制结束, 共 {len(waypoints)} 个路点')}")

        elif choice == '2':
            if not waypoints:
                print(warn("没有路点, 请先录制")); continue

            controller.enable_all()
            print(ok("扭矩已启用"))

            try:
                repeat = int(input(f"  回放次数 (默认 1): ").strip() or "1")
            except ValueError:
                repeat = 1

            try:
                speed = float(input(f"  回放速度倍率 (默认 1.0): ").strip() or "1.0")
            except ValueError:
                speed = 1.0

            steps = max(10, int(30 / speed))

            print(info(f"开始回放, 共 {len(waypoints)} 个路点 × {repeat} 次\n"))
            for r in range(repeat):
                if repeat > 1:
                    print(f"  {DIM}── 第 {r + 1}/{repeat} 轮 ──{CLR}")
                for i, wp in enumerate(waypoints):
                    print(f"  → 路点 {i + 1}/{len(waypoints)}", end="\r")
                    controller.move_smooth(wp, steps=steps)
                    time.sleep(0.1)
                print()

            print(ok("回放完成"))

        elif choice == '3':
            if not waypoints:
                print(warn("暂无路点")); continue
            print(f"\n  {BOLD}路点列表 (共 {len(waypoints)} 个){CLR}")
            for i, wp in enumerate(waypoints, 1):
                vals = " ".join(f"{v:>4}" for v in wp.values())
                print(f"    {DIM}{i:>3}.{CLR} [{vals}]")
            print()

        elif choice == '4':
            waypoints.clear()
            print(ok("路点已清空"))

        elif choice == '5':
            if not waypoints:
                print(warn("没有路点可保存")); continue
            serializable = [dict(wp) for wp in waypoints]
            with open(FILE, 'w', encoding='utf-8') as f:
                json.dump(serializable, f, indent=2)
            print(ok(f"轨迹已保存到 {FILE} ({len(waypoints)} 个路点)"))

        elif choice == '6':
            if not os.path.exists(FILE):
                print(warn(f"未找到 {FILE}")); continue
            with open(FILE, 'r', encoding='utf-8') as f:
                loaded = json.load(f)
            waypoints = [{int(k): v for k, v in wp.items()} for wp in loaded]
            print(ok(f"已加载 {len(waypoints)} 个路点"))


def mode_calibration(controller: SO100Controller):
    """模式5：校准"""
    box_title("校准")

    found = controller.check_motors()
    if len(found) < 6:
        print(err(f"只检测到 {len(found)}/6 个电机")); pause(); return

    print(ok("6 个电机全部就绪"))

    # 步骤 1：禁用扭矩
    print(f"\n  {BOLD}[步骤 1]{CLR} 禁用扭矩")
    controller.disable_all()
    print(ok("可以手动移动机械臂"))

    # 步骤 2：移动到中间位置
    print(f"\n  {BOLD}[步骤 2]{CLR} 将机械臂移动到中间位置")
    print(f"  {DIM}提示: 大臂约45度, 小臂约90度, 手腕水平{CLR}")
    pause("准备好后按 Enter...")

    # 记录零点
    print(f"\n  {BOLD}[步骤 3]{CLR} 记录零点位置")
    zero_positions = {}
    for m in range(1, 7):
        pos = controller.get_position(m)
        zero_positions[m] = pos
        print(f"    {controller.MOTOR_SHORT[m]}  {pos:>4}  {bar(pos)}")

    # 步骤 3：连续记录运动范围
    print(f"\n  {BOLD}[步骤 4]{CLR} 记录运动范围")
    print(f"  {DIM}请逐个移动所有关节（除了 wrist_roll）通过它们的完整运动范围{CLR}")
    pause("准备好后按 Enter 开始记录...")

    # 初始化范围
    ranges = {motor_id: [4095, 0] for motor_id in range(1, 7)}
    ranges[5] = [0, 4095]  # wrist_roll 全范围

    # 记录状态
    recording = True
    stop_event = threading.Event()

    def record_loop():
        """记录循环"""
        nonlocal recording
        while recording:
            for motor_id in range(1, 7):
                pos = controller.get_position(motor_id)
                if pos > 0:
                    # 更新最小/最大值
                    if pos < ranges[motor_id][0]:
                        ranges[motor_id][0] = pos
                    if pos > ranges[motor_id][1]:
                        ranges[motor_id][1] = pos
            time.sleep(0.1)  # 10 Hz 采样

    def display_loop():
        """显示循环"""
        nonlocal recording
        while recording:
            clear_screen()
            print(f"\n{BOLD}  ● 实时记录中...{CLR}  (按 Enter 停止)\n")
            print(f"  {'电机':<10} {'最小':>6} {'当前':>6} {'最大':>6} {'范围':>6}  可视化")
            print(f"  {'─' * 62}")

            for motor_id in range(1, 7):
                pos = controller.get_position(motor_id)
                mn = ranges[motor_id][0]
                mx = ranges[motor_id][1]
                rng = mx - mn
                print(f"  {controller.MOTOR_SHORT[motor_id]:<10} {mn:>6} {CYAN}{pos:>6}{CLR} {mx:>6} {rng:>6}  {bar(pos)}")

            print(f"\n  {DIM}提示: 逐个移动每个关节通过完整运动范围{CLR}")
            time.sleep(0.2)

    # 启动记录和显示线程
    record_thread = threading.Thread(target=record_loop)
    display_thread = threading.Thread(target=display_loop)

    record_thread.start()
    display_thread.start()

    # 等待用户停止
    input()

    # 停止记录
    recording = False
    record_thread.join()
    display_thread.join()

    # 显示结果
    box_title("校准完成")

    print(f"\n  {BOLD}零点位置:{CLR}")
    for motor_id, pos in zero_positions.items():
        print(f"    {controller.MOTOR_SHORT[motor_id]}  {pos:>4}")

    print(f"\n  {BOLD}运动范围:{CLR}")
    for motor_id in range(1, 7):
        min_pos, max_pos = ranges[motor_id]
        rng = max_pos - min_pos
        print(f"    {controller.MOTOR_SHORT[motor_id]}  {min_pos:>4} ~ {max_pos:>4}  (范围 {rng})")

    # 保存校准数据
    filename = "so100_calibration.txt"
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("# SO100 机械臂校准数据\n")
        f.write("# 生成时间: " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n\n")
        f.write("# 零点位置\n")
        for motor_id, pos in zero_positions.items():
            f.write(f"zero_{controller.MOTOR_NAMES[motor_id]} = {pos}\n")

        f.write("\n# 运动范围\n")
        for motor_id, (min_p, max_p) in ranges.items():
            f.write(f"range_{controller.MOTOR_NAMES[motor_id]} = {min_p}, {max_p}\n")

    print(ok(f"校准数据已保存到: {filename}"))

    # 启用扭矩
    controller.enable_all()
    print(ok("扭矩已启用"))
    pause()


# ──────────────────────────────────────────
# 模式5：实时位置监控（新增）
# ──────────────────────────────────────────

def mode_monitor(controller: SO100Controller):
    """模式5：实时位置监控"""
    box_title("实时位置监控")

    found = controller.check_motors()
    if not found:
        print(err("未检测到任何电机")); pause(); return

    print(ok(f"检测到 {len(found)}/{6} 个电机"))
    print(info("按 Ctrl+C 返回主菜单\n"))

    try:
        while True:
            clear_screen()
            print(f"\n{BOLD}  ◉ 实时位置监控{CLR}  {DIM}(Ctrl+C 退出){CLR}\n")
            print(f"  {'ID':<4} {'名称':<10} {'位置':>6}  {'可视化'}")
            print(f"  {'─' * 50}")
            for m in found:
                pos = controller.get_position(m)
                print(f"  {CYAN}{m:<4}{CLR} {controller.MOTOR_SHORT[m]:<10} {pos:>6}  {bar(pos)}")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print(f"\n{ok('已退出监控')}")
        pause()


def mode_freedrag(controller: SO100Controller):
    """模式7：自由拖动"""
    box_title("自由拖动")

    found = controller.check_motors()
    if not found:
        print(err("未检测到任何电机")); pause(); return

    print(ok(f"检测到 {len(found)}/6 个电机"))

    # 释放所有扭矩
    controller.disable_all()
    print(ok("扭矩已释放, 现在可以自由拖动各个关节"))
    print(info("按 Ctrl+C 退出并重新启用扭矩\n"))

    try:
        while True:
            clear_screen()
            print(f"\n{BOLD}  ✋ 自由拖动模式{CLR}  {DIM}(Ctrl+C 退出){CLR}")
            print(f"  {DIM}扭矩已释放 — 请用手移动机械臂各关节{CLR}\n")
            print(f"  {'ID':<4} {'名称':<10} {'位置':>6}  {'可视化'}")
            print(f"  {'─' * 50}")
            for m in found:
                pos = controller.get_position(m)
                print(f"  {CYAN}{m:<4}{CLR} {controller.MOTOR_SHORT[m]:<10} {pos:>6}  {bar(pos)}")
            print(f"\n  {DIM}提示: 您可以同时拖动多个关节, 位置会实时更新{CLR}")
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass

    # 退出时重新启用扭矩
    controller.enable_all()
    print(f"\n{ok('扭矩已重新启用')}")
    pause()


def main():
    """主函数"""
    PORT = "COM7"

    # 在 Windows 上启用 ANSI 颜色
    if os.name == 'nt':
        os.system('')

    controller = SO100Controller(PORT)

    try:
        controller.connect()
    except Exception as e:
        print(err(str(e)))
        return

    try:
        while True:
            clear_screen()
            box_title("SO100 机械臂控制台")
            print()
            box_menu([
                ("1", "电机检测         扫描并显示所有电机状态"),
                ("2", "预设动作测试     执行一组预定义运动序列"),
                ("3", "交互式关节控制   命令行实时操控每个关节"),
                ("4", "示教录制与回放   手动拖动录制轨迹并回放"),
                ("5", "校准             记录零点和运动范围"),
                ("6", "实时位置监控     持续刷新显示关节位置"),
                ("7", "自由拖动         释放扭矩并实时显示位置"),
                ("q", "退出"),
            ])
            print()

            choice = input(f"  {CYAN}请选择>{CLR} ").strip().lower()

            if choice == '1':
                mode_detection(controller)
            elif choice == '2':
                mode_control_test(controller)
            elif choice == '3':
                mode_interactive(controller)
            elif choice == '4':
                mode_teach(controller)
            elif choice == '5':
                mode_calibration(controller)
            elif choice == '6':
                mode_monitor(controller)
            elif choice == '7':
                mode_freedrag(controller)
            elif choice == 'q':
                break
            else:
                print(warn("无效选项，请重新选择"))
                time.sleep(0.5)

    except KeyboardInterrupt:
        pass

    finally:
        controller.disconnect()
        print(f"\n{DIM}再见！{CLR}\n")


if __name__ == "__main__":
    main()
