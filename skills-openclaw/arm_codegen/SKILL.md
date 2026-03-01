---
name: arm_codegen
description: 引导 OpenClaw 根据用户自然语言生成基于 so-100 的机械臂控制代码。当用户用提示词描述机械臂动作且现有脚本无法直接满足时，根据本技能提供的 SDK 与示例自动组织并生成可执行的 Python 脚本，然后直接运行。
metadata:
  {
    "openclaw":
      {
        "requires": { "bins": ["python3", "pip3"] },
      },
  }
---

## 功能概览

- 本技能用于**根据用户自然语言描述**，引导 OpenClaw **生成**可执行的 so-100 机械臂控制代码（Python 脚本），而不是仅调用现成 CLI。
- 参考 SDK：

## 何时使用本技能

- 用户说「写一段代码控制机械臂」「根据我的描述生成控制脚本」「让机械臂按顺序做多个动作」等。
- 用户明确要求「生成 Python 代码」或「给我可运行的脚本」来控制 Nero/Piper 等 AgileX 机械臂。

## 使用本技能生成代码并直接运行
   - 根据用户提示词，结合本技能的 `references/SCServo_SDK使用文档.md` 中的 API 与模板，生成一段完整、可运行的 Python 脚本。然后直接运行

## 生成代码时的规则

1. **连接与配置**

6. **实现细节**
   - When waiting for motion to complete, use shorter timeout (2-3 seconds)
   - After each mechanical arm operation, add a small sleep (0.01 seconds)
   - Motion completion detection: `robot.get_arm_status().msg.motion_status == 0` (not == 1)

## 参考文件

- **API 与最小可运行模板**：`references/SCServo_SDK使用文档.md`  
  生成代码时请结合该文件中的接口说明与代码片段，保证与 SCServo SDK 及 test1.py 用法一致。
