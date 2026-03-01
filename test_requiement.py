import torch
from lerobot.common.policies import make_policy
 
# 尝试创建一个最简单的策略实例，不加载具体权重，只测试环境
try:
    # 创建一个虚拟的Diffusion策略配置
    policy = make_policy("diffusion", policy_kwargs={"obs_dim": 10, "action_dim": 2}, device="cpu")
    print("✅ LeRobot核心策略模块导入成功！")
    print(f"创建的策略类型：{type(policy)}")
except Exception as e:
    print(f"❌ 导入失败，错误信息：{e}")
