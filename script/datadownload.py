# 训练数据下载，适配于Lebot的倒水数据集
from datasets import load_dataset

ds = load_dataset("RoboCOIN/R1_Lite_pour_water")
# https://huggingface.co/datasets/RoboCOIN/R1_Lite_pour_water

print(ds.shape)