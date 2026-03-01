import argparse
import sys
from pathlib import Path
from huggingface_hub import login
from huggingface_hub import HfApi, snapshot_download
from huggingface_hub.errors import GatedRepoError, HfHubHTTPError, RepositoryNotFoundError


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="下载 Hugging Face 数据集（默认 RoboCOIN/R1_Lite_pour_water）"
	)
	parser.add_argument(
		"--repo-id",
		default="RoboCOIN/R1_Lite_pour_water",
		help="数据集仓库 ID，默认 RoboCOIN/R1_Lite_pour_water",
	)
	parser.add_argument(
		"--local-dir",
		default="./data/R1_Lite_pour_water",
		help="本地保存目录",
	)
	parser.add_argument(
		"--meta-only",
		action="store_true",
		help="仅下载 meta 和 annotations（不下载视频）",
	)
	parser.add_argument(
		"--revision",
		default="main",
		help="仓库分支/commit/tag，默认 main",
	)
	return parser.parse_args()


def check_access(repo_id: str, revision: str) -> None:
	api = HfApi()
	try:
		api.repo_info(repo_id=repo_id, repo_type="dataset", revision=revision)
	except GatedRepoError:
		print("访问被限制：请先在网页同意数据集条款并分享联系信息。")
		print(f"页面：https://huggingface.co/datasets/{repo_id}")
		print("完成后执行：hf auth login")
		sys.exit(2)
	except RepositoryNotFoundError:
		print(f"仓库不存在或无权限：{repo_id}")
		sys.exit(2)
	except HfHubHTTPError as exc:
		print(f"访问仓库失败：{exc}")
		print("请确认已登录：hf auth login")
		sys.exit(2)


def build_patterns(meta_only: bool):
	if not meta_only:
		return None, None
	allow_patterns = [
		"meta/**",
		"annotations/**",
		"README.md",
	]
	return allow_patterns, None


def download_dataset(repo_id: str, local_dir: str, revision: str, meta_only: bool) -> Path:
	allow_patterns, ignore_patterns = build_patterns(meta_only)
	local_path = snapshot_download(
		repo_id=repo_id,
		repo_type="dataset",
		local_dir=local_dir,
		revision=revision,
		allow_patterns=allow_patterns,
		ignore_patterns=ignore_patterns,
		local_dir_use_symlinks=False,
		resume_download=True,
	)
	return Path(local_path).resolve()


def print_summary(path: Path, meta_only: bool) -> None:
	print("\n下载完成。")
	print(f"本地目录：{path}")
	key_dirs = ["meta", "annotations", "data", "videos"]
	exists = [name for name in key_dirs if (path / name).exists()]
	print(f"包含目录：{', '.join(exists) if exists else '无'}")
	if meta_only:
		print("当前为仅元数据模式（未下载 data/videos）。")


def main() -> None:
	args = parse_args()
	check_access(args.repo_id, args.revision)
	out_path = download_dataset(
		repo_id=args.repo_id,
		local_dir=args.local_dir,
		revision=args.revision,
		meta_only=args.meta_only,
	)
	print_summary(out_path, args.meta_only)


if __name__ == "__main__":
    main()
