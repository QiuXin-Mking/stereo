from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
from typing import List, Optional

import yaml

from .capture import capture_session
from .exporter import export_result
from .solver import solve_stereo


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "default.yaml"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        add_help=False,
        allow_abbrev=False,
        description="UVC Camera 1 的 SBS 双目棋盘格引导标定",
    )
    parser.add_argument("-h", "--help", action="store_true", help="显示帮助")
    parser.add_argument("--device-name", default="UVC Camera 1", help="AVFoundation 设备名称")
    parser.add_argument("--device-index", type=int, help="跳过名称解析，直接使用当前设备索引")
    parser.add_argument("--capture-width", type=int, help="完整 SBS 原生宽度覆盖值")
    parser.add_argument("--capture-height", type=int, help="完整 SBS 原生高度覆盖值")
    parser.add_argument("--square-mm", type=float, help="打印后实测的单个方格边长，单位 mm")
    parser.add_argument("--swap-eyes", action="store_true", help="交换 SBS 左右半幅")
    parser.add_argument("--web", action="store_true", help="启动 RK3588 无界面浏览器控制台")
    parser.add_argument("--device", default="/dev/video0", help="Linux V4L2 图像节点")
    parser.add_argument("--host", default="0.0.0.0", help="Web 监听地址")
    parser.add_argument("--port", type=int, default=8765, help="Web 监听端口")
    parser.add_argument("--target", type=int, default=32, help="目标自动接收图像对数，默认 32")
    parser.add_argument(
        "--session-root", type=Path, default=PROJECT_ROOT / "sessions", help="session 输出根目录"
    )
    parser.add_argument("--dry-run", action="store_true", help="只显示配置，不打开相机")
    return parser


def _print_help(parser: argparse.ArgumentParser) -> None:
    parser.print_help()
    print("\n默认设备: UVC Camera 1；棋盘: A4 9x6 内角点；输入打印后实测方格边长。")


def main(argv: Optional[List[str]] = None) -> int:
    parser = _parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit:
        return 2
    if args.help:
        _print_help(parser)
        return 0
    if args.square_mm is not None and args.square_mm <= 0:
        print("错误：方格边长必须大于 0 mm", file=sys.stderr)
        return 2
    if args.target < 20 or args.target > 40:
        print("错误：目标采集数量必须在 20 到 40 之间", file=sys.stderr)
        return 2
    if args.port < 1 or args.port > 65535:
        print("错误：端口必须在 1 到 65535 之间", file=sys.stderr)
        return 2

    config = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    config["device"]["name"] = args.device_name
    if args.device_index is not None:
        config["device"]["index"] = args.device_index
    if (args.capture_width is None) != (args.capture_height is None):
        print("错误：--capture-width 与 --capture-height 必须同时提供", file=sys.stderr)
        return 2
    if args.capture_width is not None:
        if args.capture_width <= 0 or args.capture_height <= 0:
            print("错误：采集分辨率必须大于 0", file=sys.stderr)
            return 2
        config["device"]["capture_width"] = args.capture_width
        config["device"]["capture_height"] = args.capture_height
    config["device"]["swap_eyes"] = bool(args.swap_eyes)
    config["capture"]["target_pairs"] = args.target

    square_mm = args.square_mm
    if square_mm is None and not args.dry_run:
        try:
            square_mm = float(input("请输入打印后实测单格边长（mm）: ").strip())
        except (ValueError, EOFError):
            print("错误：方格边长输入无效", file=sys.stderr)
            return 2
        if square_mm <= 0:
            print("错误：方格边长必须大于 0 mm", file=sys.stderr)
            return 2
    if square_mm is None:
        square_mm = 20.0

    print(
        f"设备={args.device_name}  棋盘={config['board']['columns']}x{config['board']['rows']}  "
        f"方格={square_mm:.3f} mm  目标={args.target} 对"
    )
    if args.web:
        print(f"RK3588 模式=MJPG 3840x1080@30  设备={args.device}")
        print(f"Web 地址=http://{args.host}:{args.port}")
    if args.dry_run:
        print("DRY RUN：不会打开相机，也不会创建 session。")
        return 0

    session_dir = args.session_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir.mkdir(parents=True, exist_ok=False)
    resolved = dict(config)
    resolved["measurement"] = {"square_size_mm": square_mm, "square_size_m": square_mm / 1000.0}
    (session_dir / "session.yaml").write_text(
        yaml.safe_dump(resolved, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    if args.web:
        from .headless import HeadlessCalibrationEngine
        from .web import create_server

        engine = HeadlessCalibrationEngine(
            config,
            session_dir,
            square_size_m=square_mm / 1000.0,
            device=args.device,
        )
        server = None
        try:
            engine.start()
            server = create_server(engine, args.host, args.port)
            print(f"浏览器访问: http://{args.host}:{args.port}")
            server.serve_forever()
        except KeyboardInterrupt:
            print("收到 Ctrl-C，正在停止服务...")
        except (RuntimeError, OSError, ValueError) as error:
            print(f"失败：{error}", file=sys.stderr)
            return 1
        finally:
            engine.action("stop")
            engine.join(timeout=5)
            if server is not None:
                server.server_close()
        return 1 if engine.status_snapshot()["state"] == "error" else 0
    try:
        pairs = capture_session(
            config,
            session_dir,
            square_size_m=square_mm / 1000.0,
            swap_eyes=args.swap_eyes,
        )
        minimum = int(config["capture"]["minimum_pairs"])
        if len(pairs) < minimum:
            print(f"RETAKE：仅采集 {len(pairs)} 对，至少需要 {minimum} 对。")
            return 1
        pattern = (int(config["board"]["columns"]), int(config["board"]["rows"]))
        result = solve_stereo(
            pairs,
            pattern,
            square_mm / 1000.0,
            config["validation"],
        )
        output_name = "final" if result.passed else "diagnostic"
        output_dir = session_dir / "results" / output_name
        export_result(result, output_dir, pairs[0], square_mm / 1000.0)
        print(
            f"{'PASS' if result.passed else 'RETAKE'}：有效 {result.valid_pair_count} 对，"
            f"左/右 RMS={result.mono_rms_left:.3f}/{result.mono_rms_right:.3f}px，"
            f"极线 P95={result.epipolar_p95:.3f}px"
        )
        print(f"结果目录: {output_dir}")
        if result.failure_reasons:
            print("原因: " + "；".join(result.failure_reasons))
        return 0 if result.passed else 1
    except (RuntimeError, ValueError) as error:
        print(f"失败：{error}", file=sys.stderr)
        print(f"session 已保留: {session_dir}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
