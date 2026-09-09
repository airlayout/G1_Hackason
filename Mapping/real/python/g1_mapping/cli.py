"""mapctlコマンド本体。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import os
import subprocess
import sys

from .compose import ComposeError, ComposeRuntime
from .config import Settings
from .doctor import DiagnosticReport, diagnose, print_report
from .mock import write_mock_artifacts
from .pcd import validate_session
from .rebuild import rebuild_map
from .session import MappingSession, SessionManager


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mapctl", description="Unitree G1 部屋Mappingの共通操作CLI"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("build", help="onboard/raw Docker imageを構築する")

    doctor = subparsers.add_parser("doctor", help="接続・センサー・backendを非破壊診断する")
    doctor.add_argument(
        "--backend", choices=("auto", "onboard", "raw", "sim"), default="auto"
    )
    doctor.add_argument("--timeout", type=int, default=5)
    doctor.add_argument("--json", action="store_true", dest="as_json")
    doctor.add_argument("--mock", action="store_true")

    start = subparsers.add_parser("start", help="新しいMappingセッションを開始する")
    start.add_argument(
        "--backend",
        choices=("auto", "onboard", "raw", "sim", "mock"),
        default="auto",
    )
    start.add_argument("--name", default="room")
    start.add_argument("--timeout", type=int, default=5)
    start.add_argument(
        "--force",
        action="store_true",
        help="backend probe失敗時も明示指定したbackendを開始する",
    )

    subparsers.add_parser("status", help="実行中セッションとコンテナ状態を表示する")

    stop = subparsers.add_parser("stop", help="地図を保存してセッションを停止する")
    stop.add_argument("--allow-partial", action="store_true")

    validate = subparsers.add_parser("validate", help="PCD・rosbag・軌跡を検証する")
    validate.add_argument("session_id", nargs="?")
    validate.add_argument("--allow-partial", action="store_true")
    validate.add_argument("--json", action="store_true", dest="as_json")

    view = subparsers.add_parser("view", help="ライブ地図または保存PCDをRVizで表示する")
    view.add_argument("session_id", nargs="?")
    view.add_argument("--live", action="store_true", help="実行中セッションを表示する")
    view.add_argument(
        "--publish-only",
        action="store_true",
        help="保存PCDを配信し、RVizは起動しない",
    )
    view.add_argument("--fixed-frame")
    view.add_argument("--map-topic")

    rebuild = subparsers.add_parser(
        "rebuild", help="rosbagの地図点群からmap_raw.pcdを再構成する"
    )
    rebuild.add_argument("session_id", nargs="?")
    rebuild.add_argument(
        "--topic", help="使うPointCloud2トピック（既定: 共通の登録済み点群）"
    )
    rebuild.add_argument(
        "--voxel",
        type=float,
        default=0.05,
        help="ボクセル一辺[m]。0で間引きなし（既定: 0.05）",
    )
    rebuild.add_argument("--output", type=Path, help="出力先PCD（既定: セッションのmap/）")
    rebuild.add_argument(
        "--force", action="store_true", help="既存のmap_raw.pcdを上書きする"
    )

    bundle = subparsers.add_parser("bundle", help="オフライン現場配備物を作る")
    bundle.add_argument("--output", type=Path)
    return parser


def _choose_backend(
    settings: Settings,
    requested: str,
    *,
    timeout: int,
    force: bool,
) -> tuple[str, DiagnosticReport]:
    if requested == "mock":
        report = diagnose(settings, requested_backend="onboard", mock=True)
        return "mock", report

    report = diagnose(
        settings, requested_backend=requested, mock=False, timeout_seconds=timeout
    )
    if report.ready and report.selected_backend:
        return report.selected_backend, report
    host_failed = any(
        check.status == "FAIL" and check.name.startswith("host.")
        for check in report.checks
    )
    if force and not host_failed and requested in {"onboard", "raw", "sim"}:
        return requested, report
    print_report(report)
    raise RuntimeError("利用可能なbackendを確認できませんでした")


def _print_validation(report: dict[str, object]) -> None:
    for check in report["checks"]:
        assert isinstance(check, dict)
        print(f"[{check['status']}] {check['name']}: {check['message']}")
    print("[RESULT_OK] 地図成果物は有効です" if report["success"] else "[RESULT_NG] 成果物に問題があります")


def _start(settings: Settings, arguments: argparse.Namespace) -> int:
    backend, report = _choose_backend(
        settings,
        arguments.backend,
        timeout=arguments.timeout,
        force=arguments.force,
    )
    manager = SessionManager(settings)
    session = manager.create(
        name=arguments.name,
        backend=backend,
        diagnostic=report.to_dict(),
    )
    try:
        if backend != "mock":
            ComposeRuntime(settings).start(session)
        manager.update(session, "running", f"{backend} backendでMapping中です")
    except Exception as exc:
        manager.finish(session, success=False, message=f"開始失敗: {exc}")
        raise

    print(f"[STARTED] {session.session_id}")
    print(f"[BACKEND] {backend}")
    print(f"[OUTPUT] {session.directory}")
    if backend in {"onboard", "raw"}:
        print("[NEXT] G1を5〜10秒静止させてから、純正リモコンで低速移動してください")
    elif backend == "sim":
        print("[NEXT] Isaac Simを走行させ、別端末で ./mapctl view --live を実行できます")
    return 0


def _stop(settings: Settings, arguments: argparse.Namespace) -> int:
    manager = SessionManager(settings)
    session = manager.active()
    if session is None:
        raise RuntimeError("実行中のMappingセッションがありません")
    manager.update(session, "stopping", "地図保存と記録停止を実行中です")
    errors: list[str] = []

    if session.backend == "mock":
        write_mock_artifacts(session)
    else:
        runtime = ComposeRuntime(settings)
        try:
            runtime.save_and_stop(session)
        except Exception as exc:
            errors.append(f"backend停止: {exc}")
        try:
            runtime.collect_logs(session)
        except Exception as exc:
            errors.append(f"ログ回収: {exc}")
        if not session.map_path.is_file():
            try:
                rebuild_map(
                    session.directory,
                    topic="/g1_mapping/cloud_registered",
                    output_path=session.map_path,
                    voxel_size=0.05,
                )
            except Exception as exc:
                errors.append(f"共通登録点群からのPCD再構成: {exc}")

    valid, report = validate_session(
        session.directory,
        min_map_points=settings.min_map_points,
        allow_partial=arguments.allow_partial,
    )
    _print_validation(report)
    success = valid and (not errors or arguments.allow_partial)
    message = "Mappingを完了しました"
    if errors:
        message = "; ".join(errors)
        for error in errors:
            print(f"[WARN] {error}")
    manager.finish(session, success=success, message=message)
    print(f"[OUTPUT] {session.directory}")
    return 0 if success else 1


def _status(settings: Settings) -> int:
    manager = SessionManager(settings)
    session = manager.active()
    if session is None:
        print("[IDLE] 実行中のMappingセッションはありません")
        try:
            latest = manager.resolve()
        except RuntimeError:
            return 0
        print(f"[LATEST] {latest.session_id}: {manager.state(latest)['status']}")
        return 0

    state = manager.state(session)
    print(f"[SESSION] {session.session_id}")
    print(f"[BACKEND] {session.backend}")
    print(f"[STATUS] {state['status']}: {state['message']}")
    if session.backend != "mock":
        print(ComposeRuntime(settings).ps(session))
    return 0


def _validate(settings: Settings, arguments: argparse.Namespace) -> int:
    session = SessionManager(settings).resolve(arguments.session_id)
    success, report = validate_session(
        session.directory,
        min_map_points=settings.min_map_points,
        allow_partial=arguments.allow_partial,
    )
    if arguments.as_json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        _print_validation(report)
        print(f"[OUTPUT] {session.directory / 'report' / 'quality.json'}")
    return 0 if success else 1


def _view(settings: Settings, arguments: argparse.Namespace) -> int:
    if arguments.live and arguments.session_id:
        raise ValueError("--liveとsession_idは同時に指定できません")
    if arguments.live and arguments.publish_only:
        raise ValueError("--publish-onlyは保存PCDの表示でのみ利用できます")

    manager = SessionManager(settings)
    if arguments.live:
        session = manager.active()
        if session is None:
            raise RuntimeError("ライブ表示には実行中のMappingセッションが必要です")
        if session.backend == "mock":
            raise RuntimeError("mockセッションはライブ表示できません。保存後に表示してください")
        mode = "live"
        pcd_path = ""
        fixed_frame = arguments.fixed_frame or "map"
        map_topic = arguments.map_topic or "/g1_mapping/map"
    else:
        session = manager.resolve(arguments.session_id)
        if not session.map_path.is_file():
            raise RuntimeError(f"PCDがありません: {session.map_path}")
        mode = "saved"
        pcd_path = f"/runs/{session.session_id}/map/map_raw.pcd"
        fixed_frame = arguments.fixed_frame or "map"
        map_topic = arguments.map_topic or "/g1_mapping/map"

    rviz = not arguments.publish_only
    if rviz:
        display = os.environ.get("DISPLAY", "")
        if not display:
            raise RuntimeError(
                "DISPLAYがありません。GUI端末で実行するか--publish-onlyを指定してください"
            )
        xauthority = os.environ.get("XAUTHORITY")
        if xauthority and not Path(xauthority).is_file():
            raise RuntimeError(f"XAUTHORITYが見つかりません: {xauthority}")

    print(
        f"[VIEW] mode={mode} frame={fixed_frame} topic={map_topic}", flush=True
    )
    if mode == "saved":
        print(f"[PCD] {session.map_path}", flush=True)
    try:
        ComposeRuntime(settings).view(
            mode=mode,
            pcd_path=pcd_path,
            fixed_frame=fixed_frame,
            map_topic=map_topic,
            rviz=rviz,
            backend=session.backend if mode == "live" else None,
        )
    except KeyboardInterrupt:
        print("\n[STOPPED] 可視化を終了しました")
        return 130
    return 0


def _rebuild(settings: Settings, arguments: argparse.Namespace) -> int:
    """rosbagに残っている地図点群から map_raw.pcd を作り直す。"""

    manager = SessionManager(settings)
    session = manager.resolve(arguments.session_id)
    output_path = arguments.output or session.map_path
    if output_path.exists() and not arguments.force:
        # 実機から回収した地図のほうが正なので、黙って潰さない
        raise RuntimeError(
            f"既に存在します: {output_path}（上書きするなら --force）"
        )
    topic = arguments.topic or "/g1_mapping/cloud_registered"
    print(f"[REBUILD] session={session.session_id} backend={session.backend}")
    print(f"[REBUILD] topic={topic} voxel={arguments.voxel}m")

    def report(messages: int, raw_points: int, kept: int) -> None:
        print(
            f"  {messages} msg / {raw_points} points -> {kept} points",
            flush=True,
        )

    result = rebuild_map(
        session.directory,
        topic=topic,
        output_path=output_path,
        voxel_size=arguments.voxel,
        progress=report,
    )
    extent = [
        result.maximum_xyz[index] - result.minimum_xyz[index] for index in range(3)
    ]
    print(f"[OK] {result.written_points} points を書き出しました")
    print(f"[INFO] 読み込み {result.message_count} メッセージ / {result.raw_points} points")
    print(f"[INFO] 範囲 x={extent[0]:.2f}m y={extent[1]:.2f}m z={extent[2]:.2f}m")
    print(f"[OUTPUT] {result.output_path}")
    print("[NEXT] ./mapctl validate で検証できます")
    return 0


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    settings = Settings.load()
    try:
        if arguments.command == "build":
            ComposeRuntime(settings).build()
            print("[OK] Mapping imagesを構築しました")
            return 0
        if arguments.command == "doctor":
            report = diagnose(
                settings,
                requested_backend=arguments.backend,
                mock=arguments.mock,
                timeout_seconds=arguments.timeout,
            )
            if arguments.as_json:
                print(json.dumps(report.to_dict(), ensure_ascii=False, indent=2))
            else:
                print_report(report)
            return 0 if report.ready else 1
        if arguments.command == "start":
            return _start(settings, arguments)
        if arguments.command == "status":
            return _status(settings)
        if arguments.command == "stop":
            return _stop(settings, arguments)
        if arguments.command == "validate":
            return _validate(settings, arguments)
        if arguments.command == "view":
            return _view(settings, arguments)
        if arguments.command == "rebuild":
            return _rebuild(settings, arguments)
        if arguments.command == "bundle":
            script = settings.project_dir / "scripts" / "make-field-kit.sh"
            command = [str(script)]
            if arguments.output:
                command.append(str(arguments.output))
            return subprocess.run(command, check=False).returncode
    except (ComposeError, RuntimeError, ValueError, OSError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    raise AssertionError(f"未処理のcommandです: {arguments.command}")
