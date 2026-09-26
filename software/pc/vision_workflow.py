"""Audit, train, and evaluate the rescue-object YOLO dataset."""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import tempfile

import cv2
import numpy as np


CLASS_REGISTRY = Path(__file__).with_name("rescue_classes.json")
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".webp"}
SPLITS = ("train", "val", "test")


def load_class_registry(path: Path = CLASS_REGISTRY) -> list[dict]:
    try:
        registry = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read class registry {path}: {error}") from error
    classes = registry.get("classes") if isinstance(registry, dict) else None
    if not isinstance(classes, list) or not classes:
        raise ValueError("class registry must contain a non-empty 'classes' list")

    ids = [item.get("id") for item in classes if isinstance(item, dict)]
    names = [item.get("name") for item in classes if isinstance(item, dict)]
    if (
        len(ids) != len(classes)
        or any(type(class_id) is not int for class_id in ids)
        or ids != list(range(len(classes)))
    ):
        raise ValueError("class ids must be consecutive integers starting at zero")
    if any(not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) for name in names):
        raise ValueError("class names must use ASCII letters, digits, and underscores")
    if len(set(names)) != len(names):
        raise ValueError("class names must be unique")
    if registry.get("schema_version") != 1:
        raise ValueError("unsupported class registry schema_version")
    return classes


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _decode_image(path: Path) -> np.ndarray | None:
    try:
        encoded = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if encoded.size == 0:
        return None
    try:
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    except cv2.error:
        return None


def _validate_label_file(
    path: Path,
    class_count: int,
    *,
    relative_name: str,
) -> tuple[list[str], Counter[int]]:
    errors: list[str] = []
    counts: Counter[int] = Counter()
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except (OSError, UnicodeError) as error:
        return [f"{relative_name}: cannot read label file ({error})"], counts

    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        fields = line.split()
        prefix = f"{relative_name}:{line_number}"
        if len(fields) != 5:
            errors.append(f"{prefix}: expected 5 values, got {len(fields)}")
            continue
        try:
            class_id = int(fields[0])
            x_center, y_center, width, height = (float(value) for value in fields[1:])
        except ValueError:
            errors.append(f"{prefix}: class id must be an integer and box values numeric")
            continue
        if not 0 <= class_id < class_count:
            errors.append(f"{prefix}: class id {class_id} is outside 0..{class_count - 1}")
            continue
        values = (x_center, y_center, width, height)
        if not all(math.isfinite(value) for value in values):
            errors.append(f"{prefix}: box values must be finite")
            continue
        if not (0.0 <= x_center <= 1.0 and 0.0 <= y_center <= 1.0):
            errors.append(f"{prefix}: normalized box center must be within 0..1")
            continue
        if not (0.0 < width <= 1.0 and 0.0 < height <= 1.0):
            errors.append(f"{prefix}: normalized width and height must be in (0, 1]")
            continue
        if (
            x_center - width / 2.0 < -1e-6
            or x_center + width / 2.0 > 1.0 + 1e-6
            or y_center - height / 2.0 < -1e-6
            or y_center + height / 2.0 > 1.0 + 1e-6
        ):
            errors.append(f"{prefix}: bounding box extends beyond the image")
            continue
        counts[class_id] += 1
    return errors, counts


def audit_dataset(dataset_root: Path, classes: list[dict]) -> dict:
    root = dataset_root.expanduser().resolve()
    class_names = [item["name"] for item in classes]
    report = {
        "dataset_root": str(root),
        "class_names": class_names,
        "errors": [],
        "warnings": [],
        "splits": {},
        "classes_missing_from_train": [],
        "classes_missing_from_val": [],
    }
    all_hashes: dict[str, list[tuple[str, str]]] = defaultdict(list)

    for split in SPLITS:
        image_root = root / "images" / split
        label_root = root / "labels" / split
        required = split in ("train", "val")
        if not image_root.is_dir():
            if required:
                report["errors"].append(f"missing required image directory: {image_root}")
            continue
        if not label_root.is_dir():
            report["errors"].append(f"missing label directory: {label_root}")
            continue

        images = sorted(
            path for path in image_root.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        )
        if not images:
            message = f"no supported images found in {image_root}"
            (report["errors"] if required else report["warnings"]).append(message)

        counts: Counter[int] = Counter()
        expected_labels: set[Path] = set()
        backgrounds = 0
        split_image_names: set[Path] = set()
        for image_path in images:
            image_relative = image_path.relative_to(image_root)
            label_relative = image_relative.with_suffix(".txt")
            if label_relative in split_image_names:
                report["errors"].append(
                    f"multiple image files map to one label name in {split}: {image_relative}"
                )
            split_image_names.add(label_relative)
            expected_labels.add(label_relative)
            label_path = label_root / label_relative
            display_name = f"{split}/{image_relative.as_posix()}"

            decoded = _decode_image(image_path)
            if decoded is None or decoded.size == 0:
                report["errors"].append(f"{display_name}: image cannot be decoded")
            else:
                try:
                    image_hash = _sha256(image_path)
                except OSError as error:
                    report["errors"].append(f"{display_name}: cannot read image bytes ({error})")
                else:
                    all_hashes[image_hash].append((split, image_relative.as_posix()))

            if not label_path.is_file():
                report["errors"].append(
                    f"{display_name}: missing label file; use an empty .txt for a verified background"
                )
                continue
            label_errors, label_counts = _validate_label_file(
                label_path,
                len(classes),
                relative_name=f"labels/{display_name.rsplit('/', 1)[0]}/{label_relative.name}",
            )
            report["errors"].extend(label_errors)
            counts.update(label_counts)
            if not label_counts and not label_errors:
                backgrounds += 1

        actual_labels = {
            path.relative_to(label_root)
            for path in label_root.rglob("*")
            if path.is_file() and path.suffix.lower() == ".txt"
        }
        for orphan in sorted(actual_labels - expected_labels):
            report["errors"].append(f"{split}: orphan label has no matching image: {orphan}")

        report["splits"][split] = {
            "images": len(images),
            "background_images": backgrounds,
            "instances_by_class": {
                str(class_id): counts[class_id] for class_id in range(len(classes))
            },
        }

    for image_hash, locations in all_hashes.items():
        split_names = {split for split, _ in locations}
        if len(split_names) > 1:
            places = ", ".join(f"{split}/{name}" for split, name in locations)
            report["errors"].append(f"identical image content crosses dataset splits: {places}")
        elif len(locations) > 1:
            places = ", ".join(name for _, name in locations)
            report["warnings"].append(
                f"duplicate image content within {locations[0][0]} split: {places}"
            )

    for split, key in (("train", "classes_missing_from_train"), ("val", "classes_missing_from_val")):
        counts = report.get("splits", {}).get(split, {}).get("instances_by_class", {})
        missing = [
            class_names[class_id]
            for class_id in range(len(class_names))
            if int(counts.get(str(class_id), 0)) == 0
        ]
        report[key] = missing
        if missing:
            report["warnings"].append(f"{split} split has no instances for: {', '.join(missing)}")
    return report


def print_report(report: dict) -> None:
    print(f"Dataset: {report['dataset_root']}")
    for split, values in report["splits"].items():
        detail = ", ".join(
            f"{report['class_names'][int(class_id)]}={count}"
            for class_id, count in values["instances_by_class"].items()
        )
        print(
            f"{split}: images={values['images']}, backgrounds={values['background_images']}; "
            f"instances: {detail}"
        )
    for severity, messages in (("ERROR", report["errors"]), ("WARNING", report["warnings"])):
        for message in messages:
            print(f"{severity}: {message}")
    print(f"Result: {len(report['errors'])} error(s), {len(report['warnings'])} warning(s)")


def _write_dataset_yaml(dataset_root: Path, classes: list[dict]) -> Path:
    root = dataset_root.expanduser().resolve()
    rows = [f"path: {json.dumps(root.as_posix(), ensure_ascii=False)}"]
    rows.append("train: images/train")
    rows.append("val: images/val")
    if (root / "images" / "test").is_dir():
        rows.append("test: images/test")
    rows.append("names:")
    rows.extend(f"  {item['id']}: {item['name']}" for item in classes)
    temporary = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="rescue_dataset_",
        delete=False,
    )
    with temporary:
        temporary.write("\n".join(rows) + "\n")
    return Path(temporary.name)


def _next_run_directory(project: Path, name: str) -> Path:
    project_root = project.expanduser().resolve()
    candidate = project_root / name
    suffix = 2
    while candidate.exists():
        candidate = project_root / f"{name}_{suffix}"
        suffix += 1
    return candidate


def _load_ultralytics():
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError(
            "training/evaluation needs Ultralytics; install software/pc/requirements-yolo.txt"
        ) from error
    return YOLO


def _model_class_names(model) -> list[str] | None:
    names = getattr(model, "names", None)
    if isinstance(names, dict):
        if not names or any(type(class_id) is not int for class_id in names):
            return None
        if sorted(names) != list(range(len(names))):
            return None
        return [str(names[class_id]) for class_id in range(len(names))]
    if isinstance(names, (list, tuple)) and names:
        return [str(name) for name in names]
    return None


def _run_training(args: argparse.Namespace, classes: list[dict]) -> int:
    report = audit_dataset(args.dataset_root, classes)
    print_report(report)
    if report["errors"]:
        return 2
    class_gaps = report["classes_missing_from_train"] + report["classes_missing_from_val"]
    if class_gaps and not args.allow_incomplete_classes:
        print("Training stopped: add examples for every class or pass --allow-incomplete-classes for a prototype.")
        return 2

    try:
        YOLO = _load_ultralytics()
        model = YOLO(args.model)
        run_directory = _next_run_directory(args.project, args.name)
        data_yaml = _write_dataset_yaml(args.dataset_root, classes)
        try:
            model.train(
                data=str(data_yaml),
                epochs=args.epochs,
                imgsz=args.imgsz,
                batch=args.batch,
                device=args.device,
                workers=args.workers,
                seed=args.seed,
                deterministic=True,
                project=str(run_directory.parent),
                name=run_directory.name,
                exist_ok=False,
                plots=True,
            )
        finally:
            data_yaml.unlink(missing_ok=True)
    except (ImportError, RuntimeError, OSError, ValueError) as error:
        print(f"Training failed: {error}", file=sys.stderr)
        return 2
    print(f"Training finished. Best weights: {run_directory / 'weights' / 'best.pt'}")
    print("Review per-class validation metrics before using the model in autonomous decisions.")
    return 0


def _run_evaluation(args: argparse.Namespace, classes: list[dict]) -> int:
    report = audit_dataset(args.dataset_root, classes)
    print_report(report)
    if report["errors"]:
        return 2
    if args.split not in report["splits"] or report["splits"][args.split]["images"] == 0:
        print(f"Evaluation split is missing or empty: {args.split}", file=sys.stderr)
        return 2
    if not args.weights.is_file():
        print(f"weights file does not exist: {args.weights}", file=sys.stderr)
        return 2
    try:
        YOLO = _load_ultralytics()
        model = YOLO(str(args.weights))
        actual_names = _model_class_names(model)
        expected_names = [item["name"] for item in classes]
        if actual_names != expected_names:
            print(
                "Evaluation stopped: weight class order does not match the rescue class registry. "
                f"Expected {expected_names}; got {actual_names}.",
                file=sys.stderr,
            )
            return 2
        run_directory = _next_run_directory(args.project, args.name)
        data_yaml = _write_dataset_yaml(args.dataset_root, classes)
        try:
            model.val(
                data=str(data_yaml),
                split=args.split,
                imgsz=args.imgsz,
                batch=args.batch,
                device=args.device,
                plots=True,
                project=str(run_directory.parent),
                name=run_directory.name,
                exist_ok=False,
            )
        finally:
            data_yaml.unlink(missing_ok=True)
    except (ImportError, RuntimeError, OSError, ValueError) as error:
        print(f"Evaluation failed: {error}", file=sys.stderr)
        return 2
    print(f"Evaluation outputs: {run_directory}")
    return 0


def _add_dataset_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--classes", type=Path, default=CLASS_REGISTRY)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    audit = subparsers.add_parser("audit", help="check images, labels, class coverage, and split leakage")
    _add_dataset_arguments(audit)
    audit.add_argument("--report", type=Path, help="optional JSON report output path")

    train = subparsers.add_parser("train", help="train a custom rescue detector")
    _add_dataset_arguments(train)
    train.add_argument("--model", required=True, help="pretrained .pt weights or model YAML")
    train.add_argument("--epochs", type=int, default=100)
    train.add_argument("--imgsz", type=int, default=416)
    train.add_argument("--batch", type=int, default=8)
    train.add_argument("--device", default="0", help="for example 0 or cpu")
    train.add_argument("--workers", type=int, default=4)
    train.add_argument("--seed", type=int, default=42)
    train.add_argument("--project", type=Path, default=Path("runs/rescue"))
    train.add_argument("--name", default="rescue")
    train.add_argument("--allow-incomplete-classes", action="store_true")

    evaluate = subparsers.add_parser("evaluate", help="evaluate weights on val or held-out test images")
    _add_dataset_arguments(evaluate)
    evaluate.add_argument("--weights", type=Path, required=True)
    evaluate.add_argument("--split", choices=("val", "test"), default="val")
    evaluate.add_argument("--imgsz", type=int, default=416)
    evaluate.add_argument("--batch", type=int, default=8)
    evaluate.add_argument("--device", default="0", help="for example 0 or cpu")
    evaluate.add_argument("--project", type=Path, default=Path("runs/rescue_eval"))
    evaluate.add_argument("--name", default="evaluation")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        classes = load_class_registry(args.classes)
    except ValueError as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return 2

    if args.command == "audit":
        report = audit_dataset(args.dataset_root, classes)
        print_report(report)
        if args.report is not None:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"JSON report saved: {args.report}")
        return 1 if report["errors"] else 0
    if args.command == "train":
        if args.epochs <= 0 or args.imgsz <= 0 or args.batch <= 0 or args.workers < 0:
            parser.error("epochs, imgsz, and batch must be positive; workers cannot be negative")
        if not args.name or "/" in args.name or "\\" in args.name or args.name in (".", ".."):
            parser.error("name must be a single directory name")
        return _run_training(args, classes)
    if args.command == "evaluate":
        if args.imgsz <= 0 or args.batch <= 0:
            parser.error("imgsz and batch must be positive")
        if not args.name or "/" in args.name or "\\" in args.name or args.name in (".", ".."):
            parser.error("name must be a single directory name")
        return _run_evaluation(args, classes)
    parser.error(f"unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
