#!/usr/bin/env python3
"""Compute similarity between two embedding tensors saved as .pt files."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import torch
except ImportError:
    sys.stderr.write(
        "PyTorch is required to run this script. Install with `pip install torch`.\n"
    )
    sys.exit(1)


# Auto-detect: this file is at lste_ws/src/lste_core/scripts/scoring/, LSTE is at ../../../../LSTE/
_SCRIPT_DIR = Path(__file__).resolve().parents[0]
_LSTE_DIR = _SCRIPT_DIR.parents[4] / ".." / "LSTE"
DEFAULT_BASE_DIR = _LSTE_DIR / "Initialization" if _LSTE_DIR.is_dir() else _SCRIPT_DIR.parents[4] / ".." / ".." / "LSTE" / "Initialization"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Load two .pt files from the initialization directory and print their "
            "cosine similarity."
        )
    )
    parser.add_argument(
        "--a",
        required=True,
        help="First embedding file (name like book.pt or absolute path).",
    )
    parser.add_argument(
        "--b",
        required=True,
        help="Second embedding file (name like green.pt or absolute path).",
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=DEFAULT_BASE_DIR,
        help=f"Directory containing embedding files (default: {DEFAULT_BASE_DIR}).",
    )
    parser.add_argument(
        "--index-a",
        type=int,
        default=None,
        help="Row/index to pick when the first tensor has multiple embeddings.",
    )
    parser.add_argument(
        "--index-b",
        type=int,
        default=None,
        help="Row/index to pick when the second tensor has multiple embeddings.",
    )
    return parser.parse_args(argv)


def resolve_path(raw: str, base_dir: Path) -> Path:
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path


def load_embedding(path: Path) -> torch.Tensor:
    try:
        payload = torch.load(path, map_location="cpu")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"Embedding file not found: {path}") from exc
    except Exception as exc:  # pragma: no cover - defensive for torch load errors
        raise RuntimeError(f"Failed to load '{path}': {exc}") from exc

    def flatten(data: object) -> list[torch.Tensor]:
        if isinstance(data, torch.Tensor):
            return [data.reshape(-1)]

        if isinstance(data, (list, tuple)):
            if not data:
                raise ValueError(f"Empty embedding list in '{path}'.")
            pieces: list[torch.Tensor] = []
            for item in data:
                pieces.extend(flatten(item))
            return pieces

        if isinstance(data, dict):
            pieces: list[torch.Tensor] = []
            known_keys = [
                "embedding",
                "embeddings",
                "feat",
                "features",
                "positional_embeddings",
                "pos_embeds",
                "pos",
            ]
            used = set()
            for key in known_keys:
                if key in data:
                    pieces.extend(flatten(data[key]))
                    used.add(key)
            for key in data:
                if key not in used:
                    pieces.extend(flatten(data[key]))
            if not pieces:
                raise TypeError(
                    f"Dict payload in '{path}' does not contain embeddable values."
                )
            return pieces

        try:
            return [torch.as_tensor(data).reshape(-1)]
        except Exception as exc:  # pragma: no cover - fallback path
            raise TypeError(f"Unsupported payload type in '{path}': {type(data)}") from exc

    pieces = flatten(payload)
    if not pieces:
        raise ValueError(f"No embedding data found in '{path}'.")

    # Concatenate all flattened pieces into a single vector.
    tensor = torch.cat(pieces)

    tensor = tensor.float()
    if tensor.numel() == 0:
        raise ValueError(f"Empty embedding tensor in '{path}'.")
    return tensor


def select_vector(tensor: torch.Tensor, index: int | None, label: str, path: Path) -> torch.Tensor:
    if tensor.ndim == 1:
        vec = tensor
    else:
        if index is not None:
            if index < 0 or index >= tensor.shape[0]:
                raise IndexError(
                    f"--index-{label}={index} is out of range for tensor shape {tuple(tensor.shape)}"
                )
            vec = tensor[index]
        elif tensor.shape[0] == 1:
            vec = tensor[0]
        else:
            raise ValueError(
                f"Tensor from '{path}' has shape {tuple(tensor.shape)}; "
                f"use --index-{label} to pick which embedding to compare."
            )
    return vec.reshape(-1)


def cosine_similarity(vec_a: torch.Tensor, vec_b: torch.Tensor) -> float:
    if vec_a.numel() != vec_b.numel():
        raise ValueError(
            f"Embedding lengths differ: {vec_a.numel()} vs {vec_b.numel()}."
        )

    vec_a = vec_a.float()
    vec_b = vec_b.float()

    norm_a = torch.linalg.norm(vec_a)
    norm_b = torch.linalg.norm(vec_b)
    if norm_a.item() == 0 or norm_b.item() == 0:
        raise ValueError("Zero-norm embedding encountered; cannot compute similarity.")

    return torch.dot(vec_a, vec_b).item() / (norm_a.item() * norm_b.item())


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    base_dir = args.base_dir.expanduser()

    path_a = resolve_path(args.a, base_dir)
    path_b = resolve_path(args.b, base_dir)

    tensor_a = load_embedding(path_a)
    tensor_b = load_embedding(path_b)

    vec_a = select_vector(tensor_a, args.index_a, label="a", path=path_a)
    vec_b = select_vector(tensor_b, args.index_b, label="b", path=path_b)

    similarity = cosine_similarity(vec_a, vec_b)
    print(f"Cosine similarity: {similarity:.6f}")


if __name__ == "__main__":
    main()
