# 树状图生成脚本
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tree_pith.py — 打印项目树, 并在每个文件后附上它的用途说明。

描述来源优先级:
  1. // title = xxx   或  * title = xxx   或  # title = xxx
  2. // @brief xxx
  3. 头部第一段注释里的第一行非空内容

用法:
  python tree_pith.py [项目根目录]
  python tree_pith.py . -d 3
  python tree_pith.py . --no-desc
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Optional


# ============================ 配置 ============================

IGNORE_DIRS = {
    ".git", ".pio", ".vscode", ".idea", "build", "dist",
    "__pycache__", ".pytest_cache", "node_modules",
    ".doxygen", "managed_components",
}

CODE_SUFFIXES = {
    ".c", ".h", ".cpp", ".hpp", ".cc", ".hh", ".cxx",
    ".py", ".ino",
    ".md", ".txt", ".cfg", ".ini",
    ".cmake", ".mk", ".sh", ".ps1",
}

KNOWN_FILES = {
    "platformio.ini":     "PlatformIO 工程配置",
    "sdkconfig.defaults": "ESP-IDF 默认配置",
    "partitions_2app.csv":"双 App OTA 分区表",
    "partitions.csv":     "分区表",
    "CMakeLists.txt":     "CMake 构建脚本",
    "README":             "项目说明",
    "README.md":          "项目说明",
    "LICENSE":            "许可证",
    ".gitignore":         "Git 忽略规则",
}

HEAD_LINES = 40


# ============================ 注释剥壳 + 描述提取 ============================
# ============================ 注释剥壳 ============================

def _strip_marker(line: str) -> str:
    """剥掉行首注释标记 (// /**, /*, *, #, ;) 和行尾 */, 返回内容。"""
    s = line.strip()
    s = re.sub(r'^(?:/\*+|//+|#+|;+|\*+)\s*', '', s)   # 行首
    s = re.sub(r'\s*\*+/\s*$', '', s)                  # 行尾 */
    return s


# 支持: title = xxx / title: xxx / title xxx / @title: xxx
#      @brief / @description 同理
RE_TITLE = re.compile(r'^@?title\s*[:=]?\s*(.*)$', re.IGNORECASE)
RE_BRIEF = re.compile(r'^@?brief\s*[:=]?\s*(.*)$', re.IGNORECASE)
RE_DESC  = re.compile(r'^@?description\s*[:=]?\s*(.*)$', re.IGNORECASE)


def _clean(text: str) -> str:
    text = text.strip()
    text = re.sub(r'^[=\-*#\s]+', '', text)
    text = re.sub(r'[=\-*#\s]+$', '', text)
    if len(text) > 60:
        text = text[:57] + "..."
    return text


def extract_desc(path: Path) -> str:
    name = path.name
    if name in KNOWN_FILES:
        return KNOWN_FILES[name]

    if path.suffix.lower() not in CODE_SUFFIXES:
        return ""

    try:
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            head = [next(f, "") for _ in range(HEAD_LINES)]
    except (OSError, StopIteration):
        return ""

    content = [_strip_marker(l) for l in head]

    # ---- 1) title（优先级最高） ----
    for idx, c in enumerate(content):
        m = RE_TITLE.match(c)
        if m:
            val = m.group(1).strip()
            if not val:                       # "title" 独占一行 → 取下一行非空
                val = _next_nonempty(content, idx)
            if val:
                return _clean(val)

    # ---- 2) @description ----
    for idx, c in enumerate(content):
        m = RE_DESC.match(c)
        if m:
            val = m.group(1).strip()
            if not val:                       # "@description:" 后空 → 取下一行
                val = _next_nonempty(content, idx)
            if val:
                return _clean(val)

    # ---- 3) @brief ----
    for idx, c in enumerate(content):
        m = RE_BRIEF.match(c)
        if m:
            val = m.group(1).strip()
            if not val:
                val = _next_nonempty(content, idx)
            if val:
                return _clean(val)

    # ---- 4) 第一段注释里的第一行非空 ----
    for c in content:
        if not c:
            continue
        if c.startswith("#!") or "coding" in c:
            continue
        if set(c) <= set("=-* "):             # 纯分隔线
            continue
        # 跳过纯标签行 (author / date 等)
        if re.match(r'^@?\w+\s*[:=]', c):
            continue
        return _clean(c)

    return ""


def _next_nonempty(content, start_idx: int) -> str:
    """从 start_idx+1 开始找第一行非空内容。"""
    for c in content[start_idx + 1:]:
        c = c.strip()
        if c and not set(c) <= set("=-* "):
            return c
    return ""


# ============================ 树渲染 ============================

def walk(root: Path, max_depth: Optional[int], show_desc: bool):

    def visit(dir_path: Path, prefix: str, depth: int):
        if max_depth is not None and depth >= max_depth:
            return

        try:
            entries = list(dir_path.iterdir())
        except PermissionError:
            return

        entries = [
            p for p in entries
            if not (p.is_dir() and p.name in IGNORE_DIRS)
            and not p.name.startswith(".pio")
        ]

        dirs  = sorted([p for p in entries if p.is_dir()],  key=lambda p: p.name.lower())
        files = sorted([p for p in entries if p.is_file()], key=lambda p: p.name.lower())
        ordered = dirs + files

        for i, p in enumerate(ordered):
            is_last  = (i == len(ordered) - 1)
            branch   = "└── " if is_last else "├── "
            next_pad = "    " if is_last else "│   "

            if p.is_dir():
                print(f"{prefix}{branch}📂 {p.name}/")
                visit(p, prefix + next_pad, depth + 1)
            else:
                desc = extract_desc(p) if show_desc else ""
                if desc:
                    print(f"{prefix}{branch}📄 {p.name}    # {desc}")
                else:
                    print(f"{prefix}{branch}📄 {p.name}")

    print(f"📦 {root.resolve().name}/")
    visit(root, "", 0)


# ============================ 入口 ============================

def main() -> int:
    ap = argparse.ArgumentParser(prog="tree_pith",
                                 description="打印项目树, 每个文件后附用途说明。")
    ap.add_argument("root", nargs="?", default=".", help="项目根目录, 默认当前目录")
    ap.add_argument("-d", "--depth", type=int, default=None, help="最大深度")
    ap.add_argument("--no-desc", action="store_true", help="不读取文件描述")
    args = ap.parse_args()

    root = Path(args.root)
    if not root.exists() or not root.is_dir():
        print(f"[tree_pith] 不是有效目录: {root}", file=sys.stderr)
        return 1

    walk(root, args.depth, show_desc=not args.no_desc)
    return 0


if __name__ == "__main__":
    sys.exit(main())

# python .\src\util\tree_pith.py