#!/usr/bin/env python3
"""Verify that local links and assets in the built static site resolve."""

from __future__ import annotations

import sys
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "_site"
SITE_PREFIX = "/2026Spring/"


class ReferenceParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.references: list[tuple[str, str]] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attributes = dict(attrs)
        for attribute in ("href", "src"):
            value = attributes.get(attribute)
            if value:
                self.references.append((attribute, value))

        srcset = attributes.get("srcset")
        if srcset:
            for candidate in srcset.split(","):
                value = candidate.strip().split(" ", 1)[0]
                if value:
                    self.references.append(("srcset", value))


def local_target(document: Path, reference: str) -> Path | None:
    parsed = urlsplit(reference)
    if parsed.scheme or parsed.netloc or reference.startswith(("#", "mailto:", "data:")):
        return None

    path = unquote(parsed.path)
    if not path:
        return None
    if path.startswith(SITE_PREFIX):
        target = SITE / path[len(SITE_PREFIX):]
    elif path.startswith("/"):
        target = SITE / path.lstrip("/")
    else:
        target = document.parent / path

    if path.endswith("/"):
        target /= "index.html"
    elif not target.suffix and target.is_dir():
        target /= "index.html"
    return target.resolve()


def main() -> int:
    if not SITE.is_dir():
        print(f"Site directory does not exist: {SITE}", file=sys.stderr)
        return 1

    missing: list[tuple[Path, str, Path]] = []
    html_files = sorted(SITE.rglob("*.html"))
    for document in html_files:
        parser = ReferenceParser()
        parser.feed(document.read_text(encoding="utf-8"))
        for _attribute, reference in parser.references:
            target = local_target(document, reference)
            if target is not None and not target.exists():
                missing.append((document.relative_to(SITE), reference, target))

    if missing:
        for document, reference, target in missing:
            print(f"{document}: {reference} -> missing {target}", file=sys.stderr)
        print(f"Found {len(missing)} broken local references.", file=sys.stderr)
        return 1

    print(f"Checked {len(html_files)} HTML files; all local references resolve.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
