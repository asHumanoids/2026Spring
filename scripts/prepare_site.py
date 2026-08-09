#!/usr/bin/env python3
"""Assemble the repository's course documents into the MkDocs source tree."""

from __future__ import annotations

import re
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / ".site-docs"
REPOSITORY_URL = "https://github.com/asHumanoids/2026Spring"

LESSONS = (
    {
        "source": "C01_RobotPerspectives",
        "destination": "module-01/robot-perspectives",
        "label": "Class 01 · Module 01",
        "slides": "../../slides/robot-perspectives/",
    },
    {
        "source": "C02_RobotDescription",
        "destination": "module-01/robot-description",
        "label": "Class 02 · Module 01",
    },
    {
        "source": "C03_MuJoCo",
        "destination": "module-01/mujoco",
        "label": "Class 03 · Module 01",
    },
    {
        "source": "00-DVSI",
        "destination": "module-02/simulation-basics",
        "label": "Class 04 · Module 02",
    },
    {
        "source": "01-KG-03-MovRetarget",
        "destination": "module-02/motion-retargeting",
        "label": "Class 05 · Module 02",
    },
    {
        "source": "01-KG-04-HumanoidBasics",
        "destination": "module-02/humanoid-simulation",
        "label": "Class 06 · Module 02",
        "notebooks": "../../notebooks/",
    },
    {
        "source": "01-KG-05-HumanoidAdvanced",
        "destination": "module-02/humanoid-walking",
        "label": "Class 07 · Module 02",
        "notebooks": "../../notebooks/",
    },
    {
        "source": "02-DC-03-WholeBody",
        "destination": "module-02/whole-body-control",
        "label": "Class 08 · Module 02",
    },
    {
        "source": "02-DC-04-ReinforceL",
        "destination": "module-02/reinforcement-learning",
        "label": "Class 09 · Module 02",
    },
    {
        "source": "03-FM-01-VLA",
        "destination": "module-02/foundation-models",
        "label": "Class 10 · Module 02",
    },
    {
        "source": "C11_HumanoidMovement",
        "destination": "module-03/humanoid-movement",
        "label": "Class 11 · Module 03",
    },
    {
        "source": "C12_HumanoidContact",
        "destination": "module-03/humanoid-contact",
        "label": "Class 12 · Module 03",
    },
    {
        "source": "C13_HumanoidControl",
        "destination": "module-03/humanoid-control",
        "label": "Class 13 · Module 03",
    },
    {
        "source": "C14_HumanoidLearning",
        "destination": "module-03/humanoid-learning",
        "label": "Class 14 · Module 03",
    },
    {
        "source": "FinalProjectExample",
        "destination": "projects/final-project-example",
        "label": "Module 04 · Final Project",
    },
)

MEDIA_SUFFIXES = {
    ".gif",
    ".jpeg",
    ".jpg",
    ".mp4",
    ".pdf",
    ".png",
    ".svg",
    ".webm",
    ".webp",
}


def reset_docs() -> None:
    if DOCS.name != ".site-docs" or DOCS.parent != ROOT:
        raise RuntimeError(f"Refusing to reset unexpected path: {DOCS}")
    if DOCS.exists():
        shutil.rmtree(DOCS)
    DOCS.mkdir(parents=True)


def read_readme(directory: Path) -> str:
    for filename in ("README.md", "ReadMe.md", "readme.md"):
        candidate = directory / filename
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    raise FileNotFoundError(f"No README found in {directory}")


def insert_after_title(markdown: str, block: str) -> str:
    lines = markdown.splitlines()
    for index, line in enumerate(lines):
        if line.startswith("# "):
            lines[index + 1:index + 1] = ["", block, ""]
            return "\n".join(lines).rstrip() + "\n"
    return f"{block}\n\n{markdown.rstrip()}\n"


def lesson_actions(lesson: dict[str, str]) -> str:
    source_url = f"{REPOSITORY_URL}/tree/main/{lesson['source']}"
    actions = [
        f'<a class="md-button" href="{source_url}">View source on GitHub</a>'
    ]
    if "slides" in lesson:
        actions.insert(
            0,
            f'<a class="md-button md-button--primary" href="{lesson["slides"]}">Open full-screen slides</a>',
        )
    if "notebooks" in lesson:
        actions.append(
            f'<a class="md-button" href="{lesson["notebooks"]}">Open notebooks</a>'
        )
    return (
        f'<p class="lesson-kicker">{lesson["label"]}</p>\n'
        f'<div class="page-actions">{"".join(actions)}</div>'
    )


def copy_lesson_assets(source: Path, destination: Path, skip_images: bool) -> None:
    for asset_directory in ("images", "assets"):
        candidate = source / asset_directory
        if not candidate.is_dir() or (skip_images and asset_directory == "images"):
            continue
        shutil.copytree(candidate, destination / asset_directory, dirs_exist_ok=True)

    for candidate in source.iterdir():
        if candidate.is_file() and candidate.suffix.lower() in MEDIA_SUFFIXES:
            shutil.copy2(candidate, destination / candidate.name)


def prepare_lessons() -> None:
    for lesson in LESSONS:
        source = ROOT / lesson["source"]
        destination = DOCS / lesson["destination"]
        destination.mkdir(parents=True, exist_ok=True)

        markdown = read_readme(source)
        if lesson["source"] == "C01_RobotPerspectives":
            markdown = markdown.replace(
                "<images/", "<../../slides/robot-perspectives/images/"
            )
            markdown = markdown.replace(
                "](images/", "](../../slides/robot-perspectives/images/"
            )
            markdown = markdown.replace(
                'src="images/', 'src="../../slides/robot-perspectives/images/'
            )

        markdown = insert_after_title(markdown, lesson_actions(lesson))
        (destination / "index.md").write_text(markdown, encoding="utf-8")
        copy_lesson_assets(
            source,
            destination,
            skip_images=lesson["source"] == "C01_RobotPerspectives",
        )


def prepare_homepage() -> None:
    markdown = (ROOT / "README.md").read_text(encoding="utf-8")
    for lesson in LESSONS:
        source_url = f"{REPOSITORY_URL}/tree/main/{lesson['source']}"
        markdown = markdown.replace(source_url, f"{lesson['destination']}/")

    hero = """<div class="course-hero">
  <p class="course-eyebrow">ROB803 · SPRING 2026</p>
  <p class="course-lede">A hands-on path from robot description and simulation to whole-body control, reinforcement learning, and foundation models.</p>
  <div class="hero-actions">
    <a class="md-button md-button--primary" href="module-01/robot-perspectives/">Start the course</a>
    <a class="md-button" href="slides/robot-perspectives/">View lecture slides</a>
  </div>
</div>

<div class="course-grid">
  <a class="course-card" href="module-01/robot-perspectives/"><span>Module 01</span><strong>Robot Description</strong><small>Machine · Graph · Manifold</small></a>
  <a class="course-card" href="module-02/simulation-basics/"><span>Module 02</span><strong>Simulation & Interaction</strong><small>Retargeting · Control · Learning</small></a>
  <a class="course-card" href="module-03/humanoid-movement/"><span>Module 03</span><strong>Selected Topics</strong><small>Movement · Contact · Control</small></a>
  <a class="course-card" href="projects/final-project-example/"><span>Module 04</span><strong>Final Project</strong><small>Capture · Retarget · Evaluate</small></a>
</div>"""
    homepage = insert_after_title(markdown, hero)
    (DOCS / "index.md").write_text(homepage, encoding="utf-8")


def prepare_notebooks() -> None:
    notebooks = []
    for source_directory in (
        "01-KG-04-HumanoidBasics",
        "01-KG-05-HumanoidAdvanced",
    ):
        source = ROOT / source_directory
        for notebook in sorted(source.glob("*.ipynb")):
            destination = DOCS / "notebooks" / "files" / source_directory
            destination.mkdir(parents=True, exist_ok=True)
            shutil.copy2(notebook, destination / notebook.name)
            notebooks.append((source_directory, notebook.name))

    rows = []
    for source_directory, filename in notebooks:
        github_url = f"{REPOSITORY_URL}/blob/main/{source_directory}/{filename}"
        colab_url = (
            "https://colab.research.google.com/github/asHumanoids/2026Spring/"
            f"blob/main/{source_directory}/{filename}"
        )
        download_url = f"files/{source_directory}/{filename}"
        rows.append(
            f"| `{filename}` | [{source_directory}]({REPOSITORY_URL}/tree/main/{source_directory}) "
            f"| [Preview]({github_url}) · [Colab]({colab_url}) · [Download]({download_url}) |"
        )

    content = """# Jupyter Notebooks

Use these notebooks as companion exercises. GitHub provides a read-only preview, Colab opens an executable cloud copy, and Download preserves the original notebook file.

| Notebook | Course area | Open |
|---|---|---|
""" + "\n".join(rows) + "\n"
    destination = DOCS / "notebooks"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "index.md").write_text(content, encoding="utf-8")


def prepare_slides_index() -> None:
    content = """# Slide Decks

## Perspectives in Understanding Robots

The 47-slide presentation introduces four complementary views of a robot: a machine, a graph, a manifold, and an embodied agent.

<div class="page-actions">
  <a class="md-button md-button--primary" href="robot-perspectives/">Open full-screen presentation</a>
  <a class="md-button" href="https://github.com/asHumanoids/2026Spring/blob/main/C01_RobotPerspectives/slides.md">View slide source</a>
</div>

<div class="slides-frame">
  <iframe src="robot-perspectives/" title="Perspectives in Understanding Robots slide presentation" loading="lazy" allowfullscreen></iframe>
</div>

Use the arrow keys or swipe to move between slides. Press `f` for full screen, `o` for overview, and `p` for presenter view.
"""
    destination = DOCS / "slides"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "index.md").write_text(content, encoding="utf-8")


def copy_site_assets() -> None:
    source = ROOT / "site-assets"
    if source.is_dir():
        shutil.copytree(source, DOCS / "assets", dirs_exist_ok=True)


def main() -> None:
    reset_docs()
    prepare_homepage()
    prepare_lessons()
    prepare_notebooks()
    prepare_slides_index()
    copy_site_assets()
    markdown_pages = len(list(DOCS.rglob("*.md")))
    print(f"Prepared {markdown_pages} Markdown pages in {DOCS}")


if __name__ == "__main__":
    main()
