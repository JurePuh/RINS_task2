"""Inspection report PDF generation."""

import os
from datetime import datetime

from fpdf import FPDF

from task2.movement.models import (
    AnomalyTask,
    CountRingsTask,
    InspectBarrelsTask,
    Task,
)


ROBOT_NAME = "R2D2"
_IMG_W = 70  # mm
_IMG_PAIR_W = 60


def build_report(tasks: list[tuple[Task, str]], out_dir: str, logger) -> str:
    """Build the inspection PDF.

    `tasks` is a list of (task, title) pairs in the order they should appear.
    `title` is the human-readable section title (e.g. "Anomaly Detection (Red)").
    Returns the output path.
    """
    logger.info(
        f"build_report: starting; out_dir={out_dir!r} task_count={len(tasks)} "
        f"titles={[title for _, title in tasks]}"
    )
    logger.debug(
        f"build_report: task inputs = "
        f"{[(title, type(t).__name__, repr(t)) for t, title in tasks]}"
    )
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(out_dir, f"report_{ts}.pdf")

    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()

    pdf.set_font("Helvetica", "B", 18)
    pdf.cell(0, 12, "Inspection report", new_x="LMARGIN", new_y="NEXT")
    pdf.set_font("Helvetica", "", 11)
    pdf.cell(0, 6, f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
             new_x="LMARGIN", new_y="NEXT")
    pdf.cell(0, 6, f"Robot: {ROBOT_NAME}", new_x="LMARGIN", new_y="NEXT")
    pdf.ln(4)

    for task, title in tasks:
        _render_task(pdf, task, title, logger)

    pdf.output(out_path)
    logger.info(f"build_report: wrote PDF to {out_path}")
    return out_path


def _render_task(pdf: FPDF, task: Task, title: str, logger) -> None:
    logger.debug(
        f"_render_task: title={title!r} type={type(task).__name__} "
        f"requesters={[p.name or p.face_id for p in task.requesters]} "
        f"task_repr={task!r}"
    )
    pdf.ln(2)
    pdf.set_font("Helvetica", "B", 14)
    pdf.cell(0, 8, f"Task: {title}", new_x="LMARGIN", new_y="NEXT")

    pdf.set_font("Helvetica", "", 11)
    requested_by = ", ".join(p.name or p.face_id for p in task.requesters) or "—"
    pdf.cell(0, 6, f"Requested by: {requested_by}", new_x="LMARGIN", new_y="NEXT")
    pdf.cell(0, 6, "Results:", new_x="LMARGIN", new_y="NEXT")

    if isinstance(task, CountRingsTask):
        _render_rings(pdf, task)
    elif isinstance(task, InspectBarrelsTask):
        _render_barrels(pdf, task, logger)
    elif isinstance(task, AnomalyTask):
        _render_anomaly(pdf, task, logger)
    else:
        pdf.cell(0, 6, f"  (unknown task type {type(task).__name__})",
                 new_x="LMARGIN", new_y="NEXT")
    pdf.ln(2)


def _render_rings(pdf: FPDF, task: CountRingsTask) -> None:
    pdf.cell(0, 6, f"  Total rings: {len(task.rings)}", new_x="LMARGIN", new_y="NEXT")
    pdf.cell(0, 6, "  By color:", new_x="LMARGIN", new_y="NEXT")
    by_color = task.rings_by_color
    for color, rings in by_color.items():
        pdf.cell(0, 6, f"    {color.capitalize()}: {len(rings)}",
                 new_x="LMARGIN", new_y="NEXT")


def _render_barrels(pdf: FPDF, task: InspectBarrelsTask, logger) -> None:
    pdf.cell(0, 6, f"  Total number of barrels inspected: {len(task.barrels)}",
             new_x="LMARGIN", new_y="NEXT")
    pdf.ln(1)

    headings = ("Barrel ID", "Colour", "Orientation", "Leak detected")
    rows = [
        (
            b.id,
            b.color,
            "Horizontal" if b.horizontal else "Vertical",
            "Yes" if b.leaking else ("No" if b.leaking is False else "Unknown"),
        )
        for b in task.barrels
    ]
    _draw_table(pdf, headings, rows)

    leaking = task.leaking_barrels
    if leaking:
        pdf.ln(3)
        pdf.set_font("Helvetica", "B", 11)
        pdf.cell(0, 6, "Leaking barrels:", new_x="LMARGIN", new_y="NEXT")
        pdf.set_font("Helvetica", "", 10)
        for b in leaking:
            _embed_image(pdf, b.image_path, f"Barrel {b.id}", _IMG_W, logger)


def _render_anomaly(pdf: FPDF, task: AnomalyTask, logger) -> None:
    pdf.cell(0, 6, f"  Total number of tiles inspected: {len(task.tiles)}",
             new_x="LMARGIN", new_y="NEXT")
    pdf.cell(0, 6, f"  Number of broken: {len(task.broken_tiles)}",
             new_x="LMARGIN", new_y="NEXT")
    pdf.ln(1)

    headings = ("Tile ID", "Status")
    rows = [
        (
            str(t.index),
            "NOK" if t.broken else ("OK" if t.broken is False else "Unknown"),
        )
        for t in task.tiles
    ]
    _draw_table(pdf, headings, rows)

    broken = task.broken_tiles
    if broken:
        pdf.ln(3)
        pdf.set_font("Helvetica", "B", 11)
        pdf.cell(0, 6, "Broken tiles:", new_x="LMARGIN", new_y="NEXT")
        pdf.set_font("Helvetica", "", 10)
        for t in broken:
            _embed_tile_pair(pdf, t.image_path, t.mask_path, f"Tile {t.index}", logger)


def _draw_table(pdf: FPDF, headings: tuple, rows: list) -> None:
    pdf.set_font("Helvetica", "", 10)
    with pdf.table() as table:
        head = table.row()
        for h in headings:
            head.cell(h)
        for row in rows:
            r = table.row()
            for cell in row:
                r.cell(str(cell))


def _embed_image(pdf: FPDF, path: str | None, caption: str, width: float, logger) -> None:
    if not path or not os.path.exists(path):
        logger.warn(f"report: missing image for {caption} (path={path!r}); skipping")
        pdf.cell(0, 6, f"  {caption}: (no image)", new_x="LMARGIN", new_y="NEXT")
        return
    pdf.cell(0, 5, caption, new_x="LMARGIN", new_y="NEXT")
    pdf.image(path, w=width)
    pdf.ln(2)


def _embed_tile_pair(pdf: FPDF, img: str | None, mask: str | None,
                     caption: str, logger) -> None:
    pdf.cell(0, 5, caption, new_x="LMARGIN", new_y="NEXT")
    y = pdf.get_y()
    x = pdf.get_x()
    placed = False
    if img and os.path.exists(img):
        pdf.image(img, x=x, y=y, w=_IMG_PAIR_W)
        placed = True
    else:
        logger.warn(f"report: missing tile image for {caption} (path={img!r}, mask={mask!r})")
    if mask and os.path.exists(mask):
        pdf.image(mask, x=x + _IMG_PAIR_W + 5, y=y, w=_IMG_PAIR_W)
        placed = True
    else:
        logger.warn(f"report: missing tile mask for {caption} (img={img!r}, mask={mask!r})")
    if placed:
        pdf.ln(_IMG_PAIR_W * 0.75 + 3)
    else:
        pdf.cell(0, 6, "  (no images)", new_x="LMARGIN", new_y="NEXT")
