"""Render BENCH_TEST_CHECKLIST.md to BENCH_TEST_CHECKLIST.pdf.

Run from anywhere:
    python make_bench_test_checklist_pdf.py
"""
from pathlib import Path

from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Preformatted

here = Path(__file__).resolve().parent
src = here / "BENCH_TEST_CHECKLIST.md"
out = here / "BENCH_TEST_CHECKLIST.pdf"

styles = getSampleStyleSheet()
h1 = styles["Title"]
h1.fontSize = 16
h1.leading = 20
h1.spaceAfter = 10

h2 = ParagraphStyle(
    name="H2",
    parent=styles["Heading2"],
    fontSize = 12,
    leading = 14,
    spaceBefore = 10,
    spaceAfter = 6,
    textColor = colors.HexColor("#1f4e79"),
)

normal = styles["BodyText"]
normal.fontSize = 10
normal.leading = 13
normal.spaceAfter = 3

bullet = ParagraphStyle(
    name="BulletItem",
    parent=normal,
    leftIndent = 14,
    bulletIndent = 0,
    spaceAfter = 2,
)

code = ParagraphStyle(
    name="Code",
    parent=normal,
    fontName = "Courier",
    fontSize = 9,
    leading = 11,
    leftIndent = 14,
    backColor = colors.HexColor("#f2f2f2"),
    spaceAfter = 6,
)

text = src.read_text(encoding="utf-8")
story = []
in_code = False
code_lines = []

for line in text.splitlines():
    stripped = line.strip()

    if stripped.startswith("```"):
        if in_code:
            story.append(Preformatted("\n".join(code_lines), code))
            code_lines = []
            in_code = False
        else:
            in_code = True
        continue

    if in_code:
        code_lines.append(line)
        continue

    if not stripped:
        story.append(Spacer(1, 6))
    elif stripped.startswith("# "):
        story.append(Paragraph(stripped[2:], h1))
    elif stripped.startswith("## "):
        story.append(Paragraph(stripped[3:], h2))
    elif stripped.startswith("- "):
        story.append(Paragraph("- " + stripped[2:], bullet))
    else:
        story.append(Paragraph(stripped, normal))

doc = SimpleDocTemplate(
    str(out),
    pagesize=letter,
    rightMargin=48,
    leftMargin=48,
    topMargin=48,
    bottomMargin=48,
    title="TLS Pie Bench Test Checklist",
)
doc.build(story)
print(out)
