from pathlib import Path
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer

root = Path('Raspberry Pie4')
files = [
    ('TLS_Pie_Pi_Setup_Checklist.md', 'TLS_Pie_Pi_Setup_Checklist.pdf'),
    ('TLS_Pie_Pi_Setup_Guide.md', 'TLS_Pie_Pi_Setup_Guide.pdf'),
]

for src_name, out_name in files:
    src = (root / src_name).read_text(encoding='utf-8')
    styles = getSampleStyleSheet()
    h1 = styles['Heading1']
    h2 = styles['Heading2']
    normal = styles['Normal']
    h2.fontSize = 12
    h2.leading = 14
    normal.fontSize = 10
    normal.leading = 12

    story = []
    for line in src.splitlines():
        if not line.strip():
            story.append(Spacer(1, 6))
        elif line.startswith('# '):
            story.append(Paragraph(line[2:], h1))
        elif line.startswith('## '):
            story.append(Paragraph(line[3:], h2))
        elif line.startswith('```'):
            continue
        elif line.startswith('- ') or line.startswith('* '):
            story.append(Paragraph(line[2:], normal))
        else:
            story.append(Paragraph(line, normal))

    doc = SimpleDocTemplate(str(root / out_name), pagesize=letter, rightMargin=54, leftMargin=54, topMargin=54, bottomMargin=54)
    doc.build(story)

print('pdfs-generated')
