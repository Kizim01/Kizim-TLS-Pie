from pathlib import Path
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak
from reportlab.lib.styles import getSampleStyleSheet

files = [
    'FUNDING_CONCEPT_NOTE.md',
    'PITCH_DECK_OUTLINE.md',
    'COMPANY_LAUNCH_CHECKLIST.md',
]

out = Path('Kizim_Robotics_Funding_Packet.pdf')
styles = getSampleStyleSheet()
story = []

for path_str in files:
    path = Path(path_str)
    text = path.read_text(encoding='utf-8')
    lines = text.splitlines()
    for line in lines:
        if line.startswith('# '):
            story.append(Paragraph(line[2:], styles['Title']))
        elif line.startswith('## '):
            story.append(Paragraph(line[3:], styles['Heading2']))
        elif line.startswith('### '):
            story.append(Paragraph(line[4:], styles['Heading3']))
        elif line.strip().startswith('- '):
            story.append(Paragraph(line[2:], styles['BodyText']))
        elif line.strip():
            story.append(Paragraph(line, styles['BodyText']))
        else:
            story.append(Spacer(1, 6))
    story.append(PageBreak())

if story and isinstance(story[-1], PageBreak):
    story.pop()

doc = SimpleDocTemplate(str(out), pagesize=letter, title='Kizim Robotics Funding Packet')
doc.build(story)
print(out.resolve())
