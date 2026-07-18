from pathlib import Path
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer
from reportlab.lib import colors

out = Path('TLS_Pie_Test_Guide.pdf')
styles = getSampleStyleSheet()
styles['Title'].fontName = 'Helvetica-Bold'
styles['Title'].fontSize = 18
styles['Title'].leading = 22
styles['Title'].spaceAfter = 12
styles['Heading2'].fontName = 'Helvetica-Bold'
styles['Heading2'].fontSize = 12
styles['Heading2'].leading = 14
styles['Heading2'].spaceAfter = 8
styles['Heading2'].textColor = colors.HexColor('#1f4e79')
styles['BodyText'].fontName = 'Helvetica'
styles['BodyText'].fontSize = 10
styles['BodyText'].leading = 12
styles['BodyText'].spaceAfter = 5
if 'Bullet' in styles: 
    bullet_style = styles['Bullet']
else:
    bullet_style = ParagraphStyle(name='Bullet', parent=styles['BodyText'], leftIndent=12, bulletIndent=0, spaceAfter=3)
    styles.add(bullet_style)

story = []
story.append(Paragraph('TLS_Pie troubleshooting guide', styles['Title']))
story.append(Paragraph('Printable fault-finding reference', styles['BodyText']))
story.append(Spacer(1, 8))

sections = [
    ('What changed from the original', [
        'Arduino record outputs moved from A4/A3 to D7/D8 to avoid display conflicts.',
        'Pi scripts now wait for Arduino trigger pulses on GPIO17 and GPIO27.',
        'The capture directory is now created before tcpdump starts.'
    ]),
    ('Quick wiring summary', [
        'Arduino D7 -> Pi GPIO17 (record start)',
        'Arduino D8 -> Pi GPIO27 (record stop)',
        'Arduino GND -> Pi GND',
        'A0/A1/A2/D2 -> buttons -> GND',
        'D3/D5/D6 -> stepper driver inputs',
        'Use a separate motor supply for the stepper driver motor power.'
    ]),
    ('Where to test', [
        '5V rail and GND on Arduino and Pi',
        'Button input pins A0, A1, A2, D2',
        'Trigger pins D7 and D8',
        'Stepper driver pins D3, D5, D6',
        'Pi GPIO17 and GPIO27'
    ]),
    ('Expected behavior', [
        'Display should start and show the menu.',
        'Button press should change the selection.',
        'Starting a scan should briefly pull D7 LOW.',
        'Stopping a scan should briefly pull D8 LOW.',
        'A .pcap file should be created by tcpdump.'
    ]),
    ('Quick checklist', [
        'Power present on Arduino and Pi',
        'Common ground connected',
        'Buttons pull the pins low when pressed',
        'Stepper driver receives step pulses',
        'Pi receives the Arduino trigger pulses',
        'Capture folder exists and tcpdump runs'
    ])
]

for title, bullets in sections:
    story.append(Paragraph(title, styles['Heading2']))
    for item in bullets:
        story.append(Paragraph('- ' + item, styles['Bullet']))
    story.append(Spacer(1, 6))

story.append(Paragraph('If anything fails, start at the top and verify power, ground, then buttons, then trigger pulses, then the stepper driver and Pi capture.', styles['BodyText']))

SimpleDocTemplate(str(out), pagesize=letter, rightMargin=36, leftMargin=36, topMargin=36, bottomMargin=36).build(story)
print(out.resolve())
