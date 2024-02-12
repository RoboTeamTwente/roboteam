from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, PageBreak, Frame
import webbrowser
import socket

from PIL import Image as PILImage

def generate_pdf(a, b, c, header_image_path, center_image_path, filename='performance_report.pdf'):
    # Crear el objeto SimpleDocTemplate para el PDF
    pdf_document = SimpleDocTemplate(filename, pagesize=letter, topMargin=0, leftMargin = 0, rightMargin=0)

    # Establecer el estilo del párrafo
    styles = getSampleStyleSheet()
    header_style = ParagraphStyle(
        'CustomStyle',
        parent=styles['BodyText'],
        leftIndent=40,
        spaceAfter=12,
        textColor=colors.black,
        fontName='Helvetica-Bold',
        fontSize = 24
    )

    custom_style = ParagraphStyle(
        'CustomStyle',
        parent=styles['BodyText'],
        leftIndent=50,
        spaceBefore=6,
        spaceAfter=6,
        textColor=colors.black,
        fontName='Helvetica-Bold',
        fontSize = 12
    )
    
    # Crear una lista de elementos para agregar al PDF
    content = []

    # Agregar el encabezado con la imagen
    header_image = PILImage.open(header_image_path)
    header_image_width, header_image_height = int(pdf_document.width*1.2), 100
    header_image = header_image.resize((header_image_width, header_image_height))
    header_image_path = 'header_image.png'  # Nombre del archivo de imagen de encabezado
    header_image.save(header_image_path)

    content.append(Image(header_image_path, width=pdf_document.width, height=header_image_height))
    content.append(Spacer(1, 20))  # Espaciador para separar el encabezado del contenido principal

    # Agregar los puntos con texto en negrita
    content.append(Paragraph("<b>PERFORMANCE REPORT:</b>", header_style))
    content.append(Spacer(1, 10))  # Espaciador para separar el encabezado del contenido principal
    content.append(Paragraph(f"<b>- Possession:</b> {a}", custom_style))
    content.append(Paragraph(f"<b>- Attacking:</b> {b}", custom_style))
    content.append(Paragraph(f"<b>- Defending:</b> {c}", custom_style))

    # Agregar espacio antes de la imagen centrada
    content.append(Spacer(1, 30))

    # Agregar la imagen centrada
    center_image = PILImage.open(center_image_path)
    center_image_width, center_image_height = 200, 200
    center_image = center_image.resize((center_image_width, center_image_height))
    center_image_path = 'center_image.png'  # Nombre del archivo de imagen centrada
    center_image.save(center_image_path)

    content.append(Image(center_image_path, width=center_image_width, height=center_image_height, hAlign='CENTER'))

    # Construir el PDF
    pdf_document.build(content)

    webbrowser.open(filename)

# Create socket (SERVER)
HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        #print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            conn.sendall(data)

# Ejemplo de uso con valores a, b y c, y rutas de las imágenes
'''a_value = int(60)
b_value = int(75)
c_value = int(85)
header_image_path = 'header.png'  # Reemplaza con la ruta de tu imagen de encabezado
center_image_path = 'heat.png'  # Reemplaza con la ruta de tu imagen centrada'''

#generate_pdf(a_value, b_value, c_value, header_image_path, center_image_path)
