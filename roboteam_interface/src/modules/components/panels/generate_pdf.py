from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, PageBreak, Frame
import webbrowser
import socket
from flask import Flask, request, jsonify
import numpy as np
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import time
from datetime import datetime
from PIL import Image as PILImage

def generate_heatmap(data_array): 

    plt.imshow(data_array, cmap='RdBu', interpolation='nearest')
    plt.savefig("heatmap_from_interface.png")

def generate_pdf(a, b, c, d, e, header_image_path, center_image_path, filename='performance_report.pdf'):
    
    current_time = time.time()
    date = datetime.fromtimestamp(current_time)
    formated_date = date.strftime("%Y-%m-%d %H:%M:%S")

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
        leftIndent=40,
        spaceBefore=6,
        spaceAfter=6,
        textColor=colors.black,
        fontName='Helvetica-Bold',
        fontSize = 12
    )

    aligh_left = ParagraphStyle(
        'CustomStyle',
        parent=styles['BodyText'],
        leftIndent=450,
        spaceBefore=6,
        spaceAfter=6,
        textColor=colors.black,
        fontName='Helvetica-Bold',
        fontSize = 10
    )
    
    # Crear una lista de elementos para agregar al PDF
    content = []

    # Agregar el encabezado con la imagen
    header_image = PILImage.open(header_image_path)
    header_image_width, header_image_height = int(pdf_document.width*1.2), 100
    header_image = header_image.resize((header_image_width, header_image_height))
    header_image_path = 'robo_header.jpeg'  # Nombre del archivo de imagen de encabezado
    header_image.save(header_image_path)

    content.append(Image(header_image_path, width=pdf_document.width, height=header_image_height))
    content.append(Spacer(1, 20))  

    # Agregar los puntos con texto en negrita
    content.append(Paragraph(f"{formated_date}", aligh_left))
    content.append(Paragraph("<b>PERFORMANCE REPORT</b>", header_style))
    content.append(Spacer(1, 10))
    content.append(Paragraph(f"<b>- Total time:</b> {a}", custom_style))
    content.append(Paragraph(f"<b>- Possession:</b> {b}", custom_style))
    content.append(Paragraph(f"<b>- Number of offensive actions:</b> {c}", custom_style))
    content.append(Paragraph(f"<b>- Number of defensive actions:</b> {d}", custom_style))
    content.append(Paragraph(f"<b>- Number of keeper actions:</b> {e}", custom_style))

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

# Create Flask server
# /home/juanjo20/roboteam/roboteam_interface/src/modules/components/panels
# flask --app generate_pdf run
    
app = Flask(__name__)

@app.route('/')
def hello_world():
    return "<p>Hello, World!</p>"

# Permitir solicitudes CORS
@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Methods', 'POST')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type')
    return response

@app.route('/process_data', methods=['POST'])
def manejar_post():
    try:
        # Obtener los datos JSON del cuerpo de la solicitud
        datos = request.get_json()
        metrics = datos.get('metrics')

        print("Msg received successfully")

        general_metrics = metrics[0]
        heatmap_metrics = metrics[1]

        heatmap_metrics = np.array(heatmap_metrics)
        heatmap_metrics = heatmap_metrics.reshape(-1, 10).tolist()  

        generate_heatmap(heatmap_metrics)
        print("Heatmap generated successfully")

        generate_pdf(general_metrics[0], 
                     general_metrics[1], 
                     general_metrics[2], 
                     general_metrics[3], 
                     general_metrics[4], 
                     "robo_header.jpeg",
                     "heatmap_from_interface.png", 
                     "performance_report.pdf")
        print("Report generated successfully!!!")


        # Devolver una respuesta JSON
        return jsonify({'mensaje': 'Process finished sucessfully'})
    
    except Exception as e:
        # Manejar cualquier error que pueda ocurrir durante el procesamiento
        print("Msg NOT processed successfully")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    # Ejecutar el servidor en el puerto 5000 (puedes cambiarlo según tus necesidades)
    app.run(port=50000)


# Ejemplo de uso con valores a, b y c, y rutas de las imágenes
'''a_value = int(60)
b_value = int(75)
c_value = int(85)
header_image_path = 'header.png'  # Reemplaza con la ruta de tu imagen de encabezado
center_image_path = 'heat.png'  # Reemplaza con la ruta de tu imagen centrada'''

#generate_pdf(a_value, b_value, c_value, header_image_path, center_image_path)
