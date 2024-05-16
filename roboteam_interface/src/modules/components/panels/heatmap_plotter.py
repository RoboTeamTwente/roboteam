import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt
import sys

#file = input("File route?...\n")
#file = "heatmap.csv"

#data = np.genfromtxt(file, delimiter=',')
data = sys.argv[1]

with open("test_text.txt", 'w') as archivo:
    # Escribe el valor de la variable en el archivo
    archivo.write(str(data))

#numpy_array = np.array(data)
numpy_array = np.genfromtxt(data, delimiter=',')

# Crea un heatmap utilizando matplotlib
plt.imshow(numpy_array, cmap='RdBu', interpolation='nearest')

# Agrega barra de colores
plt.colorbar()

# Añade etiquetas de ejes (puedes personalizarlas según tus datos)
plt.xlabel('X')
plt.ylabel('Y')

# Añade un título (puedes personalizarlo según tus datos)
plt.title('Heatmap')

plt.savefig("heatmap_from_interface.png")