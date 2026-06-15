import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def draw_lane_lines(image, left_fit, right_fit):
    """
    Dibuja las líneas extrapoladas sobre la imagen.
    """
    # Crear una imagen negra del mismo tamaño para dibujar las líneas
    line_image = np.zeros_like(image)
    height = image.shape[0]

    # Definir desde dónde y hasta dónde en el eje Y queremos dibujar las líneas
    # y1 es la parte de abajo de la imagen.
    # y2 es hasta dónde queremos que llegue la línea hacia arriba (ej. 60% de la altura).
    y1 = height
    y2 = int(height * 0.6) 

    # Dibujar línea izquierda (Azul)
    if left_fit is not None:
        # Calcular los puntos X usando la ecuación de la línea evaluada en y1 y y2
        left_x1 = int(np.polyval(left_fit, y1))
        left_x2 = int(np.polyval(left_fit, y2))
        cv2.line(line_image, (left_x1, y1), (left_x2, y2), (255, 0, 0), 10)

    # Dibujar línea derecha (Roja)
    if right_fit is not None:
        right_x1 = int(np.polyval(right_fit, y1))
        right_x2 = int(np.polyval(right_fit, y2))
        cv2.line(line_image, (right_x1, y1), (right_x2, y2), (0, 0, 255), 10)

    # Superponer las líneas sobre la imagen original
    # (0.8 es la opacidad de la imagen de fondo, 1.0 la opacidad de las líneas)
    combined_image = cv2.addWeighted(image, 0.8, line_image, 1.0, 0.0)
    
    return combined_image

def compute_steering_angle(lines, width, height):
    """Computes the steering angle based on detected lane lines from HoughLinesP.
    
        Args:        
            lines: Detected lane lines from HoughLinesP.
            width: Width of the input image.
            height: Height of the input image.
        Returns:
            Steering angle in radians, where 0 is straight, negative is left, and positive is right.
            
            """
    if lines is None:
        return 0, None, None  # No lanes detected, keep steering angle straight

    left_lane = []
    right_lane = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
        if slope < -0.5:  # Left lane
            left_lane.append(line)
        elif slope > 0.5:  # Right lane
            right_lane.append(line)

    if not left_lane and not right_lane:
        return 0, None, None  # No lanes detected, keep steering angle straight

    # Average the positions of the left and right lanes
    left_x = [line[0][0] for line in left_lane] + [line[0][2] for line in left_lane]
    left_y = [line[0][1] for line in left_lane] + [line[0][3] for line in left_lane]
    right_x = [line[0][0] for line in right_lane] + [line[0][2] for line in right_lane]
    right_y = [line[0][1] for line in right_lane] + [line[0][3] for line in right_lane]

    if left_x and left_y:
        left_fit = np.polyfit(left_y, left_x, 1)
        left_line_x = np.polyval(left_fit, height)
    else:
        left_line_x = width // 2
        left_fit = None

    if right_x and right_y:
        right_fit = np.polyfit(right_y, right_x, 1)
        right_line_x = np.polyval(right_fit, height)
    else:
        right_line_x = width // 2
        right_fit = None

    lane_center = (left_line_x + right_line_x) / 2
    frame_center = width / 2
    steering_angle = (lane_center - frame_center) / frame_center * 0.3

    return steering_angle, left_fit, right_fit

ema_angle = None #agregar EMA
alpha = 0.2 #agregar EMA

while True:
    ret, frame_completo = cap.read()
    if not ret: break
    LENTE_A_USAR = 'DERECHO'
    alto, ancho_total, _ = frame_completo.shape
    mitad = ancho_total // 2
    
    # Recortamos la imagen dependiendo de qué lente elegiste arriba
    if LENTE_A_USAR == 'IZQUIERDO':
        cv2_image = frame_completo[:, :mitad]
    else:
        cv2_image = frame_completo[:, mitad:]

    # Convert to HLS
    hls = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HLS)

    # Define range for white
    lower_white = np.array([0, 190, 0])   # Adjust '200' to tune sensitivity
    upper_white = np.array([180, 255, 50]) # Adjust '50' to allow more/less color
    
    # Create and apply mask
    mask = cv2.inRange(hls, lower_white, upper_white)
    result = cv2.bitwise_and(cv2_image, cv2_image, mask=mask)

    canny = cv2.Canny(result, 50, 150)

    alto_img, ancho_img = canny.shape

    # ROI dinámica
    vertices = np.array([[
        (0, alto_img),                        
        (ancho_img, alto_img),                    
        (int(ancho_img * 0.65), int(alto_img * 0.73)), 
        (int(ancho_img * 0.3), int(alto_img * 0.73))  
    ]], dtype=np.int32)
    
    mask = np.zeros_like(canny)
    cv2.fillPoly(mask, vertices, 255)
    roi_canny = cv2.bitwise_and(canny, mask)

    roi_canny2=cv2.resize(roi_canny, (int(ancho_img*0.5), int(alto_img*0.5)))

    lineas = cv2.HoughLinesP(roi_canny, rho=1, theta=np.pi/180, threshold=40, 
                                     minLineLength=105, maxLineGap=100)
    
    img_debug = cv2_image.copy() 

    steering_angle, left, right=compute_steering_angle(lineas, ancho_img, alto_img)

    #print(left)

    img=draw_lane_lines(img_debug, left, right)

    if ema_angle is None: #agregar EMA
        ema_angle = steering_angle #agregar EMA
    else: #agregar EMA
        ema_angle = (alpha * steering_angle) + ((1.0 - alpha) * ema_angle) #agregar EMA

    print(f"Raw: {np.rad2deg(steering_angle):.2f}° | EMA: {np.rad2deg(ema_angle):.2f}°") #agregar EMA
   
    # if lineas is not None:
    #             for linea in lineas:
    #                 x1, y1, x2, y2 = linea[0]
                    
    #                 # Dibujar líneas en la imagen de debug
                   
    #                 cv2.line(img_debug, (x1, y1), (x2, y2), (0, 255, 0), 3)

    img_debugr=cv2.resize(img, (int(ancho_img*0.5), int(alto_img*0.5)))

    cv2.imshow('White Extraction', img_debugr)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()