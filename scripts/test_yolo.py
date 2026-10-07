"""
from ultralytics import YOLO


model = YOLO("../../yolo_env/artefacts/yolo26n.onnx")

results = model.predict("https://ultralytics.com/images/bus.jpg", imgsz=320)

print(results)

"""
import time
import sys

import cv2
import onnxruntime as ort
import numpy as np

image_path = "/home/lu/yolo_env/artefacts/bus.jpg"

session = ort.InferenceSession("yolo26n.onnx")

print(session.get_inputs())
input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name

print("Input: ", input_name, session.get_inputs()[0].shape)
print("Output: ", output_name, session.get_outputs()[0].shape)

# Preprocess here.
source_image = cv2.imread(image_path)
resized_image = cv2.resize(source_image, (640, 640))
print("Original / Resized: ", source_image.shape, " / ", resized_image.shape)

rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
normalized_image = (rgb_image / 255.0).astype(np.float32)
print("Type Min Max: ", normalized_image.dtype, normalized_image.min(), normalized_image.max())

chw = normalized_image.transpose(2, 0, 1)
input_tensor = np.expand_dims(chw, axis=0)
print("Transformed shape: ", input_tensor.shape)


 

start = time.time()
outputs = session.run([output_name], {input_name: input_tensor})
end = time.time()

interval = end - start
detections = outputs[0]

detections = np.squeeze(detections)
mask = detections[:, 4] > 0.25
kept = detections[mask] 
print("kept shape: ", kept.shape)
print("process time: ", interval * 1000, " ms")


canvas = resized_image.copy()

for row in kept:
    x1 = int(row[0])
    y1 = int(row[1])
    x2 = int(row[2])
    y2 = int(row[3])
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv2.imwrite("/home/lu/yolo_env/artefacts/output.jpg", canvas)
