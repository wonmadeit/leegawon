# app.py
from flask import Flask, request, jsonify, send_file
from inference import run_inference
from PIL import Image
import io
import numpy as np
import cv2

app = Flask(__name__)

@app.route("/detect", methods=["POST"])
def detect_lane():
    if 'image' not in request.files:
        return jsonify({"error": "No image uploaded"}), 400

    image_file = request.files['image']
    image = Image.open(image_file.stream).convert("RGB")
    mask = run_inference(image)

    # 결과를 이미지로 반환
    _, buffer = cv2.imencode(".png", mask)
    return send_file(
        io.BytesIO(buffer.tobytes()),
        mimetype='image/png'
    )

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
