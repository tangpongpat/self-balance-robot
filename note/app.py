from flask import Flask, render_template, request, jsonify
import requests  # ใช้สำหรับส่ง HTTP requests

app = Flask(__name__)

# กำหนด IP ของ ESP32
ESP32_IP = 'http://192.168.1.100'  # เปลี่ยนเป็น IP ของ ESP32 ของคุณ

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        setpoint = request.form.get('setpoint')
        Kp = request.form.get('Kp')
        Ki = request.form.get('Ki')
        Kd = request.form.get('Kd')
        deadband = request.form.get('deadband')

        # ส่งข้อมูลไปยัง ESP32
        requests.get(f'{ESP32_IP}/setpoint?value={setpoint}')
        requests.get(f'{ESP32_IP}/Kp?value={Kp}')
        requests.get(f'{ESP32_IP}/Ki?value={Ki}')
        requests.get(f'{ESP32_IP}/Kd?value={Kd}')
        requests.get(f'{ESP32_IP}/deadband?value={deadband}')
        requests.get(f'{ESP32_IP}/NEW_CALIBRATE')  # ส่งคำสั่งคาลิเบตใหม่

        return 'Parameters updated successfully!'

    return render_template('index.html')

@app.route('/data', methods=['GET'])
def get_data():
    # อ่านข้อมูลจาก ESP32
    response = requests.get(f'{ESP32_IP}/data')  # ส่งคำขอไปยัง ESP32 เพื่อรับข้อมูล
    if response.status_code == 200:
        return jsonify(response.json())  # ส่งข้อมูลกลับเป็น JSON

    return jsonify({'angle_data': []})  # ถ้าไม่มีข้อมูล

if __name__ == '__main__':
    app.run(debug=True)