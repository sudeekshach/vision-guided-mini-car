# 🤖 ROSMASTER X3 Mini Autonomous Car – Line Following & Traffic Sign Detection

## 📘 Project Overview

This project showcases a simplified **autonomous driving system** using the **Yahboom ROSMASTER X3** mobile robot. The robot performs:

- **Blue path following** using HSV color filtering
- **Traffic sign detection and response** using a custom-trained **YOLOv5 model**

The system is implemented in **ROS2** and deployed on a **Jetson Nano**, leveraging real-time camera input, object detection, and PID-based motion control.

---

## 🎯 Project Goals

- Build a mini robot that can follow a colored path using its onboard camera
- Detect and interpret four predefined traffic signs
- Respond to signs and stay on path in real-world conditions

---

## 🎥 Demo Video

▶️ [Watch Demo on YouTube](https://youtu.be/37Uw26qA_U4) 

---

## 🧠 Technologies Used

- **Yahboom ROSMASTER X3** with Mecanum wheels
- **Jetson Nano** for onboard compute
- **ROS2 Foxy Fitzroy**
- **Python 3**, **OpenCV**
- **YOLOv5** (custom-trained model in `.engine` format)
- **HSV filtering** for real-time color tracking
- **PID controller** for smooth directional control

---

## 🧩 System Architecture

### ➤ Hardware
- Astra Pro depth camera
- 333 RPM geared motors
- Mecanum wheels for omni-directional movement
- 12V battery with onboard Jetson Nano

### ➤ Software Nodes
- Camera stream acquisition (USB or CSI)
- Color mask generation using HSV
- PID error calculation based on path offset
- YOLOv5 object detection on live video stream
- Motor control based on navigation and detection feedback

---

## 🔧 Key Features

### 🚗 Path Following via HSV

- The camera detects a **blue path** using real-time HSV color filtering.
- A PID controller computes the offset and adjusts velocity commands.
- The system allows **HSV calibration** for different environments.

💡 *Reference: Based on Yahboom’s documentation for HSV color filtering and line following.*

### 🧠 Pseudo Code: Line Following

```pseudo
initialize_camera()
initialize_pid_controller()

while robot_is_active:
    frame = get_camera_frame()
    blue_mask = apply_HSV_filter(frame)
    error = calculate_offset_from_center(blue_mask)
    control_signal = PID.update(error)
    send_motor_commands(control_signal)
```

---

### 🛑 Traffic Sign Detection using YOLOv5

- A custom YOLOv5 model (`.engine`) is trained to detect 4 signs.
- Camera feed is processed directly on the Jetson Nano.
- The system recognizes signs like **STOP** and adjusts robot behavior.

🧠 Best practices followed:
- Enabled **MAXN power mode** (`nvpmodel -m 0/2`)
- Enabled **jetson_clocks** for full CPU/GPU utilization
- Used optimized ONNX/TensorRT-based inference pipeline

💡 *Reference: Adapted from Yahboom’s official YOLOv5 inference guide for Jetson platforms.*

### 🧠 Pseudo Code: Sign detection and response

```pseudo
load_yolo_model("best.engine")
start_camera_stream()

while camera_is_open:
    frame = capture_frame()
    detections = model.infer(frame)

    if "stop_sign" in detections:
        halt_movement()
    elif "go_straight" in detections:
        maintain_velocity()
    elif "turn_left" in detections:
        execute_left_turn()
    elif "turn_right" in detections:
        execute_right_turn()

    display_frame_with_detections()

```
---

## 📊 Performance

| Feature               | Outcome                              |
|-----------------------|---------------------------------------|
| Line following        | Smooth and reliable in good lighting |
| Sign detection range  | ~15–20 inches with single sign focus |
| Detection speed       | 80–95% accuracy with some lag (~15–20s) |
| Deployment platform   | Fully onboard using Jetson Nano      |

---

## 🧪 Limitations & Improvements

### ✅ Working
- YOLOv5 inference on Jetson
- HSV tracking and PID control
- Single-object detection at close range

### ⚠️ Issues
- Sign recognition lag on low-power settings
- Difficulty in detecting signs when multiple present
- Requires good lighting for color filtering

### 🚀 Future Enhancements
- Add multi-object detection support
- Reduce latency with model optimization
- Integrate obstacle avoidance
- Expand to traffic lights and intersections

---

## 📚 References

- **Yahboom ROSMASTER X3 – Official Product Page**
- **HSV Line Following Guide (Section 5.1–5.2)**
- **YOLOv5 Jetson Nano Deployment Tutorial (CLI + Python Inference)**
- [**Ultralytics YOLOv5 Repository**](https://github.com/ultralytics/yolov5)

---

## 👥 Team Members

- **Satya** (Team Lead)  
- **Sudeeksha**  
- **Dheeraj**  
- **Harsh**  
- **Jay**  
- **Raunakjit**


---

## 🛡 License

This project is shared for academic and educational use. Please credit the team and linked references if you adapt or reuse the work.

---

## 📬 Contact

**Maintainer**: [Your Name]  
📧 Email: [sudeekshach@gmail.com]  
🔗 LinkedIn: [https://www.linkedin.com/in/sudeeksha-chagarlamudi/](https://www.linkedin.com/in/sudeeksha-chagarlamudi/)


