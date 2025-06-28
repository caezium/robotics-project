from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO("yolo11n.pt")
    results = model.train(data=r"C:\Users\yunch\OneDrive\Desktop\robotics-project\data\data.yaml", 
                          epochs=100, imgsz=512, batch=16)