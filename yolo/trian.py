from ultralytics import YOLO

model = YOLO("yolo26n.pt")
results = model.val()
results = model("https://ultralytics.com/images/bus.jpg")