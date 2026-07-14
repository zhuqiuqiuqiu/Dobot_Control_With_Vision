from ultralytics import YOLO

def train_yolo():

    # 1. 加载模型（YOLOv11 nano / small / medium）
    model = YOLO("yolo11n.pt")  # n=轻量 s/m/l/x更强

    # 2. 开始训练
    model.train(
        data="dataset/data.yaml",
        epochs=60,

        imgsz=640,
        batch=4,

        device=0,

        workers=0,
        cache=False,

        augment=True,

        mosaic=1.0,
        mixup=0.3,
        copy_paste=0.3,

        degrees=20,
        scale=0.8,
        translate=0.2,

        hsv_h=0.03,
        hsv_s=0.9,
        hsv_v=0.6,
    )

if __name__ == "__main__":
    train_yolo()