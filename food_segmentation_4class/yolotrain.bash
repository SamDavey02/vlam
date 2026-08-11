yolo segment train \
    model=yolo11n-seg.pt \
    data=data.yaml \
    imgsz=640 \
    epochs=100 \
    batch=8 \
    device=0 \
    patience=20 \
    project=~/dev_ws/src/valm/training_runs \
    name=food_seg_v3
