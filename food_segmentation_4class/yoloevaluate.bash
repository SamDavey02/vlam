yolo segment val \
    model=~/dev_ws/src/valm/training_runs/food_seg_v3/weights/best.pt \
    data=data.yaml \
    split=test \
    imgsz=640 \
    device=0
