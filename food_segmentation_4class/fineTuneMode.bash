yolo segment train \
    model=~/dev_ws/src/valm/training_runs/food_seg_v3/weights/best.pt \
    data=data.yaml \
    imgsz=640 \
    epochs=50 \
    batch=8 \
    device=0 \
    patience=15 \
    project=~/dev_ws/src/valm/training_runs \
    name=food_seg_v4_angles
