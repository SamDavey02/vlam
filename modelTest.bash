yolo segment predict \
  model=~/dev_ws/src/valm/training_runs/food_seg_v3/weights/best.pt \
  source=/home/sam/dev_ws/src/valm/jpegmini_optimized \
  imgsz=640 \
  conf=0.25 \
  device=0 \
  save=True
