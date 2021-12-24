#! /bin/bash 

VAR=$1
VIDEO_SRC=$(if [[ $2 ]]; then echo "$2"; else echo "0"; fi)
echo $VIDEO_SRC

if [ "$VAR" = "model" ]
then
  echo "Using downloaded weights to convert darknet weights to tensorflow model"
  python save_model.py --model yolov4 
elif [ "$VAR" = "model_tiny" ]
then
  echo "Using TINY downloaded weights to convert darknet weights to tensorflow model"
  python save_model.py --weights ./data/yolov4-tiny.weights --output ./checkpoints/yolov4-tiny-416 --model yolov4 --tiny
elif [ "$VAR" = "run" ] 
then
    echo "Running Yolov4 on camera"
    python object_tracker.py --video $VIDEO_SRC --output ./outputs/webcam.avi --model yolov4
elif [ "$VAR" = "run_tiny" ]
then
    echo "Running Yolov4 Tiny on camera"
    python object_tracker_module.py --weights ./checkpoints/yolov4-tiny-416 --model yolov4 --video $VIDEO_SRC --output ./outputs/tiny.avi --tiny
fi

