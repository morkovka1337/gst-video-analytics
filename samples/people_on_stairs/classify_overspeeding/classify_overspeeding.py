from os.path import join, realpath
from os import listdir, environ
import shlex
import subprocess
import pickle
import json
import pickle as pkl
import time
import numpy as np
from copy import copy

MODEL_PATH = ("/root/Projects/models/intel/person-detection-retail-0013/FP32"
                "/person-detection-retail-0013.xml")

DATASET_PATH = "/root/Projects/augmented_dataset/augmented_videos/val"
ALPHA = 0.1
ALPHA_HW = 0.01
RES_PATH = ("/root/Projects/gst-video-analytics-0.7.0/samples/"
            "people_on_stairs/classify_overspeeding/res.json")

SVM_PATH = '/root/Projects/models/overspeed_classify/SVM_Classifier_without_interval.sav'

CLASSIFY_PIPELINE_TEMPLATE = """gst-launch-1.0 filesrc \
        location={} \
        ! decodebin  ! videoconvert ! video/x-raw,format=BGRx ! gvadetect  \
        model={} ! queue  \
        ! gvaspeedometer alpha={} alpha-hw={} interval=0.03333333 \
        ! gvapython module={} class=OverspeedClassifier arg=[\\"{}\\"]   \
        ! fakesink sync=false"""


class OverspeedClassifier():
    def __init__(self, out_path=RES_PATH):

        self.velocities = []
        self._result_path = out_path
        self.frames_processed = 0

    def process_frame(self, frame):

        for region in frame.regions():
            for tensor in region.tensors():
                if tensor.has_field("velocity"):
                    self.velocities.append(tensor['velocity'])

        self.__updateJSON()
        self.frames_processed += 1

    def __updateJSON(self):
        with open(self._result_path, "w") as write_file:
            json.dump(self.velocities,
                      write_file, indent=4, sort_keys=True)

    def __dump_data(self):
        with open(self._result_path, "a") as write_file:
            write_file.write("{} \n".format(self.velocities))


if __name__ == "__main__":
    svclassifier = pickle.load(open(SVM_PATH, 'rb'))
    for file_name in listdir(DATASET_PATH):
        if file_name.endswith(".mp4"):
            video_path = join(DATASET_PATH, file_name)
            pipeline_str = CLASSIFY_PIPELINE_TEMPLATE.format(
                video_path,
                MODEL_PATH,
                ALPHA,
                ALPHA_HW,
                realpath(__file__),
                join(DATASET_PATH, file_name.replace('.mp4', '.json'))
            )
            print(pipeline_str)
            proc = subprocess.run(
                shlex.split(pipeline_str), env=environ.copy())

            if proc.returncode != 0:
                print("Error while running pipeline")
                exit(-1)
