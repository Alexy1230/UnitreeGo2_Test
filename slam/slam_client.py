import json

from ...rpc.client import Client
from .slam_api import *


"""
" class SlamClient
"""
class SlamClient(Client):
    def __init__(self):
        super().__init__(SLAM_SERVICE_NAME, False)

    def Init(self):
        # set api version
        self._SetApiVerson(SLAM_API_VERSION)
        # regist api
        self._RegistApi(SLAM_API_ID_START_MAPPING, 0)
        self._RegistApi(SLAM_API_ID_END_MAPPING, 0)
        self._RegistApi(SLAM_API_ID_INITIALIZE_POSE, 0)
        self._RegistApi(SLAM_API_ID_POSE_NAVIGATION, 0)
        self._RegistApi(SLAM_API_ID_PAUSE_NAVIGATION, 0)
        self._RegistApi(SLAM_API_ID_RESUME_NAVIGATION, 0)
        self._RegistApi(SLAM_API_ID_CLOSE_SLAM, 0)

    # 1801
    def StartMapping(self):
        p = {}
        p["data"] = {"slam_type": "indoor"}
        parameter = json.dumps(p)

        code, data = self._Call(SLAM_API_ID_START_MAPPING, parameter)
        return code

    # 1802
    def EndMapping(self, save_path):
        p = {}
        p["data"] = {"address": save_path}
        parameter = json.dumps(p)

        code, data = self._Call(SLAM_API_ID_END_MAPPING, parameter)
        return code

    # 1804
    def InitializePose(self, load_path):
        p = {
            "data": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "q_x": 0.0,
                "q_y": 0.0,
                "q_z": 0.0,
                "q_w": 1.0,
                "address": load_path
            }
        }
        parameter = json.dumps(p)

        code = self._Call(SLAM_API_ID_INITIALIZE_POSE, parameter)
        return code

    # 1901
    def CloseSlam(self):
        p = {
        "data": {
            }
        }
        parameter = json.dumps(p)

        code, data = self._Call(SLAM_API_ID_CLOSE_SLAM, parameter)
        return code