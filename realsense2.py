from slam_wrapper import SLAMWrapper

class SLAM:
    def __init__(self):
        # ORDER IS IMPORTANT
        channels=[SLAMWrapper.RGB_RAW,
                  SLAMWrapper.DEPTH_RAW,
                  SLAMWrapper.INFRA1_RAW,
                  SLAMWrapper.INFRA2_RAW,
                  SLAMWrapper.ODOMETRY]
        self.slam=SLAMWrapper()
        self.slam.subscribe_batch(channels, self.ros_update)
        self.frames=None
    def ros_update(self, data):
        self.frames=data
    def run(self):
        return self.frames
