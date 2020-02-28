from slam_wrapper import SLAMWrapper
d=None
def callback(data):
    global d
    d=data
    print(data.keys())

slam=SLAMWrapper()

slam.subscribe_single(slam.RGB_RAW, callback)
print('subscribed to {}'.format(slam.topics[slam.RGB_RAW]))
slam.start()
slam.spin()
print('done')
