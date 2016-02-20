import picamera
from time import sleep, gmtime

#
# Duration of video capture, in seconds
#
VIDEO_DURATION = 600

#
# Returns the current time as a string with the format:
# mm-dd@hh:mm:ss
#
def getCurrentTimeString ():
    tm = gmtime()

    return "%04d-%02d-%02d_%02d:%02d:%02d" % \
        (tm.tm_year,tm.tm_mon,tm.tm_mday,tm.tm_hour,
        tm.tm_min,tm.tm_sec)

try:
    #
    # Initialize camera and settings
    #
    camera = picamera.PiCamera()

    camera.sharpness = 0
    camera.contrast = 0
    camera.brightness = 50
    camera.saturation = 0
    camera.ISO = 0
    camera.video_stabilization = False
    camera.exposure_compensation = 0
    camera.exposure_mode = 'auto'
    camera.meter_mode = 'average'
    camera.awb_mode = 'auto'
    camera.image_effect = 'none'
    camera.color_effects = None
    camera.rotation = 0
    camera.hflip = False
    camera.vflip = False
    camera.crop = (0.0, 0.0, 1.0, 1.0)

    #
    # Take initial image
    #
    camera.capture('init_image_' + getCurrentTimeString() + '.jpg')

    #
    # Capture video
    #
    camera.start_recording('video_' + getCurrentTimeString() + '.h264')

    #
    # Wait predetermined amount of time
    #
    sleep(VIDEO_DURATION)

finally: #finally clause to stop recording if interrupted during sleep

    #
    # Stop video capture
    #
    camera.stop_recording()

    #
    # Take final image
    #
    camera.capture('fin_image_' + getCurrentTimeString())
