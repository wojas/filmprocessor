# https://community.platformio.org/t/any-way-to-configure-timeout-for-upload-monitor/3812/2

Import("env")

def after_upload(source, target, env):
    print("Delay monitor after uploading...")
    import time
    time.sleep(5)
    print("Done!")

env.AddPostAction("upload", after_upload)
