from invoke import task, run
from termcolor import colored

'''
This task compiles an arduino sketch file (*.ino) and uploads to
the Arduino in the given port.
'''
@task
def upload(context, module=0):
    modules = range(1,7)
    ARDUINO_PATH = '~/Software/arduino-1.8.7/'
    SKETCH_PATH = '~/snake_robot/Arduino/'
    ARDUINO_PORT = '/dev/ttyUSB0'
    if not module:
        print(colored('Module number(1-7) must be specified', 'red'))
        return
    if not module in modules:
        print(colored(f"Unknown module '{module}'", 'yellow'))
        return
    command = (f'{ARDUINO_PATH}./arduino '
              f'--upload {SKETCH_PATH}/module{module}/module{module}.ino '
              f'--port {ARDUINO_PORT}')
    print(run(command))

@task
def installRosDep(context):
    BASE = 'ros-melodic-'
    ros_packages = ['joy']
    packages = ''
    for package in ros_packages:
        packages += f'{BASE}{package} '
    command = f'sudo apt install {packages}'
    print(run(command))
