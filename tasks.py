from invoke import task, run
from termcolor import colored

'''
This task compiles an arduino sketch file (*.ino) and uploads to
the Arduino in the given port.
'''
@task
def upload(context, module=1, device='USB0'):
    modules = range(1,7)
    ARDUINO_PATH = '~/Software/arduino-1.8.7/'
    SKETCH_PATH = 'Arduino/module/module.ino'
    ARDUINO_PORT = f'/dev/tty{device}'
    if not module:
        print(colored('Module number (1-7) must be specified', 'red'))
        return
    if not module in modules:
        print(colored(f"Unknown module '{module}'", 'yellow'))
        return
    # Change module ID sketch
    command = (f"sed -i 's/MOD_ID = .*/MOD_ID = \"{module}\";/g'"
               f" {SKETCH_PATH}")
    print(colored("Changing sketch module id . . .", 'yellow'))
    run(command)
    # Upload scketch
    command = (f'{ARDUINO_PATH}./arduino '
              f'--upload {SKETCH_PATH} '
              f'--port {ARDUINO_PORT}')
    print(run(command))
