# Snake-like robot repository

## Requirements
* Git
* [ROS Melodic Morenia](http://wiki.ros.org/melodic/)
* Python 3
* Pip
* Virtualenv

### Automatic using `Invoke`
You must first install or verify that your computer has installed `python3`, `pip` and `virtualenv`.

```
      $ ➜ virtualenv -p python3 env
      $ ➜ source env/bin/activate
 (env)$ ➜ pip install -r requirements.txt
```

### Upload an sketch to a specific snake robot module (e.g. module 1)
Make sure to physically connect to the right module before upload.
```
  (env)$ ➜ inv upload --module=1
```
`ttyUSB0` is used by default, but optionally you can specify the port by:
```
  (env)$ ➜ inv upload --module=1 --device=USB1
```

### Launch Snake Robot ROS Nodes
In order to launch snake robot ROS nodes, python virtual environment must be deactivated or execute commands in other terminal.
```
  (env)$ ➜ deactivate
  $ ➜ sh install_ros_pkgs.sh #Install ROS required pkgs
  $ ➜ roslaunch bring_up snake_robot.launch
```

### Collaborate!
If you're interested in collaborate on this project, feel free to send a request to the Trello development board: https://trello.com/b/kSHrthf3/snakerobot.
