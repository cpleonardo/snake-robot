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
