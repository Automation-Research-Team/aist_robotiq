# Initial Setup
* Install package

```bash
pip install python-can
```

# How to use
Connect xela_sensor to PC.

Before running it you must do the following.

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

Run 'demo.launch' of o2as_xela_sensor.
