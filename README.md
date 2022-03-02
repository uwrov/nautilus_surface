# nautilus_surface
Nautilus' Surface Control ROS Packages


# How to launch

## 1. Check IP's (for SSH access)
IP's should already be configured for the router used, but in case something went wrong, these are the expected values:
- Surface computer: `192.168.0.69` (port 22)
- Raspberry Pi on ROV: `192.168.0.99` (port 69 for the Pi itself, port 22 for the `nautilus_pi` container)

## 2. On the ROV Raspberry Pi

Start the Pi docker container

``` bash
cd nautilus_pi # or wherever the repo is located
sudo docker-compose up
```

## 3. On the surface computer

In one terminal window:
``` bash
cd nautilus_surface
sudo docker-compose up
```

In another window:
``` bash
con # alias for `sudo docker exec -it surface bash`
```

## 4. Within the surface docker container
``` bash
roslaunch nautilus_launch system.launch
```

(Optional) execute `pi` to SSH into the Raspberry Pi


# For work on the simulator
See instructions in the [simulator repo](https://github.com/uwrov/nautilus_sim).


# More details
For more details, visit the [additional documentation](docker.md).
