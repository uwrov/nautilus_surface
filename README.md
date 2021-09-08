# nautilus_surface
Nautilus' Surface Control ROS Packages

# Usage
Make sure you have docker desktop installed

## Useful Methods

### Listing Images
```
docker images
```

### Checking for Running Images
```
docker ps -a
```

## Container Setup

### Build the Image
```
docker build --tag nautilus_surface .
```

### Run the Container
```
docker run --name nautilus_surface -p 3000:3000 -it nautilus_surface
```

## System commands

### Run the System
```
roslaunch nautilus_launch system2.launch
```