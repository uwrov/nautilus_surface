# Bonus documentation file
Make sure you have docker desktop installed

## Container Setup

### Build the Image
Do this the first time you run and whenever you want to make any big code changes
```
docker build --tag nautilus_surface .
```

### Run the Container
```
docker run --name nautilus_surface -p 3000:3000 -v nautilus_logs:/root/logs -it nautilus_surface
```

## Startup Processes

### Debug the interface
1. Start up the interface
    ```
    docker run --name nautilus_surface -p 3000:3000 -v nautilus_logs:/root/logs -it nautilus_surface

    roslaunch nautilus_launch system2.launch
    ```
2. Run whatever

### Launch control system with simulation
1. Launch surface and simulation containers.
    ```
    docker compose up
    ```
2. Open shells into the two containers and run whatever you need to

### Launch control system with robot
1. Launch surface container
    ```
    docker run --name nautilus_surface --network host -v nautilus_logs:/root/logs -it nautilus_surface
    ```
2. Ensure pi is on and pointing to the correct address

3. Run whatever you need to