# nautilus_surface
Nautilus' Surface Control ROS Packages

# How to launch

## On the actual ROV

```
docker-compose up
```

## Without simulator
```
docker-compose -f local-compose.yaml up
```

## With simulator
Ensure you have the [simulator repo](https://github.com/uwrov/nautilus_sim) in a sibling directory.
```
docker compose -f local-compose.yaml --profile sim up
```

For more details, visit the [additional documentation](docker.md)
