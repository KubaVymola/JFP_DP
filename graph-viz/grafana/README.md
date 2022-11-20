```
$ docker build --tag graph-viz .

$ docker run -p 3000:3000 -v$(pwd):/data -it --name graph-viz-container --rm graph-viz

$ docker exec -it graph-viz-container /bin/sh
```