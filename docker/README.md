## Updated Quick Setup

### First Time
1. Login to the server via `ssh`
2. Copy the following to the end of your `.bashrc` file for easy navigation:
```bash
export RAID_HOME=/raid0/docker-raid/${USER}  
export COVERAGECONTROL_WS=${RAID_HOME}/CoverageControl_ws  
export LD_LIBRARY_PATH="${COVERAGECONTROL_WS}/install/lib/:${LD_LIBRARY_PATH}"  
```
3. Logout and login again 
4. Go to your raid0 directory:  
`cd ${RAID_HOME}`
5. `mkdir -p CoverageControl_ws/src`
6. Create directory for storing data  
`mkdir CoverageControl_ws/data`
7. Create directory for storing models  
`mkdir CoverageControl_ws/models`
8. `cd CoverageControl_ws/src`
9. `git clone <CoverageControl>`
10. `git clone <gnn_coverage_control>`
11. `bash ${COVERAGECONTROL_WS}/src/gnn_coverage_control/docker/create_gpu_container.sh`
12. You should now be **inside the container** with a `bash` prompt
13. `python -m venv $VIRTUAL_ENV`
14. `source ${VIRTUAL_ENV}/bin/activate`
15. `cd ${COVERAGECONTROL_WS}/src/gnn_coverage_control`
16. `pip install -r requirements.txt`
17. `cd ${COVERAGECONTROL_WS}/src/CoverageControl`
18. `./setup.sh -c && ./setup.sh -i`
19. `pip install .`
20. You can exit the container with `ctrl+d`

### Run Docker and GNN
1. Login to the server via `ssh`
2. `tmux` # or screen
3. `docker start gnn-$USER-jammy` # starts stopped docker
4. `docker exec -it gnn-$USER-jammy bash` # execute bash inside docker
5. `source ${VIRTUAL_ENV}/bin/activate`
6. Execute your scripts `python <script.py>`
7. `ctrl+b + d` to detach on tmux
8. `tmux attach` to reattach on tmux

### Updating pyCoverageControl
1. Login to the server via `ssh`
2. `cd ${COVERAGECONTROL_WS}/src/CoverageControl`
3. `git pull`
4. Get inside your docker container
5. `source ${VIRTUAL_ENV}/bin/activate`
6. `cd ${COVERAGECONTROL_WS}/src/CoverageControl`
7. `./setup.sh -u`
8. `pip install .`
9. You can exit the container with `ctrl+d`


## Getting started with docker

1. Make sure `nvidia-docker` is installed on your machine, and you have ~8GB of free disk space.
2. From this directory, run `docker build -f Dockerfile -t gnn-img .`. This (re)builds a docker image named `gnn-img` and is only neccessary after the Dockerfile changes.
3. From this directory, run `./docker/create_gpu_container.sh` to create the docker container. If you are running locally and not on the DGX server, run `./docker/create_gpu_container.sh -l -g all -p ~/path-to-repo/heterogeneous-gnn-tasks`. This creates a container with the name `gnn-$USER` where `$USER` is your actual username by default. It also automatically allocates all your gpus. There are also more fun flags in `./docker/create_gpu_container.sh`, please check carefully how to utilize them.
4. You should now be inside the container. You can exit the container with `ctrl+d`
5. Did you exit your container and want to enter it again?
```
docker start gnn-$USER
docker exec -it gnn-$USER bash
```
To stop the docker container:
```
docker stop gnn-$USER
```
To remove the container (if you need to rebuild it)
```
docker rm gnn-$USER
```

## Updating the image
Edit the `Dockerfile` which defines what is installed in the image. Stop and remove any containers that depend on the old image, then rebuild the image, and then start a new container.
```
docker stop gnn-$USER
docker rm gnn-$USER
docker build -f code/Dockerfile -t gnn-img code/
./docker/create_gpu_container.sh <optional flags>
```

## Running on the remote server

Login to the remote server according to these instructions: [Instructions link](https://docs.google.com/document/d/1S6r9ycEaHQr_P2sC4J_n2FMSH2_F8WT13L6u2cm1pfU/edit?usp=sharing)

The docker image `gnn-img` should already exist on the DGX server. Please let others know via slack before updating it.

After logging in, start a docker container. It may be best practice to run the container from a tmux or other terminal emulator so you can detach the session, and not worry about maintaining the ssh connection:

```
tmux # start tmux interactive session, in which we'll create the docker container
```
**todo: setup a test script, this currently doesn't exist:**
To run the basics, in the container run:
```
./test.sh
```

You can detach the tmux session in which `test.sh` or any other code is running by pressing `CTRL+b` and then `d`.

It's now safe to log out of the ssh session - your code will still be running on the remote server.

Re-attach to the session at any time by logging back into the server and running `tmux attach-session` or `tmux attach-session -t SESSION_ID` for a specific session.

After your code is finished running, exit the tmux session by re-attaching and pressing `CTRL+d` or running `tmux kill-session -t SESSION_ID`

## Files on DGX
Docker containers have their own file systems that are not typically accessible outside of the container. We have mounted volumes for 'experiments' and 'datasets' that allow access to data from outside of these containers. We've made two linked volumes:
* `/root/gnn_coverage_control/experiments` within the container is linked to `/raid0/docker-raid/walker/experiments` in the DGX filesystem
* `/root/gnn_coverage_control/datasets` within the container is linked to `/raid0/docker-raid/walker/datasets` in the DGX filesystem
So when uploading datasets to use for training, place them in `/raid0/docker-raid/walker/datasets`. When a model is trained within the container, it should be saved to the `/root/gnn_coverage_control/experiments` folder. Please check this before stopping the container.

## Gpus on DGX - Currently Broken
Now that we're here, we need to pick what GPU we want to run. so first we'll check which is in use:

```
$ nvidia-smi
Fri Nov 13 17:25:12 2020
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 384.183      Driver Version: 384.183      CUDA Version: 9.0      |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  Tesla V100-SXM2...  On   | 00000000:06:00.0 Off |                    0 |
| N/A   29C    P0    43W / 300W |   3000MiB / 32502MiB |    100%      Default |
+-------------------------------+----------------------+----------------------+
|   1  Tesla V100-SXM2...  On   | 00000000:07:00.0 Off |                    0 |
| N/A   31C    P0    44W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
|   2  Tesla V100-SXM2...  On   | 00000000:0A:00.0 Off |                    0 |
| N/A   31C    P0    42W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
|   3  Tesla V100-SXM2...  On   | 00000000:0B:00.0 Off |                    0 |
| N/A   29C    P0    43W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
|   4  Tesla V100-SXM2...  On   | 00000000:85:00.0 Off |                    0 |
| N/A   29C    P0    43W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
|   5  Tesla V100-SXM2...  On   | 00000000:86:00.0 Off |                    0 |
| N/A   32C    P0    43W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
|   6  Tesla V100-SXM2...  On   | 00000000:89:00.0 Off |                    0 |
| N/A   31C    P0    43W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+
|   7  Tesla V100-SXM2...  On   | 00000000:8A:00.0 Off |                    0 |
| N/A   30C    P0    42W / 300W |     10MiB / 32502MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
|  No running processes found                                                 |
+-----------------------------------------------------------------------------+
```

We can see that GPU `0` is in use, but we can use another, say `6` and `7`:
```
export CUDA_VISIBLE_DEVICES=6,7
```


### DEPRECATED USAGE:
We need to create a GPU container on the DGX server that uses GPUs 6 and 7. On the DGX terminal, type:
```
./docker/create_gpu_container.sh -g 6,7
```
Now you're in the docker container, which again will be named `gnn-$USER`, and utilizing gpus `6` and `7`.

