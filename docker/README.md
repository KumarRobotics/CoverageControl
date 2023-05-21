## Updated Quick Setup

### First Time (for DGX server)
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
5. Create directories for src, data, and models
```bash
  mkdir -p ${COVERAGECONTROL_WS}/src
  mkdir -p ${COVERAGECONTROL_WS}/data
  mkdir -p ${COVERAGECONTROL_WS}/models
  ```
6. `cd CoverageControl_ws/src`
7. `git clone <CoverageControl>`
8. `git clone <gnn_coverage_control>`
9. If a docker image is not created already:  
```bash
  cd ${COVERAGECONTROL_WS}/src/CoverageControl/docker
  docker build -t coverage_control_gnn .
  ```
10. `bash ${COVERAGECONTROL_WS}/src/gnn_coverage_control/docker/create_gpu_container.sh ${COVERAGECONTROL_WS}`
11. You should now be **inside the container** with a `bash` prompt
12. `cd ${COVERAGECONTROL_WS}/src/CoverageControl`
13. `bash setup.sh -i`
14. You can exit the container with `ctrl+d`

### Run Docker and GNN
1. Login to the server via `ssh`
2. `tmux` # or screen
3. `docker start gnn-$USER-jammy` # starts stopped docker
4. `docker exec -it gnn-$USER-jammy bash` # execute bash inside docker
5. Execute your scripts `python <script.py>`
6. `ctrl+b + d` to detach on tmux
7. `tmux attach` to reattach on tmux

### Updating pyCoverageControl
1. Login to the server via `ssh`
2. `cd ${COVERAGECONTROL_WS}/src/CoverageControl`
3. `git pull`
4. Get inside your docker container
6. `cd ${COVERAGECONTROL_WS}/src/CoverageControl`
7. `bash setup.sh -i`
8. You can exit the container with `ctrl+d`


