# Docker setup for RTT

## Why docker?
Docker can create a virtual environment with pre installed depencendies. This makes sure everyone with this virtual environment can run our code.

For a simulation tournament (2022/04), every team had to put their program in a docker image, so that their image could run on the server of the people that hosted the tournament (So less delay between our program and the simulator).
## How to make?
The `Dockerfile` file contains the steps to create the image. It contains all the commands that need to be run to install the required dependencies. It also copies the `clone.sh` script to the image so you dont have to type git clone for our repo.
- To create the image: (note the dot at the end)
    
        docker build -t roboteamtwente/practice_tournament .
- To run the image in a container:

        docker run -e VNC_PW=12345 -p 5901:5901 -p 2222:2222 -it roboteamtwente/practice_tournament

This image that will be created is also publicly available at [the docker hub](https://hub.docker.com/r/roboteamtwente/practice_tournament). This allows others to run our image. Be sure to update that docker once in a while.


## How to use?
On startup, the container will open a VNC session. With this you can remote desktop into the container, where you will be able to run our code. Download a VNC viewer for the remote desktopping(I suggest [this one](https://www.realvnc.com/en/connect/download/viewer/linux/), and log in with the VNC password you set in the `docker run` command (12345).

## Nice to know
- The image already has the text editor sublime text. To start it, open a terminal, and enter `subl <folder>` to open that specific folder.
- Use the sript `clone.sh` to clone the roboteam repo located at `/`.