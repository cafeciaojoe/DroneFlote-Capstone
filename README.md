DroneFlote-Capstone
==============================================================

DroneFlote
----------

The DroneFlote project is a parallel extension of previous work on the Drone Chi project by PHD Candidate Joseph La Delfa which designed an interactive human-drone interface with concepts taken from Tai-Chi and backed by motion capture.

The goal of DroneFlote was to move to an interaction system using live video and the concept of pose recognition whereby the movement and location of specific key points on a person directly impact the way a drone responds in real time and to remove the need for physical controllers.

DroneFlote has heavily utilised the robot operating system (ROS) as a communications platform for messaging to a simulated drone and is fully encapsulated in containers through the use of docker. Through using docker, DroneFlote divides each of its responsibilities into a unique container to enhance scalability and ease deployment across different environments.

Due to the impact of COVID we were forced to move to a simulated environment rather than interacting directly with physical drones. This provided several challenges, such as getting a simulator working inside a container and simulating a drone modeled after the physical drones the client is using. This also gave us a unique opportunity where solving this simulation issue could lead to expanding the RMIT VXLabs range of available robots for future projects.

The final product shall be set of containers which allow a user to access a pose recognition interface through a web browser and view a real time simulation of a CrazyFlie drone as it reacts to the users actions recorded through their attached camera.

Demo
------------

[![DroneFlote Base Demo](https://img.youtube.com/vi/n9M-YfXLTYI/0.jpg)](https://youtu.be/n9M-YfXLTYI)

Deployment
-----------
Installation requirements
--------------------------

Since we have built this project with the aim to separate each of its features for ease of deployment, dockers and containers are the main parts of the running system. I will cover how to install Docker first.

**Git**

This project resides on github where you will find all the relevant files required to deploy DroneFlote. As such we need to download and install git so we can retrieve a copy remotely to deploy.

On Windows we need to install git gui, which can be downloaded and installed here:
https://git-scm.com/downloads

The default components selected by the installer are fine but the option called “configuring the line ending conversions” must be set to “checkout as-is, commit as-is” or the project will not deploy on windows. 

This is the only change you are required to make during installation of git. All other options may be left at the default selected choice.

On linux, installing git from your package manager is more than enough.

***Docker and Compose***

Docker is a solution for creating containers to house applications or parts therein to ease development and deployment, with containers being a low cost solution to deploying individual environments or operating systems as opposed to virtual machines.

***Windows and Mac***

From what I understand, the installation for both systems is somewhat similar but this will guide you through setting it up on windows specifically.

1. First we need to install docker. The current version from windows can be downloaded from the page:
https://www.docker.com/get-started

1. Run the executable once downloaded and make sure all the configuration options are checked.

1. On Windows, Docker relies on a windows feature called wsl2 which must be installed for docker to run. Information about installing this can be found here:
https://docs.microsoft.com/en-us/windows/wsl/install-win10

1. Steps 6 onward may not be necessary.

1. Once the install is complete, restart your PC before continuing.

1. Luckily Docker-Compose comes with Docker desktop so we can stop here.

***Linux***

Both docker and compose can be either installed via the package manager or manually for many versions of linux.

For ease of use I suggest you either install from the package manager or follow the instructions on dockers website:
https://docs.docker.com/engine/install/

The same follows for docker-compose.

**Building**

This is a two part process. First we need to get the project code, then build it into containers before we can then deploy the project as a whole.

Provided you are not updating anything and have already completed these first two steps, to deploy again we need to follow the deployment instructions.

If you do download an updated version you will have to build it again before you run it.

First open up a terminal window or powershell window if you’re on windows and navigate to a directory you would like the source code to live.

For windows users, the easiest way to do this is to open the file explorer and click through to the directory you choose, then click on the address bar (shown below), delete the text, type “powershell” without quotes and hit enter. This opens the powershell in the directory you were in.

> Now we will clone the repository into this folder, creating a new folder containing our project.
```console
$ git clone https://github.com/hupsuni/DroneFlote-Capstone.git
```

> We now have a local copy of DroneFlote. Now enter that directory from the terminal and proceed.

```console
$ cd DroneFlote-Capstone
```
> Build the project
With a terminal open inside the newly downloaded directory we are going to have docker compose build all of our containers for us.
```console
$ docker-compose build
```
*Note: The compose file simplifies the build process extensively from a user perspective but it can take quite some time so let it finish building before proceeding.*

Live Deployment
--------------
Making use of the docker compose feature greatly uncomplicates deployment and running of all the involved containers.While each container can be run individually we can deploy the whole project from docker compose.

>To deploy and run the project, once it has finished building, simply enter this command in the terminal:
```console
$ docker-compose up -d
```

*Note: It may take a minute or two to run all the containers and several minutes more the first time it is deployed.*

>To shut down the project and its containers use the command:
```console
$ docker-compose down
```
***Alternative Deployment using dockerhub for prebuilt images.***

>Download the dockerhub-docker-compose.yml file and run docker-compose.

```console
$ docker-compose -f dockerhub-docker-compose.yml up -d
```

**Accessing the UI**

The project contains several docker images that will all run and communicate with each other.

One of these containers holds the pose recognition over camera application on a web server accessible by going to the following address:
```text
http://localhost:8888/
```

There are two options to select from here. 
The camera demo will use a live feed from your connected camera to map key points to coordinates.
The image demo uses preloaded static images to do the same and does not require a camera.

A gazebo simulator with a simulated drone is also run in another container that can be accessed through a secondary container running vnc by going to the following address:
```text
http://localhost:8080/vnc.html
```
This container simply displays the running simulation in real time through the web browser.

External Dependencies
-------------------

This project utilizes packages and modules from several other independent projects such as; Tensorflow and posenet for image and pose detection. 

1. Docker and docker-compose for containerization, deployment and isolating individual components for cohesion.

1. ROS as a base system for messaging between containers, running python code and interacting with the simulator.

1. Gazebo for simulating our drones in a safe test environment and for testing purposes.

1. CrazyS and Mavcomm which are both git repositories that enable us to model and fly a simulated drone modeled after the drones used in practice.  
  Together with these we have integrated a system where the pose of a user can dynamically change how a simulated drone will fly.

1. Apache for serving web based interfaces for both the simulator and pose recognition.

1. No-VNC for accessing the simulator over a web browser.
