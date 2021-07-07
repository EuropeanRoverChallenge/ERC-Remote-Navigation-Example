# ERC Remote Navigation Example
This repository serves as a template that the teams can use as a starting point when developing their custom image for the Navigation and Science task in the ERC Remote competition.

To spare the teams the hassle of cross building the software for the target platform of the robot, we developed a Github workflow that automatically builds the Docker images **for amd64 and arm64 platforms**, and publishes them to the Github Container registry.

## Features
The Dockerfile extends the image from [ERC-Remote-Image-Base](https://github.com/EuropeanRoverChallenge/ERC-Remote-Image-Base) repository and:
 - installs additional packages - by default only the `tmux` package is added as an example but you can list any other packages you would like to install,
 - creates a catkin workspace, copies the ROS packages from the `src` directory, installs dependencies using `rosdep` and builds the workspace,
 - adds a line to the system-wide `bashrc` file that will automatically source the workspace on a new bash session,
 - copies the `start.sh` script and sets it as a default command for the image.

The `erc_bringup` package is added as an example and contains:
 - `image_saver` node - a script that let's you save an image from a camera to a PNG file by publishing a message, containing camera topic name, on the `image_saver/save` topic,
 - `erc_bringup.launch` file - a launch file that starts the `image_saver` node and is launched by the `start.sh` script when the container starts.

## Quick start guide
### 1. Import the code to a private repository
To make use of the Github actions, we recommend importing this project to a new private repository on Github. The simplest method to do this, is to use Github's [Import Repository](https://github.com/new/import) tool. 

For `Your old repository's clone URL`, choose: \
https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Example.git

Select the `Owner` as either your account or the organization you are in. \
Choose the `Repository Name` at your discretion.
On the `Privacy` tab, select `Private`.

Click `Begin import`.

Once the project is imported, a Github workflow run should automatically start (check the `Actions` tab). If it did not start, make sure that actions are allowed in the repository settings.

### 2. Add your custom software
Now you can clone your new repository and add your software. For example: 
 - To add custom ROS packages, simply put them into the `src` directory. Make sure that you correctly define dependencies in `package.xml` files, as rosdep will use this information to install dependent packages.
 - To install any additional packages or files, edit the Dockerfile by adding or modying existing commands.
 - To change the starting behavior, edit the `start.sh` script.

Once you have your changes ready, commit them and push to the repository. A new workflow run should start.

### 3. Import credentials for the device
You will need to obtain credentials for the device on the Freedom Robotics platform, in order to make Freedom agent connect to it when the container is started. Assuming you already created a device you would like to use, head to the **SETTINGS** -> **TOKENS**, go to the **DEVICE** tab, click **ADD TOKEN**, select your device and click **CREATE**. Use this token to replace the values in the `credentials.env` file with valid credentials. 

### 4. Pull the docker image
Once the Github workflow completes its run, you should be able to pull the image from Github container registry to your local computer for testing purposes. To do this, you will first need to [Authenticate to the Container registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry), then simply run:
```
docker pull ghcr.io/OWNER/IMAGE_NAME
```
Replace `OWNER` with your username or organization on Github and the `IMAGE_NAME` with with the repository name. \
You can tag your commits if you want to be able to access older images. For example, if you push a `v10` tag the image will also be built with this tag and you can pull it any time by running:
```
docker pull ghcr.io/OWNER/IMAGE_NAME:v10
```

### 5. Run the docker image
Now, you can run and test your image:
```
docker run -it --env-file credentials.env --name erc_img ghcr.io/OWNER/IMAGE_NAME
```
If everything is working correctly, the status of the device on the Freedom Robotics platform should change to `Connected`. You can then try to try to enable SSH tunnel by going into **Settings** -> **Remote SSH** and clicking **Enable Remote SSH**. Paste the ssh command into a terminal and run it, when asked for password, type `root`.
