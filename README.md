# MEAM520: Introduction to Robotics
### Date Created: 08/26/2021

Maintainers: Contact FY2021 TAs

# Subdirectory Structure:
- `core`: contains support code we provide to you, such as helper functions and interfaces for interacting with the robot (either simulated or real!)
- `lib`: will contain functions implementing algorithms to solve computational problems relating to the class material, such as forward and inverse kinematics and trajectory planning
- `labs`: will contain test scripts that use the algorithms you implement in `lib` and the tools we provide in `core` to control the robot and achieve tasks
- `ros`: contains code necessary to launch the simulator. You won't need to work in this directory.

`lib` and `labs` will be the main places you are working this semester!


# Native Install Instructions (NOT REQUIRED FOR VIRTUAL MACHINE)
---
**NOTE**

These installation instructions are for the TAs when setting up the Virtual Machine, and can also be followed by experienced users to set up the lab environment on a native Ubuntu Linux installation. These steps do not need to be followed by students using the Virtual Machine. For all other users, skip to the section on forking this repo.

---

## Operating System

The simulator must be run on Ubuntu 20.04 with ROS noetic installed. You can follow the standard installation instructions for [Ubuntu 20.04](https://phoenixnap.com/kb/install-ubuntu-20-04) and [ROS noetic](http://wiki.ros.org/noetic/Installation).

## panda_simulator installation

To get started using this development environment, you must first follow the instructions to install [panda_simulator](https://github.com/justagist/panda_simulator), a Gazebo-based simulator for the Franka Emika Panda robot. The only difference is that you will name the catkin workspace `meam520_ws` to avoid conflicts with other projects.

The instructions specify to use `catkin build`, but we recommend building with `catkin_make_isolated` instead.

Once you've completed that installation, add

```
source ~/meam520_ws/devel_isolated/setup.bash
```

to your `~/.bashrc` to source the workspace. If all has been done correctly, you should be able to run

```
roslaunch panda_gazebo panda_world.launch
```

to launch the Gazebo simulation.


### Speed up Gazebo shutdown

Since you may be launching and killing Gazebo many times this semester, we recommend reducing the clean shutdown wait time for a better experience. Edit the file:
```
cd /opt/ros/noetic/lib/python3/dist-packages/roslaunch
sudo vim nodeprocess.py
```
and change the lines that say
```
...
_TIMEOUT_SIGINT = 15.0 #seconds

_TIMEOUT_SIGTERM = 2.0 #seconds
...
```
to `2.0` and `1.0` respectively.

## Get the Code
Git is a code versioning tool, and can be a powerful resource. 
This year we will use git in a limited fashion to ease the dissemination of code. 
So that each student can have their own individual private work we are going to have you work on a copy of the original repository in your own GitHub account. 

If you have a GitHub account you can skip step 1 and move onto step 2: cloning a new private repository.

### Setting up a gitHub account
- Go to: https://github.com
- Select signup
- follow the prompts to make an account using your Penn email.
- login into your account.
- Go to the top right corner of your account and select the circle icon, navigate to settings, on the left column you will see a list of opitions, select SSH and GPG keys.
- Next open a terminal in the Virtual Machine (Ctrl-Alt-t), and type the following command:
```
$ ssh-keygen -t ed25519 -C "your_email@seas.upenn.edu"
```
- Once you have finished generating the key (hitting enter for defaults works) type the following commands
```
$ cd ~/.ssh
$ cat id_ed25519.pub
```
 - You will see a string output on the terminal starting with: "ssh-ed25519" and ending with your email.
 - In your GitHub account, under ``Settings'', select ``SSH and GPG Keys'', ``New SSH-key'', name the key and copy the entire string on your terminal into the appropriate box. (To copy things off of the terminal highlight the text and click Ctrl-Shift-C. You may also need to enable Devices > Shared Clipboard > Bidirectional in the VirtualBox toolbar.) You have now made it possible to clone your private repository to your virtual machine.
- Note: GitHub has an educational developers pack which you are eligible for as a Penn student. To learn more about this and to register your account go to: https://education.github.com/discount_requests/student_application. To expedite this process it is important that you use your Penn email to make your account (there is a verification process). 	

### Cloning a new private repository:
Each of you will be making a deep copy of the TA existing repository which contains code you will use throughout the semester, e.g.  unimplemented functions, that you implement for a lab.
We will outline a set of steps here so that you are able to keep your code private on your own GitHub account, and get updates as the TAs make updates to the codes, e.g. as new assignments are released. 
The following steps allow you to copy the *meam520_labs* code base:
    
1. If you are not already, login to your GitHub account
2. In your terminal do the following:
```
$ cd ~/meam520_ws/src
$ git clone --bare https://github.com/MEAM520/meam520_labs.git
```
This has created a bare repository of the TAs stub code, which we will refer to TA code for the remainder of the instructions. Note that you will not be using this cloned repository and you will see it appear in your src folder as meam520_labs.git.

3. Next go to your GitHub account and in the top right corner click on your circle user icon, go down to *your repositories*

4. At the top of the new page select New Repository, name the repository: *meam520_labs*. This will load to a blank git repository with instructions on how to clone and add things to it, ignore these. 
        
**IMPORTANT: You MUST set your repository to Private (NOT PUBLIC), since publicly posting your assignment solutions online is a violation of Penn's Code of Academic Integrity.**

5. Next we are going to push the TA code to your newly created git repository. 
```
$ cd meam520_labs.git
$ git push --mirror git@github.com:<YOUR_USERNAME>/meam520_labs.git 
```
Note you need to replace **YOUR\_USERNAME** with your GitHub username.
To check that this step is executed correctly go to your GitHub account and reload your new *meam520_labs* repository page. You should see the README and files loaded into the repository.

6. Now we will remove the TA repository from your machine as follows: 
```
$ cd ..
$ rm -rf meam520_labs.git
```    
        Now if you type *ls* in *~/meam520\_ws/src* you should not see a directory called *meam520\_labs.git*

7. We will clone your new private repo as follows (you can get the link for the new repo from the top right corner in the green box labeled Code): 

```
$ cd ~/meam520_ws/src
$ git clone git@github.com:<YOUR_USERNAME>/meam520_labs.git
```

Note you need to replace **YOUR_USERNAME** with your GitHub username.

8. You should be able to type *ls* in your terminal and see a new directory created called meam520_labs*, which now points to *your* github repository.

### Getting code updates from the TAs:

Now we are going to make it possible for you to get updates from the TAs main repository. First it is important to understand that git is a tool which is used locally to keep a history of your code.
To ensure that code is backed up to an additional location outside of your computer, and to collaborate with others, a *remote* is setup.
GitHub is an example of a location which stores remote git repositories and acts as a way to backup code to a secondary location.

To see that sure your local git repository is setup correctly type the following command:
```
$ cd meam520_sim
$ git reomte -v
```

You should see:
```
> origin  https://github.com/YOUR_USERNAME/meam520_sim.git (fetch)
> origin  https://github.com/YOUR_USERNAME/meam520_sim.git (push)
```

The *origin* is the primary remote location, and this is pointing to the repository you forked to your account.
    
Now we are going to add additional information telling your git repository to check updates made at the original repository location (in this case where the TAs will make updates and release new projects). In git language this is called *setting the remote upstream*.

Do the following:
```
$ git remote add upstream https://github.com/MEAM520/meam520_labs.git
$ git remote set-url --push upstream DISABLE
$ git remote -v
```

You should now see:

```
> origin    https://github.com/YOUR_USERNAME/meam520_sim.git (fetch)
> origin    https://github.com/YOUR_USERNAME/meam520_sim.git (push)
> upstream  https://github.com/MEAM520/meam520_labs.git (fetch)
> upstream  DISABLED (push)
```

Notice that origin still points to your fork, and that upstream is now pointing to the repository that is maintained by the TAs.
Now that an upstream is set we will ask that you periodically use the following command:

```
git pull upstream master
```

    This will ensure that you get updates when TAs make changes. You should run this command before starting each new lab. We will also post on Piazza if any changes need to be released during a given lab. 
    
    You can also use your git repository to make it easier to collaborate with your lab group as you work on the assignments (optional but helpful!). To learn more about the basics of git, check out https://guides.github.com/introduction/git-handbook/. 

## Update Python path to find core module

Add the following line to your ~/.bashrc to allow your python scripts to find the meam520 core modules.

```
export PYTHONPATH="${PYTHONPATH}:/home/${USER}/meam520_ws/src/meam520_labs"
```

## meam520_labs installation

Once you have a forked, cloned, and set the upstream for the git repository run `catkin_make_isolated` again.

Once the build finishes, open a new terminal and run:

```
roslaunch meam520_labs lab0.launch
```

To check that the installation is successful, run a demo script:

```
cd ~/meam520_ws/src/meam520_labs/labs/lab0
python demo.py
```

You should see the robot in Gazebo moving.



