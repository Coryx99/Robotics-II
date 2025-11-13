# Installation and Setup guide
To run and follow the tutorials, you will need [Ubuntu](https://releases.ubuntu.com/)(recommended: Ubuntu 22.04 LTS), preferably a version that is officially supported by [Drake](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#id1). 
 
* If you are on **Windows** and don’t have Ubuntu → follow [Install Linux via WSL](#install-linux-via-wsl).

* If you **already have Ubuntu installed** or you are on **older Ubuntu (e.g. 20.04 LTS)** → jump directly to [Install Drake](#Install-Drake).

## Install Linux via WSL
Developers on Windows can run both Windows and Linux side-by-side using the Windows Subsystem for Linux (WSL).
This is the easiest way to get Ubuntu running without a full virtual machine.

> ⚠️ Requirements: Windows 10 (build 19041+) or Windows 11.
For older versions, see the [manual install page](https://learn.microsoft.com/en-us/windows/wsl/install-manual).

1) Open PowerShell as Administrator
Right-click PowerShell → Run as administrator. Then run:
   ```powershell
   wsl --install Ubuntu-22.04
   ``` 
   <!-- This installs WSL and sets up Ubuntu 22.04 as your Linux environment. -->
   If WSL doesn’t recognize the distro, try:

   ```powershell
   wsl --install -d Ubuntu-22.04
   ``` 
   then restart your machine.

2) Check your WSL version
   ```PowerShell
   wsl.exe --list --verbose
   ```
   If Ubuntu is running with `VERSION 1`, upgrade it:
   ```powershell
   wsl --set-version Ubuntu-22.04 2
   ```

3) The first time you start Ubuntu (from the Start menu), you’ll be asked to create a Linux username and password.
<div style="text-align: center;">
    <img src="images/ubuntUSER.png" alt="UbuntuUser">
</div>

4) Update your Linux packages
   ```bash
   sudo apt update && sudo apt upgrade
   ```
## Install Drake
1. [Install the Drake Toolbox](https://drake.mit.edu/installation.html), preferably a [stable release](https://drake.mit.edu/apt.html#stable-releases), either locally or globally on your system. We recommend using the APT-based stable release:

   ```sh
   sudo apt-get update
   sudo apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget
   wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
   echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
   sudo apt-get update
   sudo apt-get install --no-install-recommends drake-dev
   ```
2. Environment variables: add the following to your ~/.bashrc (or ~/.bash_aliases):

   ```sh
   export PATH="/opt/drake/bin${PATH:+:${PATH}}"
   export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
   ```

3. Reload your shell
   ```bash
   source ~/.bashrc
   ```

## Install Required Python Packages
Install additional Python dependencies:
```ssh
sudo apt-get install python3-pip
pip install ompl numpy matplotlib
```

## Install Git and Clone the Repo
1. Install GIT :
```bash
sudo apt-get install git
```
<!-- 2. Set up SSH access to GitHub

   Since this repository is private, you need to configure SSH access with your GitHub account.
   Follow this step-by-step guide: 
   [👉 How to stup SSH keys for github cloning](https://www.theserverside.com/blog/Coffee-Talk-Java-News-Stories-and-Opinions/github-clone-with-ssh-keys). -->

2. Clone this repository   
```sh
cd 
git clone https://github.com/Coryx99/Robotics-II.git
```
<!-- for more details in case the previous is not enough::::
https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-git -->


## Getting ready Before starting: 
Navigate to the `tutorial_scripts/` folder.

- Run the sanity check script:
```sh 
cd ~/Robotics-II/tutorial_scripts 
python3 tutorial_sanity_check.py
```
Drake uses MeshCat for 3D visualization, therefore, when you run a script, you will see output like:
```sh 
INFO:drake:Meshcat listening for connections at http://localhost:7000
```
Open the printed link in your browser and you should see a robot visualized.

That’s it! You now have Ubuntu, Drake, and this repo ready to run the tutorials.
