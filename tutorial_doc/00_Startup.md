# Installation and Setup guide
To run and follow the tutorials, you will need [Ubuntu](https://releases.ubuntu.com/)(recommended: Ubuntu 22.04 LTS), preferably a version that is officially supported by [Drake](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#id1). 
 
* If you are on **Windows** and don’t have Ubuntu → follow [Install Linux via WSL](#install-linux-via-wsl).
* If you **already have Ubuntu installed** → jump directly to [Install Drake](#Install-Drake).

* If you are on **older Ubuntu (e.g. 20.04 LTS)** → the scripts here will still work using Drake’s compatible version.


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
   This installs WSL and sets up Ubuntu 22.04 as your Linux environment.
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

3) Create a Linux username and password
<div style="text-align: center;">
    <img src="images/ubuntUSER.png" alt="UbuntuUser">
</div>

   The first time you start Ubuntu (from the Start menu), you’ll be asked to create them.
   * The password entry is hidden while typing (normal in Linux).

   * This user account has admin rights (`sudo`).

      Forgot your password?

      * "**Open PowerShell** → ``wsl -u root``"

      * "**Reset with** ``passwd <username>``"

4) Update your Linux packages
   ```bash
   sudo apt update && sudo apt upgrade
   ```
## Install Drake
1. [Install the Drake Toolbox](https://drake.mit.edu/installation.html), preferably a [stable release](https://drake.mit.edu/apt.html#stable-releases), either locally or globally on your system. We recommend using the APT-based stable release (not pip). 
\
\
installation via apt:

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
## Install Git and Clone the Repo
Install Git:
```bash
sudo apt-get install git
```
Clone this repository   
```sh
cd 
git clone https://github.com/Coryx99/RoboticsII.git
```
<!-- for more details in case the previous is not enough::::
https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-git -->


## Getting ready Before starting: 
Navigate to the `tutorial_scripts/` folder.

- Run the sanity check script:
```sh 
cd ~/RoboticsII/python_tutorials 
python3 tutorial_sanity_check.py
```
Drake uses MeshCat for 3D visualization, therefore, when you run a script, you will see output like:
```sh 
INFO:drake:Meshcat listening for connections at http://localhost:7000
```
Open the printed link in your browser and you should see a robot visualized.

That’s it! You now have Ubuntu, Drake, and this repo ready to run the tutorials.
