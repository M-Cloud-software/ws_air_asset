# MCLOUD Air Asset Software – Project Workspace

Welcome to the MCLOUD Air Asset Software repository!
This workspace serves as a shared foundation for all sub-projects developed under the MCLOUD team.
It is designed to keep things organized, modular, and easy to extend as new functionality is added.

# Repository Structure
```
ws_air_asset/
├── src/                # Source packages for various project modules
│   ├── <package_name>/ # Each folder here represents a distinct package or subproject
│   │   ├── include/    # Header files (if applicable)
│   │   ├── src/        # Source code files (.cpp, .py, etc.)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── ...
├── models/             # Shared simulation or visualization models (for PX4, Gazebo, QGC, etc.)
├── launch_project      # Helper script to configure and launch the workspace
├── .gitignore
└── README.md
```
# Getting Started
### 1. Clone the Repository

```
git clone git@github.com:M-Cloud-software/Software.git
cd ws_air_asset
```

### 2. Launch the Workspace

The launch_project script is your main entry point for setting up and running the workspace environment.

`./launch_project`

The `launch_project` script initializes environment variables, sets up package paths, and ensures that dependencies are sourced properly.

### 3. **Only if you make code changes when working on a package**

Run `./launch_project build`

# Developing Packages

All development takes place inside the src/ directory.

Each subfolder in src/ represents a package or module that can be independently built, tested, and maintained.

Example layout
```
src/
├── flight_control/
│   ├── include/
│   ├── src/
│   ├── launch/
│   ├── CMakeLists.txt
│   └── package.xml
├── perception/
│   ├── src/
│   ├── data/
│   └── CMakeLists.txt
└── comms/
    ├── src/
    └── CMakeLists.txt
```

## When creating a new package:

1. Create a new folder under src/

2. Follow the existing naming convention

3. Add your build and dependency files (CMakeLists.txt, package.xml, etc.)

4. Ensure it integrates cleanly with the launch_project script if applicable

# Collaboration Guidelines

Keep all temporary or generated files (like build/, install/, and log/) out of version control
— these are already excluded via .gitignore.

Use feature branches for new functionality:

git checkout -b feature/<your-feature-name>

Keep commit messages descriptive and concise.

Push regularly to your team branch to avoid merge conflicts.

# Extra Notes

The models/ folder can be sourced by external tools such as PX4, Gazebo, or QGroundControl.

The repository is intended to serve as a central workspace, not a single monolithic project — each package should remain modular.

If you modify the launch_project script, please document your changes clearly in this README or a CHANGELOG.md.