default_path_project="$HOME/Documents/git/Maze_runner"


# checks if the first argument exists
# assignes the argument to the path variable
if [ $# == 1 ]; then
    path_project="$1"
    a=0
    echo "New Project Directory Assigned" 
else
    path_project="$default_path_project"
    a=1
fi

# checks whether the path to the project exists
if [ -d "$path_project" ]; then
    echo "Project Directory exists" 
else
    if [$a == 0]; then
        echo "Error: The Project Directory given by argument does not exist."
    else
        echo "Error: The Default Project Directory does not exist."
    fi
    exit 9999
fi

# safety: the working directory must be the project directory: 
# move your script to the same dir or go to it and call it from this directory
if [ "$PWD" != "$path_project" ]; then
    echo
    echo "Safety exit: the current working directory is not the folder"
    echo "of the project that has to be built"
    echo
    echo " change default directory in build_script.sh"
    echo " or go to the appropriate folder"
    echo
    exit 9999
fi

# make a build directory if it does not exist
build_dir="$path_project/build"

if [ -d "$build_dir" ]; then
    echo "Build Directory exists" 
else
    echo "Build Directory does not exist. Build Directory is made."
    mkdir $build_dir
fi

# go into the build dir and build the program
cd $build_dir

if [ "$PWD" == "$build_dir" ]; then
    echo "Entering the Build Directory to build the CMake project"
    rm -r *
    cmake ..
    make
    cd $path_project
else
    echo "Error: cannot enter Build Directory"
    cd $path_project
    exit 9999
fi
