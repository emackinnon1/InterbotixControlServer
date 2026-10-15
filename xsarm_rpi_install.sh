#!/usr/bin/env bash

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

PROMPT="> "

ALL_VALID_DISTROS=('melodic' 'noetic' 'galactic' 'humble' 'jazzy' 'lyrical' 'rolling')
ROS1_VALID_DISTROS=('melodic' 'noetic')
ROS2_VALID_DISTROS=('galactic' 'humble' 'jazzy' 'lyrical' 'rolling')

BIONIC_VALID_DISTROS=('melodic')
FOCAL_VALID_DISTROS=('noetic' 'galactic')
JAMMY_VALID_DISTROS=('humble' 'rolling')
NOBLE_VALID_DISTROS=('jazzy')
RESOLUTE_VALID_DISTROS=('lyrical')

NONINTERACTIVE=false
INSTALL_PATH=~/InterbotixControlServer/workspace
DISTRO_SET_FROM_CL=false

_usage="${BOLD}USAGE: ./xsarm_rpi_install.sh [-h][-d DISTRO][-p PATH][-n]${NORM}

Install the Interbotix X-Series Arms Raspberry Pi packages and their dependencies.

Options:

  -h              Display this help message and quit

  -d DISTRO       Install the DISTRO ROS distro compatible with your Ubuntu version. See
                  'https://github.com/Interbotix/.github/blob/main/SECURITY.md' for the list of
                  supported distributions. If not given, installs the ROS 1 distro compatible with
                  your Ubuntu version, or the ROS 2 distro paired with Ubuntu 22.04, 24.04, or 26.04.

  -p PATH         Sets the absolute install location for the Interbotix workspace. If not specified,
                  the Interbotix workspace directory will default to '~/InterbotixControlServer/workspace'.

  -n              Install all packages without prompting. This is useful
                  if you're running this script in a non-interactive terminal like when building a
                  Docker image.

Examples:

  ./xsarm_rpi_install.sh ${BOLD}-h${NORM}
    This will display this help message and quit.

  ./xsarm_rpi_install.sh
    This will install the ROS distro paired with your Ubuntu version. It will prompt you to ask if
    you want to install certain packages and dependencies.

  ./xsarm_rpi_install.sh ${BOLD}-p ~/custom_ws${NORM}
    Installs the Interbotix packages under the '~/custom_ws' path."

function help() {
  # print usage
  cat << EOF
$_usage
EOF
}

# https://stackoverflow.com/a/8574392/16179107
function contains_element () {
  # check if an element is in an array
  local e match="$1"
  shift
  for e; do [[ "$e" == "$match" ]] && return 0; done
  return 1
}

function failed() {
  # Log error and quit with a failed exit code
  echo -e "${ERR}[ERROR] $1${RRE}"
  echo -e "${ERR}[ERROR] Interbotix Installation Failed!${RRE}"
  exit 1
}

function normalize_git_remote() {
  local remote="$1"
  remote="${remote%.git}"
  remote="${remote#https://}"
  remote="${remote#http://}"
  remote="${remote#git@}"
  remote="${remote/\:/\/}"
  printf '%s\n' "${remote,,}"
}

function ensure_repository() {
  local repository_path="$1"
  local branch="$2"
  local repository_url="$3"

  if [[ -e "$repository_path" || -L "$repository_path" ]]; then
    if ! git -C "$repository_path" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
      failed "'$repository_path' exists but is not a Git repository. Move it aside or choose another install path."
    fi

    local actual_remote actual_branch
    actual_remote="$(git -C "$repository_path" remote get-url origin 2>/dev/null || true)"
    actual_branch="$(git -C "$repository_path" branch --show-current 2>/dev/null || true)"
    if [[ "$(normalize_git_remote "$actual_remote")" != "$(normalize_git_remote "$repository_url")" ]]; then
      failed "'$repository_path' has unexpected origin '$actual_remote'; expected '$repository_url'. Existing sources were left unchanged."
    fi
    if [[ "$actual_branch" != "$branch" ]]; then
      failed "'$repository_path' is on branch '${actual_branch:-detached HEAD}'; expected '$branch'. Existing sources were left unchanged."
    fi
    return 0
  fi

  git clone -b "$branch" "$repository_url" "$repository_path" || failed "Failed to clone '$repository_url'."
}

function ensure_submodule() {
  local repository_path="$1"
  local submodule_path="$2"
  local full_submodule_path="$repository_path/$submodule_path"

    if [[ -e "$full_submodule_path/.git" ]] && \
      git -C "$full_submodule_path" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    return 0
  fi
  if [[ -e "$full_submodule_path" ]] && \
     [[ ! -d "$full_submodule_path" || -n "$(find "$full_submodule_path" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
    failed "Submodule path '$full_submodule_path' contains non-Git data; it was left unchanged."
  fi
  git -C "$repository_path" submodule update --init -- "$submodule_path" || \
    failed "Failed to initialize submodule '$submodule_path'."
}

function setup_shell_profile() {
  local bashrc="$HOME/.bashrc"
  local marker_start="# >>> Interbotix ROS environment >>>"
  local marker_end="# <<< Interbotix ROS environment <<<"
  local profile_tmp ros_setup workspace_setup
  local ros_setup_quoted workspace_setup_quoted

  ros_setup="/opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash"
  if [[ "$ROS_VERSION_TO_INSTALL" == "1" ]]; then
    workspace_setup="$INSTALL_PATH/devel/setup.bash"
  else
    workspace_setup="$INSTALL_PATH/install/setup.bash"
  fi
  printf -v ros_setup_quoted '%q' "$ros_setup"
  printf -v workspace_setup_quoted '%q' "$workspace_setup"

  profile_tmp="$(mktemp "$HOME/.bashrc.XXXXXX")" || failed "Could not create a temporary shell profile."
  if [[ -f "$bashrc" ]]; then
    if ! awk -v start="$marker_start" -v end="$marker_end" '
      $0 == start { if (inside) exit 2; inside = 1; next }
      $0 == end { if (!inside) exit 2; inside = 0; next }
      !inside { print }
      END { if (inside) exit 2 }
    ' "$bashrc" > "$profile_tmp"; then
      rm -f "$profile_tmp"
      failed "The managed Interbotix block in '$bashrc' is malformed; it was left unchanged."
    fi
  fi

  {
    printf '%s\n' "$marker_start"
    printf 'if [[ -f %s ]]; then source %s; fi\n' "$ros_setup_quoted" "$ros_setup_quoted"
    printf 'if [[ -f %s ]]; then source %s; fi\n' "$workspace_setup_quoted" "$workspace_setup_quoted"
    printf '%s\n' 'if [[ -z "$ROS_IP" ]]; then ROS_IP="$(hostname -I | cut -d" " -f1)"; fi'
    printf '%s\n' 'if [[ -z "$ROS_IP" ]]; then ROS_IP=127.0.0.1; fi' 'export ROS_IP' "$marker_end"
  } >> "$profile_tmp"

  if [[ ! -f "$bashrc" ]] || ! cmp -s "$profile_tmp" "$bashrc"; then
    cat "$profile_tmp" > "$bashrc" || failed "Could not update '$bashrc'."
  fi
  rm -f "$profile_tmp"
}

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  return 0
fi

function validate_distro() {
  # check if chosen distro is valid and set ROS major version
  if contains_element "$ROS_DISTRO_TO_INSTALL" "${ALL_VALID_DISTROS[@]}"; then
    if contains_element "$ROS_DISTRO_TO_INSTALL" "${ROS1_VALID_DISTROS[@]}"; then
      # Supported ROS 1 distros
      ROS_VERSION_TO_INSTALL=1
    elif contains_element "$ROS_DISTRO_TO_INSTALL" "${ROS2_VALID_DISTROS[@]}"; then
      # Supported ROS 2 distros
      ROS_VERSION_TO_INSTALL=2
    else
      # For cases where it passes the first check but somehow fails the second check
      failed "Something went wrong. ROS_DISTRO_TO_INSTALL=$ROS_DISTRO_TO_INSTALL."
    fi
    echo -e "${GRN}${BOLD}Chosen Version: ROS ${ROS_VERSION_TO_INSTALL} $ROS_DISTRO_TO_INSTALL${NORM}${OFF}"
    return 0
  else
    failed "'$ROS_DISTRO_TO_INSTALL' is not a valid ROS Distribution. Choose one of: '${ALL_VALID_DISTROS[*]}'"
  fi
}

function check_ubuntu_version() {
 # check if the chosen distribution is compatible with the Ubuntu version
  case $UBUNTU_VERSION in

    18.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${BIONIC_VALID_DISTROS[@]}"; then
        PY_VERSION=2
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    20.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${FOCAL_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    22.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${JAMMY_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    24.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${NOBLE_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    26.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${RESOLUTE_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    *)
      failed "Unsupported Ubuntu version '$UBUNTU_VERSION'; supported versions are 18.04, 20.04, 22.04, 24.04, and 26.04."
      ;;

  esac
}

function install_essential_packages() {
  # Install necessary core packages
  local pip_install_args=()
  if [[ "$UBUNTU_VERSION" == "24.04" || "$UBUNTU_VERSION" == "26.04" ]]; then
    pip_install_args=(--user --break-system-packages)
  fi

  sudo apt-get install -yq curl git
  if [ $ROS_VERSION_TO_INSTALL == 2 ]; then
    sudo apt-get install -yq python3-pip
    if [[ ${#pip_install_args[@]} -gt 0 ]]; then
      python3 -m pip install "${pip_install_args[@]}" transforms3d
    else
      sudo pip3 install transforms3d
    fi
  fi
  if [ $PY_VERSION == 2 ]; then
    sudo apt-get install -yq python-pip
    python -m pip install modern_robotics
  elif [ $PY_VERSION == 3 ]; then
    sudo apt-get install -yq python3-pip
    python3 -m pip install "${pip_install_args[@]}" modern_robotics
  else
    failed "Something went wrong. PY_VERSION='$PY_VERSION', should be 2 or 3."
  fi
}

function install_ros1() {
  # Install ROS 1
  if [ "$(dpkg-query -W -f='${Status}' ros-"$ROS_DISTRO_TO_INSTALL"-desktop-full 2>/dev/null | grep -c "ok installed")" -eq 0 ]; then
    echo -e "${GRN}Installing ROS 1 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    local ros1_repository_entry
    ros1_repository_entry="deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"
    if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]] || \
       [[ "$(cat /etc/apt/sources.list.d/ros-latest.list)" != "$ros1_repository_entry" ]]; then
      printf '%s\n' "$ros1_repository_entry" | sudo tee /etc/apt/sources.list.d/ros-latest.list > /dev/null
    fi
    if ! sudo apt-key finger 2>/dev/null | grep -q 'AB17 C654'; then
      sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    fi
    sudo apt-get update
    sudo apt-get install -yq ros-"$ROS_DISTRO_TO_INSTALL"-desktop-full
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop-full is already installed!"
  fi
  source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash

  # Install rosdep and other necessary tools
  if [ $PY_VERSION == 2 ]; then
    sudo apt-get install -yq        \
      python-rosdep                 \
      python-rosinstall             \
      python-rosinstall-generator   \
      python-wstool                 \
      build-essential
  elif [ $PY_VERSION == 3 ]; then
    sudo apt-get install -yq        \
      python3-rosdep                \
      python3-rosinstall            \
      python3-rosinstall-generator  \
      python3-wstool                \
      build-essential
  fi

  # Initialize rosdep sources only when they have not been initialized yet.
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || failed "Failed to initialize rosdep sources."
  fi

  # Update local rosdep database, including EoL distros
  rosdep update --include-eol-distros

  ensure_repository "$INSTALL_PATH/src/interbotix_ros_core" "$ROS_DISTRO_TO_INSTALL" "https://github.com/Interbotix/interbotix_ros_core.git"
  ensure_repository "$INSTALL_PATH/src/interbotix_ros_manipulators" "$ROS_DISTRO_TO_INSTALL" "https://github.com/Interbotix/interbotix_ros_manipulators.git"
  ensure_repository "$INSTALL_PATH/src/interbotix_ros_toolboxes" "$ROS_DISTRO_TO_INSTALL" "https://github.com/Interbotix/interbotix_ros_toolboxes.git"

  # Reconcile workspace configuration and build on every run so partial or stale builds recover.
  cd "$INSTALL_PATH/src" || failed "Could not enter the workspace source directory '$INSTALL_PATH/src'."
  rm -f interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE      \
    interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE   \
    interbotix_ros_toolboxes/interbotix_rpi_toolbox/CATKIN_IGNORE     \
    interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE      \
    interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE
  cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk || failed "Interbotix SDK directory is missing."
  if [[ ! -f /etc/udev/rules.d/99-interbotix-udev.rules ]] || \
     ! cmp -s 99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules; then
    sudo install -m 0644 99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
  fi
  cd "$INSTALL_PATH" || failed "Could not enter the workspace '$INSTALL_PATH'."
  rosdep install --from-paths src --ignore-src -r -y
  if catkin_make; then
    echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
    source "$INSTALL_PATH"/devel/setup.bash
  else
    failed "Failed to build Interbotix Arm ROS Packages."
  fi
}

function install_ros2() {
  # Install ROS 2
  local interbotix_branch="$ROS_DISTRO_TO_INSTALL"
  if [[ "$ROS_DISTRO_TO_INSTALL" == "lyrical" ]]; then
    interbotix_branch="rolling"
    echo -e "${BLU}${BOLD}Lyrical is experimental: building Interbotix packages from their rolling branches.${NORM}${OFF}"
  fi

  if [ "$(dpkg-query -W -f='${Status}' ros-"$ROS_DISTRO_TO_INSTALL"-desktop 2>/dev/null | grep -c "ok installed")" -eq 0 ]; then
    echo -e "${GRN}Installing ROS 2 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    sudo apt-get install -yq      \
      software-properties-common  \
      gnupg
    sudo add-apt-repository -y universe
    if [[ ! -s /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi
    local ros_repository_entry
    ros_repository_entry="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main"
    if [[ ! -f /etc/apt/sources.list.d/ros2.list ]] || \
       [[ "$(cat /etc/apt/sources.list.d/ros2.list)" != "$ros_repository_entry" ]]; then
      printf '%s\n' "$ros_repository_entry" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    fi
    sudo apt-get update
    sudo apt-get install -yq ros-"$ROS_DISTRO_TO_INSTALL"-desktop
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop is already installed!"
  fi
  source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash

  # Install rosdep and other necessary tools
  sudo apt-get install -yq            \
    python3-rosdep                    \
    build-essential                   \
    python3-colcon-common-extensions || failed "Failed to install ROS 2 build dependencies."

  # Initialize rosdep sources only when they have not been initialized yet.
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || failed "Failed to initialize rosdep sources."
  fi

  # Update local rosdep database, including EoL distros
  rosdep update --include-eol-distros

  ensure_repository "$INSTALL_PATH/src/interbotix_ros_core" "$interbotix_branch" "https://github.com/Interbotix/interbotix_ros_core.git"
  ensure_repository "$INSTALL_PATH/src/interbotix_ros_manipulators" "$interbotix_branch" "https://github.com/Interbotix/interbotix_ros_manipulators.git"
  ensure_repository "$INSTALL_PATH/src/interbotix_ros_toolboxes" "$interbotix_branch" "https://github.com/Interbotix/interbotix_ros_toolboxes.git"
  ensure_repository "$INSTALL_PATH/src/moveit_visual_tools" "ros2" "https://github.com/ros-planning/moveit_visual_tools.git"

  # Reconcile workspace configuration and build on every run so partial or stale builds recover.
  cd "$INSTALL_PATH/src" || failed "Could not enter the workspace source directory '$INSTALL_PATH/src'."
  rm -f                                                                                               \
    interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE                                          \
    interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE      \
    interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE \
    interbotix_ros_toolboxes/interbotix_rpi_toolbox/COLCON_IGNORE
  ensure_submodule "$INSTALL_PATH/src/interbotix_ros_core" "interbotix_ros_xseries/dynamixel_workbench_toolbox"
  ensure_submodule "$INSTALL_PATH/src/interbotix_ros_core" "interbotix_ros_xseries/interbotix_xs_driver"
  cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk || failed "Interbotix SDK directory is missing."
  if [[ ! -f /etc/udev/rules.d/99-interbotix-udev.rules ]] || \
     ! cmp -s 99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules; then
    sudo install -m 0644 99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
  fi
  cd "$INSTALL_PATH" || failed "Could not enter the workspace '$INSTALL_PATH'."
  rosdep install --from-paths src --ignore-src -r -y
  if colcon build; then
    echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
    source "$INSTALL_PATH"/install/setup.bash
  else
    failed "Failed to build Interbotix Arm ROS Packages."
  fi
}

# parse command line arguments
while getopts 'hd:p:n' OPTION;
do
  case "$OPTION" in
    h) help && exit 0;;
    n) NONINTERACTIVE=true;;
    d) ROS_DISTRO_TO_INSTALL="$OPTARG" && DISTRO_SET_FROM_CL=true;;
    p) INSTALL_PATH="$OPTARG";;
    *) echo "Unknown argument $OPTION" && help && exit 0;;
  esac
done
shift "$((OPTIND -1))"

if [[ "$INSTALL_PATH" == "~" ]]; then
  INSTALL_PATH="$HOME"
elif [[ "$INSTALL_PATH" == "~"/* ]]; then
  INSTALL_PATH="$HOME/${INSTALL_PATH:2}"
fi
if [[ "$INSTALL_PATH" != /* ]]; then
  failed "Install path '$INSTALL_PATH' must be absolute (or begin with ~/)."
fi
if [[ -e "$INSTALL_PATH" && ! -d "$INSTALL_PATH" ]]; then
  failed "Install path '$INSTALL_PATH' exists and is not a directory."
fi

if ! command -v lsb_release &> /dev/null; then
  sudo apt-get update
  sudo apt-get install -yq lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

# set default ROS distro before reading clargs
if [ "$DISTRO_SET_FROM_CL" = false ]; then
  if [ "$UBUNTU_VERSION" == "18.04" ]; then
    ROS_DISTRO_TO_INSTALL="melodic"
  elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    ROS_DISTRO_TO_INSTALL="noetic"
  elif [ "$UBUNTU_VERSION" == "22.04" ]; then
    ROS_DISTRO_TO_INSTALL="humble"
  elif [ "$UBUNTU_VERSION" == "24.04" ]; then
    ROS_DISTRO_TO_INSTALL="jazzy"
  elif [ "$UBUNTU_VERSION" == "26.04" ]; then
    ROS_DISTRO_TO_INSTALL="lyrical"
  else
    echo -e "${BOLD}${RED}Unsupported Ubuntu version: $UBUNTU_VERSION.${NORM}${OFF}"
    failed "Supported Ubuntu versions are 18.04 Bionic, 20.04 Focal, 22.04 Jammy, 24.04 Noble, and 26.04 Resolute."
  fi
fi

validate_distro
check_ubuntu_version

if [ "$NONINTERACTIVE" = false ]; then
  if [[ "$ROS_DISTRO_TO_INSTALL" == 'rolling' ]]; then
    echo -e "${BLU}${BOLD}ROS 2 Rolling is not officially supported and full functionality is not guaranteed. Do you wish to continue anyways?\n$PROMPT${NORM}${OFF}\c"
    read -r resp
    if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
      :
    else
      help && exit 0
    fi
  fi

  echo -e "${BLU}${BOLD}RASPBERRY PI INSTALLATION SUMMARY:"
  echo -e "\tROS Distribution:              ROS ${ROS_VERSION_TO_INSTALL} ${ROS_DISTRO_TO_INSTALL}"
  echo -e "\tInstallation path:             ${INSTALL_PATH}"
  echo -e "\nIs this correct?\n${PROMPT}${NORM}${OFF}\c"
  read -r resp

  if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
    :
  else
    help && exit 0
  fi
fi

echo -e "\n\n"
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}            Starting installation!            ${NORM}${OFF}"
echo -e "${GRN}${BOLD}   This process may take around 15 Minutes!   ${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo -e "\n\n"

sleep 4
start_time="$(date -u +%s)"

# Refresh package metadata and install declared dependencies without upgrading/removing the host.
sudo apt-get update

install_essential_packages

mkdir -p "$INSTALL_PATH"/src

shopt -s extglob

if [[ $ROS_VERSION_TO_INSTALL == 1 ]]; then
  install_ros1
elif [[ $ROS_VERSION_TO_INSTALL == 2 ]]; then
  install_ros2
else
  failed "Something went wrong. ROS_VERSION_TO_INSTALL=$ROS_VERSION_TO_INSTALL."
fi

setup_shell_profile

shopt -u extglob

end_time="$(date -u +%s)"
elapsed="$((end_time-start_time))"

echo -e "${GRN}Installation complete, took $elapsed seconds in total.${OFF}"
echo -e "${GRN}NOTE: Remember to reboot the computer before using the robot!${OFF}"