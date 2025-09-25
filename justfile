set shell := ["bash", "-c"]

ros_distro := env_var('ROS_DISTRO')

default:
  @just --list

alias dep := deps
alias b := build
alias t := test
alias d := doc

_cd:
  @cd {{justfile_directory()}}

# install dependencies
deps: _cd
  vcs import --input build_depends.repos --recursive .
  sudo apt update
  rosdep update --rosdistro {{ros_distro}}
  rosdep install --from-paths . --ignore-src --rosdistro {{ros_distro}} -y

# build packages
build: _cd
  colcon build --symlink-install

# test packages
test packages: _cd
  source install/setup.bash && \
  colcon test --event-handlers console_direct+ --return-code-on-test-failure --packages-select {{packages}}

# generate documentation
doc packages: _cd
  source install/setup.bash && \
  colcon list --paths-only | grep -E "($(echo {{packages}} | tr ' ' '|'))$" | xargs -I{} rosdoc2 build -p {}


# clean workspace
clean: _cd
  -rm -rf build install log docs_build docs_output cross_reference