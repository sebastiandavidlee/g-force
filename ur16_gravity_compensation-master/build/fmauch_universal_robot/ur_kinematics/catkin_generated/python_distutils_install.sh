#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/edg-sandroom/catkin_ws/src/fmauch_universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/edg-sandroom/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/edg-sandroom/catkin_ws/install/lib/python2.7/dist-packages:/home/edg-sandroom/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/edg-sandroom/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/edg-sandroom/catkin_ws/src/fmauch_universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/edg-sandroom/catkin_ws/build/fmauch_universal_robot/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/edg-sandroom/catkin_ws/install" --install-scripts="/home/edg-sandroom/catkin_ws/install/bin"
