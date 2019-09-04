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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jesse/Documents/mx2TurtleSlam/src/turtlebot3/turtlebot3_example"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jesse/Documents/mx2TurtleSlam/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jesse/Documents/mx2TurtleSlam/install/lib/python2.7/dist-packages:/home/jesse/Documents/mx2TurtleSlam/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jesse/Documents/mx2TurtleSlam/build" \
    "/usr/bin/python" \
    "/home/jesse/Documents/mx2TurtleSlam/src/turtlebot3/turtlebot3_example/setup.py" \
    build --build-base "/home/jesse/Documents/mx2TurtleSlam/build/turtlebot3/turtlebot3_example" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/jesse/Documents/mx2TurtleSlam/install" --install-scripts="/home/jesse/Documents/mx2TurtleSlam/install/bin"
