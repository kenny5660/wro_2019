#!/bin/bash
NAME="wro19 application"
DAEMON=/home/admin/wro2019/Robot_exe
ARGS="1"
USER=admin
PIDFILE=/var/run/cRioTestC.pid
do_start() {
    /sbin/start-stop-daemon --start --pidfile $PIDFILE \
        --make-pidfile --background \
        --chuid $USER --exec $DAEMON $ARGS
}
do_stop() {
    /sbin/start-stop-daemon --stop --pidfile $PIDFILE --verbose
}

case "$1" in
  start)
    echo "Starting $NAME"
    do_start
    ;;
  stop)
    echo "Stopping $NAME"
    do_stop
    ;;
  restart)
    echo "Restarting $NAME"
    do_stop
    do_start
    ;;
  *)
    echo "Usage: $0 {start|stop|restart}"
    exit 1
    ;;
esac
exit 0