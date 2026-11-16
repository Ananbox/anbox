#!/system/bin/sh
# $1 data dir, $2 proot binary.
DIR=$1
PROOT=$2

rm $DIR/rootfs/data/system.log
rm -rf $DIR/rootfs/dev/socket
mkdir -p $DIR/rootfs/dev/socket
rm -rf $DIR/rootfs/dev/__properties__
mkdir -p $DIR/rootfs/dev/__properties__
rm $DIR/rootfs/dev/kmsg
rm $DIR/rootfs/dev/pmsg0
touch $DIR/rootfs/dev/kmsg
touch $DIR/rootfs/dev/pmsg0
mkdir -p $DIR/rootfs/data/media/0
mkdir -p $DIR/rootfs/data/system_ce/0
mkdir -p $DIR/rootfs/data/misc_ce/0
kernel_version=$(uname -r | cut -d- -f1)
major=$(echo $kernel_version | cut -d. -f1)
minor=$(echo $kernel_version | cut -d. -f2)
# disable seccomp in kernel < 4.8 for compatibility
if [ "$major" -lt 4 ] || { [ "$major" -eq 4 ] && [ "$minor" -lt 8 ]; }; then
    export PROOT_NO_SECCOMP=1
fi
export PATH=/sbin:/system/bin:/system/sbin:/system/xbin:/system/vendor/bin
export ANDROID_ASSETS=/assets
export ANDROID_DATA=/data
export ANDROID_ROOT=/system
export ANDROID_STORAGE=/storage
export ASEC_MOUNTPOINT=/mnt/asec
export EXTERNAL_STORAGE=/sdcard
export PROOT_TMP_DIR=$DIR/tmp
$PROOT --kill-on-exit -v 255 -r $DIR/rootfs -0 -w / -b /dev -b /proc -b /sys -b $DIR/rootfs/dev/kmsg:/dev/kmsg -b $DIR/rootfs/dev/pmsg0:/dev/pmsg0 -b $DIR/rootfs/system/vendor:/vendor -b $DIR/rootfs/dev/__properties__:/dev/__properties__ -b $DIR/rootfs/dev/socket:/dev/socket -b /dev/binder:/dev/binder -b /dev/ashmem:/dev/ashmem -b $DIR/qemu_pipe:/dev/qemu_pipe -b $DIR/rootfs/dev/input:/dev/input -b $DIR/rootfs/mnt/user/0:/storage/self /init >> $DIR/proot.log 2>&1 &
PROOT_PID=$!

# proot ignores SIGTERM/SIGINT (tracee/event.c sets them to SIG_IGN), so the
# only way to make it tear the guest down is SIGQUIT, whose --kill-on-exit
# handler kills every tracee before exiting. SIGTERM is caught too so a stray
# `kill <pid>` reaches the same path.
forward_quit() { kill -QUIT $PROOT_PID 2>/dev/null; }
trap forward_quit QUIT
trap forward_quit TERM

wait $PROOT_PID
