<<<<<<< HEAD
<<<<<<< HEAD
ifeq ($(CONFIG_SDCARD_FS_XATTR),y)
EXTRA_CFLAGS += -DSDCARD_FS_XATTR
endif
=======
SDCARDFS_VERSION="0.1"

EXTRA_CFLAGS += -DSDCARDFS_VERSION=\"$(SDCARDFS_VERSION)\"
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
ifeq ($(CONFIG_SDCARD_FS_XATTR),y)
EXTRA_CFLAGS += -DSDCARD_FS_XATTR
endif
>>>>>>> 2617302... source

obj-$(CONFIG_SDCARD_FS) += sdcardfs.o

sdcardfs-y := dentry.o file.o inode.o main.o super.o lookup.o mmap.o packagelist.o derived_perm.o
<<<<<<< HEAD
<<<<<<< HEAD
sdcardfs-$(CONFIG_SDCARD_FS_XATTR) += xattr.o
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
sdcardfs-$(CONFIG_SDCARD_FS_XATTR) += xattr.o
>>>>>>> 2617302... source
