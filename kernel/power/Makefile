
ccflags-$(CONFIG_PM_DEBUG)	:= -DDEBUG

obj-y				+= qos.o
obj-$(CONFIG_PM)		+= main.o
obj-$(CONFIG_VT_CONSOLE_SLEEP)	+= console.o
obj-$(CONFIG_FREEZER)		+= process.o
obj-$(CONFIG_SUSPEND)		+= suspend.o
obj-$(CONFIG_PM_TEST_SUSPEND)	+= suspend_test.o
obj-$(CONFIG_HIBERNATION)	+= hibernate.o snapshot.o swap.o user.o \
				   block_io.o
<<<<<<< HEAD
<<<<<<< HEAD
=======
obj-$(CONFIG_POWERSUSPEND)	+= powersuspend.o
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
obj-$(CONFIG_PM_AUTOSLEEP)	+= autosleep.o
obj-$(CONFIG_PM_WAKELOCKS)	+= wakelock.o
obj-$(CONFIG_SUSPEND_TIME)	+= suspend_time.o

<<<<<<< HEAD
<<<<<<< HEAD
obj-$(CONFIG_QUICK_WAKEUP)	+= quickwakeup.o
=======
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
obj-$(CONFIG_QUICK_WAKEUP)	+= quickwakeup.o
>>>>>>> 2617302... source
obj-$(CONFIG_MAGIC_SYSRQ)	+= poweroff.o

obj-$(CONFIG_SUSPEND)	+= wakeup_reason.o
