CC		= $(CROSS_COMPILE)gcc
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 2617302... source
BUILD_OUTPUT	:= $(PWD)
PREFIX		:= /usr
DESTDIR		:=

<<<<<<< HEAD
=======
BUILD_OUTPUT	:= $(CURDIR)
PREFIX		:= /usr
DESTDIR		:=

ifeq ("$(origin O)", "command line")
	BUILD_OUTPUT := $(O)
endif

>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source
turbostat : turbostat.c
CFLAGS +=	-Wall
CFLAGS +=	-DMSRHEADER='"../../../../arch/x86/include/uapi/asm/msr-index.h"'

%: %.c
	@mkdir -p $(BUILD_OUTPUT)
	$(CC) $(CFLAGS) $< -o $(BUILD_OUTPUT)/$@

.PHONY : clean
clean :
	@rm -f $(BUILD_OUTPUT)/turbostat

install : turbostat
	install -d  $(DESTDIR)$(PREFIX)/bin
	install $(BUILD_OUTPUT)/turbostat $(DESTDIR)$(PREFIX)/bin/turbostat
	install -d  $(DESTDIR)$(PREFIX)/share/man/man8
	install turbostat.8 $(DESTDIR)$(PREFIX)/share/man/man8
