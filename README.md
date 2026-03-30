# phoenix-rtos-filesystems

> Fork warning:
> This fork contains AI-generated changes for the Phoenix RTOS Raspberry Pi
> port. These changes have not been fully reviewed and have not been fully
> tested.

This repository contains the source for the Phoenix-RTOS file servers. File servers work on the user-level
and implement support for filesystems. File servers are imported into this repository from Phoenix-RTOS 2
kernel and need to be reimplemented. These drivers are marked with _ prefix.

To compile driver enter into the file server directory and type:

	$ make clean
	$ make

This work is licensed under a BSD license. See the LICENSE file for details.
