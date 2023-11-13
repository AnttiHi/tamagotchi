# invoke SourceDir generated makefile for empty.pem3
empty.pem3: .libraries,empty.pem3
.libraries,empty.pem3: package/cfg/empty_pem3.xdl
	$(MAKE) -f /home/linuxlite/Desktop/empty_CC2650STK_TI_2023/src/makefile.libs

clean::
	$(MAKE) -f /home/linuxlite/Desktop/empty_CC2650STK_TI_2023/src/makefile.libs clean

