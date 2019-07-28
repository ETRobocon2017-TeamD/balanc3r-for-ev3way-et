PROGRAM = runner.so
OBJS = runner.o
C_SRC = runner.c

.SUFFIXES: .pyx .c .so

.PHONY: all
all: $(PROGRAM)

runner.so: runner.c
	docker run -v /Users/yoshinori/src/github.com/ETRobocon2017-TeamD/balanc3r-for-ev3way-et:/host ev3jsrs /bin/bash /host/src/cross_compile.sh
	make depclean

runner.c: runner.pyx
	cython runner.pyx

.PHONY: clean
clean:
	$(RM) $(PROGRAM) $(OBJS) $(C_SRC)

.PHONY: depclean
depclean:
	$(RM) $(OBJS) $(C_SRC)
