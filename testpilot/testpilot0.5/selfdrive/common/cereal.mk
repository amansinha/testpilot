UNAME_M ?= $(shell uname -m)
UNAME_S ?= $(shell uname -s)

CEREAL_CFLAGS = -I/usr/local/include

CEREAL_CXXFLAGS = -I/usr/local/include/capnp/capnp-cpp/include
CEREAL_LIBS = -L/usr/local/lib/ \
              -l:libcapnp.a -l:libkj.a -l:libcapnp_c.a

CEREAL_OBJS = ../../cereal/gen/c/log.capnp.o ../../cereal/gen/c/car.capnp.o

log.capnp.o: ../../cereal/gen/cpp/log.capnp.c++
	@echo "[ CXX ] $@"
	$(CXX) $(CXXFLAGS) $(CEREAL_CXXFLAGS) \
           -c -o '$@' '$<'

car.capnp.o: ../../cereal/gen/cpp/car.capnp.c++
	@echo "[ CXX ] $@"
	$(CXX) $(CXXFLAGS) $(CEREAL_CXXFLAGS) \
           -c -o '$@' '$<'
