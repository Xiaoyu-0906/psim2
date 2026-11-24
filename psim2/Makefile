ifeq ($(origin CXX),default)
ifeq ($(shell uname -s),Darwin)
CXX := clang++
else
CXX := g++
endif
endif
.PHONY: all clean
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wno-unused-parameter
SRC := net.cpp lateral.cpp leaderinfo.cpp lanes.cpp link.cpp move.cpp cfmodels.cpp tls.cpp stops.cpp insert.cpp transfer.cpp lc.cpp sublane.cpp edit.cpp demand.cpp pedrouter.cpp peds.cpp routing.cpp rerouter.cpp parking.cpp rail.cpp jcollision.cpp transport.cpp intermodal.cpp trips.cpp sim.cpp main.cpp
HDR := common.hpp geom.hpp cf.hpp net.hpp ped.hpp pedrouter.hpp model.hpp threadpool.hpp railtypes.hpp intermodal.hpp lc.hpp sublanequery.hpp
# object files per source (make -j compiles them in parallel; every object depends on all headers)
OBJ := $(SRC:%.cpp=build/%.o)

all: psim2

build/%.o: %.cpp $(HDR)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -pthread -c -o $@ $<

psim2: $(OBJ)
	$(CXX) $(CXXFLAGS) -pthread -o $@ $(OBJ) -lz

psim2-tsan: $(SRC) $(HDR)
	$(CXX) -O1 -g -std=c++17 -fsanitize=thread -pthread -o $@ $(SRC) -lz

psim2-debug: $(SRC) $(HDR)
	$(CXX) -O0 -g -std=c++17 -fsanitize=address,undefined -pthread -o $@ $(SRC) -lz

clean:
	rm -rf build psim2 psim2-tsan psim2-debug *.dSYM
