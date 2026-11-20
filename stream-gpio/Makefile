CXXFLAGS ?= -O2 -Wall -Wextra

stream-gpio: stream.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -std=c++17 $(LDFLAGS) -pthread -o $@ $< $(LDLIBS)

.PHONY: clean
clean:
	rm -f stream-gpio
