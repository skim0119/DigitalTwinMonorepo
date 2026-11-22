CXXFLAGS ?= -O2 -Wall -Wextra

stream-gpio: stream.cpp hx711.cpp gpio.hpp hx711.hpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -std=c++17 $(LDFLAGS) -pthread -o $@ $(filter %.cpp,$^) $(LDLIBS)
		# CPPFLAGS="-I$(brew --prefix boost)/include"  # For mac, brew installed boost link

.PHONY: clean
clean:
	rm -f stream-gpio
