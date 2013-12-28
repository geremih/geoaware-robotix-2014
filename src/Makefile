CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = ProcessMap.cpp Landmark.cpp
HEADERS = GeoAware.h

#% : %.cpp
#	g++ $(CFLAGS) -o $@ $< $(LIBS)

all : $(SOURCES) $(HEADERS)
	g++ -g $(CFLAGS) -o GeoAware $(SOURCES) $(LIBS)
