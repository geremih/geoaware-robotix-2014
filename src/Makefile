CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = Main.cpp Map.cpp Landmark.cpp
HEADERS = GeoAware.h Map.h Landmark.h

#% : %.cpp
#	g++ $(CFLAGS) -o $@ $< $(LIBS)

all : $(SOURCES) $(HEADERS)
	g++ -g $(CFLAGS) -o GeoAware $(SOURCES) $(LIBS)
