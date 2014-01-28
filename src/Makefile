CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = Landmark.cpp  Map.cpp  MapProcessor.cpp TJunction.cpp Waypoint.cpp LiveSymbolDetector.cpp Locomotor.cpp
HEADERS = *.h

all : $(SOURCES) $(HEADERS)
	g++ -g $(CFLAGS) -o GeoAware $(SOURCES) $(LIBS)

locomotor: Locomotor.cpp Locotest.cpp
	g++ -o Locomotor Locomotor.cpp Locotest.cpp -lncurses

clean:
	rm GeoAware Locomotor 
