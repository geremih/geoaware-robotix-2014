CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = Landmark.cpp  Map.cpp  MapProcessor.cpp TJunction.cpp Waypoint.cpp LiveSymbolDetector.cpp Locomotor.cpp Main.cpp
HEADERS = *.h

all : $(SOURCES) $(HEADERS)
	g++ -g $(CFLAGS) -o GeoAware $(SOURCES) $(LIBS) -lncurses

locomotor: Locomotor.cpp Locotest.cpp
	g++ -o Locomotor Locomotor.cpp Locotest.cpp  $(LIBS) -lncurses

soundCheck: SoundPlayer.cpp
	g++ -g SoundCheck.cpp SoundPlayer.cpp -lSDL -lSDL_mixer


clean:
	rm GeoAware Locomotor 
