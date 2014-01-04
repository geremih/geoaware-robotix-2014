CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = *.cpp
HEADERS = *.h

all : $(SOURCES) $(HEADERS)
	g++ -g $(CFLAGS) -o GeoAware $(SOURCES) $(LIBS)
