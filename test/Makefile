CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = *.cpp
HEADERS = *.h

SmoothedHoughLines: SmoothedHoughLines.cpp
	g++ -g $(CFLAGS) -o SmoothedHoughLines SmoothedHoughLines.cpp $(LIBS)
	./SmoothedHoughLines
