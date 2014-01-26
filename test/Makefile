CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
SOURCES = *.cpp
HEADERS = *.h

SmoothedHoughLines: SmoothedHoughLines.cpp
	g++ -g $(CFLAGS) -o SmoothedHoughLines SmoothedHoughLines.cpp $(LIBS)
	./SmoothedHoughLines

FindWaypoints: FindWaypoints.cpp
	g++ -g $(CFLAGS) -o FindWaypoints FindWaypoints.cpp $(LIBS)


LiveSymbolDetectorTest: LiveSymbolDetectorTest.cpp
	g++ -g $(CFLAGS) -o LiveSymbolDetectorTest LiveSymbolDetectorTest.cpp $(LIBS)

LaneDetect: LaneDetect.cpp 
	g++ -g $(CFLAGS) -o LaneDetect LaneDetect.cpp $(LIBS)

