#ifndef TIMING_H
#define TIMING_H

#include <chrono>


struct TimingData {

	unsigned frameNumber;
	double lastFrameTimestamp;
	double lastFrameDuration;
	bool isPaused;
	double averageFrameDuration;
	double fps;

	static double getTime();
	static TimingData& get();
	static void update();
	static void init();
	static void deinit();

	private:

	TimingData(){};
	TimingData(const TimingData& t){};
	//TimingData& operator= (const TimingData& t){};

};


#endif