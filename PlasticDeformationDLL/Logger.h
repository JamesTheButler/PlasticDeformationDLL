#pragma once
#include <ctime>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>

using namespace std;

namespace logger {
	string logFilePath;
	string currentLogPath;
	bool isActive=false;

	void setActive(bool isActive) {
		logger::isActive = isActive;
	}

	string getTime(bool usePunctuation) {
		time_t now = time(0);
		struct tm  tstruct;
		char  buf[80];
		tstruct = *localtime(&now);
		if (usePunctuation)
			strftime(buf, sizeof(buf), "%Y.%m.%d %H:%M:%S", &tstruct);
		else
			strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);

		cout << string(buf) << '\n';
		return string(buf);
	}

	void log(const string &text) {
		if (!isActive)
			return;
		string time = getTime(true);
		ofstream ofs1(logFilePath.c_str(), ios_base::out | ios_base::app);
		ofs1 << time << '\t' << text << '\n';
		ofs1.close();

		ofstream ofs2(currentLogPath.c_str(), ios_base::out | ios_base::app);
		ofs2 << time << '\t' << text << '\n';
		ofs2.close();

	}

	void logError(const string &text) {
		if (!isActive)
			return;
		log("--!ERROR!--" + text);
	}

	void setFilePath(string path) {
		if (!isActive)
			return;
		logFilePath = path + getTime(false) + ".log";
		currentLogPath = path+"current.log";

		std::ofstream ofs;
		ofs.open(currentLogPath, std::ofstream::out | std::ofstream::trunc);
		ofs.close();

		log("--Setup Log");
	}


}