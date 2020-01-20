#pragma once
#include <ctime>
#include <iostream>
#include <chrono>
namespace logger {
	string logFilePath;
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
		ofstream ofs(logFilePath.c_str(), std::ios_base::out | std::ios_base::app);
		ofs << time << '\t' << text << '\n';
		ofs.close();
	}

	void logError(const string &text) {
		log("--!ERROR!--" + text);
	}

	void setFilePath(string path) {
		logFilePath = path + getTime(false) + ".log";
		log("--Setup Log");
	}
}