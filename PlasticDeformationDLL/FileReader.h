#pragma once

const vector<string> splitString(const string& str, const char& delim) {
	string buffer{ "" };
	vector<string> result;

	for (auto character : str) {
		if (character != delim)
			buffer += character;
		else
			if (character == delim && buffer != "") {
				result.push_back(buffer); buffer = "";
			}
	}
	if (buffer != "") result.push_back(buffer);

	return result;
}

const bool startsWith(const string &line, const string &delimiter) {
	if (line.substr(0, delimiter.size()) == delimiter)
		return true;
	return false;
}

bool fileExists(string filename) {
	std::ifstream ifile(filename);
	return (bool)ifile;
}

enum FileReadMode {
	NONE = 0,
	BC_COORD = 1,
	BC_TETID = 2,
	SURF_TO_TET_VERT_MAP = 3,
	DIST_CONST = 4,
	VOL_CONST = 5
};

bool readTetMeshFile(const string &name, 
	vector<int> &surfaceToTetVertMap, 
	vector<vec4> &barycentricCoords,
	vector<ivec4> &barycenricTetIds, 
	VolumeConstraintData &volConstData,
	DistanceConstraintData &distConstData) {

	if (!fileExists(name)) {
		return false;
	}
	FileReadMode frm = NONE;
	string line;
	ifstream inputFileStream(name);
	

	while (getline(inputFileStream, line)) {
		// check for new line read mode
		string readModeIdentifier = line.substr(0, 3);
		if (readModeIdentifier == "s2t") {
			frm = SURF_TO_TET_VERT_MAP;
		} else if (readModeIdentifier == "bcc") {
			frm = BC_COORD;
		} else if(readModeIdentifier == "bct") {
			frm = BC_TETID;
		} else if(readModeIdentifier == "dcd") {
			frm = DIST_CONST;
		} else if(readModeIdentifier == "vcd") {
			frm = VOL_CONST;
		}
		// interpret line
		vector<string> v;
		switch (frm) {
		case SURF_TO_TET_VERT_MAP:		//line: 0
			surfaceToTetVertMap.push_back(stoi(line));
			break;
		case BC_COORD:		//line: 0.0 0.0 0.0 0.0
			v = splitString(line, ' ');
			barycentricCoords.push_back(vec4(stoi(v[0]), stoi(v[1]), stoi(v[2]), stoi(v[3])));
			break;
		case BC_TETID:		//line: 0 0 0 0
			v = splitString(line, ' ');
			barycenricTetIds.push_back(ivec4(stoi(v[0]), stoi(v[1]), stoi(v[2]), stoi(v[3])));
			break;
		case DIST_CONST:		//line: 0
			//surfaceToTetVertMap.push_back(stoi(line));
			break;
		case VOL_CONST:		//line: 0
			//surfaceToTetVertMap.push_back(stoi(line));
			break;
		}


	}
	return false;
}