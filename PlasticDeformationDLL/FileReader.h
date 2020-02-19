#pragma once
#include <vector>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtx/string_cast.hpp>
#include "Logger.h"
using namespace std;
using namespace glm;

namespace fileReader {
	enum FileReadMode {
		NONE = 0,
		BC_COORD = 1,
		BC_TETID = 2,
		SURF_TO_TET_VERT_MAP = 3,
		TET_MESH_VERT = 4,
		TET_MESH_TET = 5,
		TET_MESH_SURF_VERT = 6,
		TET_MESH_SURF_TRI = 7,
	};

	// Splits up a string into a list of tokens via a given delimiter.
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

	// Checks if a given string starts with a given delimiter.
	bool startsWith(const string &line, const string &delimiter) {
		if (line.substr(0, delimiter.size()) == delimiter)
			return true;
		return false;
	}

	// Checks if a file at a given fileName exists.
	bool fileExists(string path) {
		ifstream ifile(path);
		return ifile.good();
	}

	// Parses a given .mesh file into a list of tet mesh vertices and tet mesh tetrahedra.
	void parseFile_obj_mesh(const string& fileName, vector<vec3>& tetMeshVerts, vector<ivec4>& tetMeshTets) {

		logger::log("--parseFile_obj_mesh");
		ifstream inputFileStream(fileName);
		string line;

		tetMeshVerts.resize(0);
		tetMeshTets.resize(0);

		string vertexDelimiter = "Vertices";
		string tetrahedronDelimiter = "Tetrahedra";
		string triangleDelimiter = "Triangles";
		string endDelimiter = "End";
		int readMode = NONE;
		bool isNextLineCount = false;

		while (getline(inputFileStream, line)) {
			switch (readMode) {
			case NONE:
				if (startsWith(line, vertexDelimiter)) {
					readMode = TET_MESH_VERT;
					isNextLineCount = true;
				}
				if (startsWith(line, tetrahedronDelimiter)) {
					readMode = TET_MESH_TET;
					isNextLineCount = true;
				}
				break;
			case TET_MESH_VERT:
				if (startsWith(line, triangleDelimiter)) {
					readMode = NONE;
				}
				else if (isNextLineCount) {
					tetMeshVerts.reserve(stoi(line));
					isNextLineCount = false;
				}
				else {
					vector<string> v{ splitString(line, ' ') };
					tetMeshVerts.push_back(vec3(stof(v[0]), stof(v[1]), stof(v[2])));
				}
				break;
			case TET_MESH_TET:
				if (startsWith(line, endDelimiter)) {
					readMode = NONE;
				}
				else if (isNextLineCount) {
					tetMeshTets.reserve(stoi(line));
					isNextLineCount = false;
				}
				else {
					vector<string> v{ splitString(line, ' ') };
					tetMeshTets.push_back(ivec4(stoi(v[0]) - 1, stoi(v[1]) - 1, stoi(v[2]) - 1, stoi(v[3]) - 1));		// -1 because indexing in file starts at 1 (not 0)
				}
				break;
			}
		}
		logger::log("\t -file parsed .. tet mesh vert count: " + to_string(tetMeshVerts.size())+", tet mesh tets: " + to_string(tetMeshTets.size()));
	}

	// Parses a given .tetmesh file into barycentric coordinates and a mapping of surface vertices to tet mesh vertices.
	void parseFile_tetmesh(const string &fileName,
		vector<int> &surfaceToTetMeshVertexMap,
		vector<vec4>& barycentricCoords,
		vector<int>& barycentricTetrahedronIds) {

		logger::log("-- parseFile_tetmesh");

		FileReadMode frm = NONE;
		string line;
		ifstream inputFileStream(fileName);
		while (getline(inputFileStream, line)) {
			// check for new line read mode
			string readModeIdentifier = line.substr(0, 3);
			if (readModeIdentifier == "s2t") {
				frm = SURF_TO_TET_VERT_MAP;
			}
			else if (readModeIdentifier == "bcc") {
				frm = BC_COORD;
			}
			else if (readModeIdentifier == "bct") {
				frm = BC_TETID;
			}
			else {
				// interpret line
				switch (frm) {
				case SURF_TO_TET_VERT_MAP:		//line: 0
					surfaceToTetMeshVertexMap.push_back(stoi(line));
					break;
				case BC_COORD:		//line: 0.0 0.0 0.0 0.0
					{	vector<string>v = splitString(line, ' ');
						barycentricCoords.push_back(vec4(stof(v[0]), stof(v[1]), stof(v[2]), stof(v[3])));	}
					break;
				case BC_TETID:		//line: 0
					barycentricTetrahedronIds.push_back(stoi(line));
					break;
				default: 
					break;
				}
			}
		}
		logger::log("\t -file parsed .. s2t-map count: " + to_string(surfaceToTetMeshVertexMap.size()) + ", BC vert count: " + to_string(barycentricCoords.size()) + ", BC tet count: " + to_string(barycentricTetrahedronIds.size()));
		//logger::log("\t -file parsed .. s2t-map count: " + to_string(surfaceToTetMeshVertexMap.size()));
	}

	void parseFile_obj_mesh__sf_mesh(const string& fileName, vector<vec3>& tetMeshSurfVerts, vector<ivec3>& tetMeshSurfTris) {
		logger::log("-- parseFile_obj_mesh__sf_mesh");
		
		tetMeshSurfVerts.resize(0);
		tetMeshSurfTris.resize(0);
		
		FileReadMode frm = NONE;
		string line;
		ifstream inputFileStream(fileName);
		while (getline(inputFileStream, line)) {
			// deterine read mode (vertex or face)
			if ((frm == NONE || frm == TET_MESH_SURF_TRI) && startsWith(line, "v"))
				frm = TET_MESH_SURF_VERT;
			else if ((frm == NONE || frm == TET_MESH_SURF_VERT) && startsWith(line, "f"))
				frm = TET_MESH_SURF_TRI;
			// get info from line
			vector<string> v{ splitString(line, ' ') };
			if (frm == TET_MESH_SURF_VERT) {
				tetMeshSurfVerts.push_back(vec3(stof(v[1]), stof(v[2]), stof(v[3])));
				//logger::log("\t\t vert: "+to_string(tetMeshSurfVerts[tetMeshSurfVerts.size()-1]));
			}
			else if (frm == TET_MESH_SURF_TRI) {
				tetMeshSurfTris.push_back(ivec3(stoi(v[1]) - 1, stoi(v[2]) - 1, stoi(v[3]) - 1));
				//logger::log("\t\t tri: " + to_string(tetMeshSurfTris[tetMeshSurfTris.size() - 1]));
			}
		}		
		logger::log("\t\t-parsing done. "+to_string(tetMeshSurfVerts.size())+" verts, "+to_string(tetMeshSurfTris.size())+" tris");
	}
}