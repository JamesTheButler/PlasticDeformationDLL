#pragma once

#include <string>
#include <vector>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <fstream>

#include "Constraints.h"

using namespace std;
using namespace glm;

namespace Serialization {
	namespace detail {
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
	}

	void save(
		const string &filePath,
		const vector<vec3> &tetMeshVerts,
		const vector<ivec4> &tetMeshTets,
		const vector<vec3> &surfVerts,
		const vector<int> &surfVertToTetVertMap,
		const vector<glm::vec4> &barycentricCoords,
		const vector<int> &barycentricTetIds,
		const DistanceConstraintData distanceConstraints,
		const VolumeConstraintData volumeConstraints) {

		ofstream outputFile;
		outputFile.open(filePath);
		//write tet mesh verts
		outputFile << "tmv " << tetMeshVerts.size() << "\n";
		for (int i = 0; i < tetMeshVerts.size(); i++) {
			outputFile << tetMeshVerts[i].x << " " << tetMeshVerts[i].y << " " << tetMeshVerts[i].z << "\n";
		}
		//write tet mesh tets
		outputFile << "tmt " << tetMeshTets.size() << "\n";
		for (int i = 0; i < tetMeshTets.size(); i++) {
			outputFile << tetMeshTets[i].x << " " << tetMeshTets[i].y << " " << tetMeshTets[i].z << " " << tetMeshTets[i].w << "\n";
		}
		//write sur verts
		outputFile << "smv " << surfVerts.size() << "\n";
		for (int i = 0; i < surfVerts.size(); i++) {
			outputFile << surfVerts[i].x << " " << surfVerts[i].y << " " << surfVerts[i].z << "\n";
		}
		//write tet mesh tets
		outputFile << "s2t " << surfVertToTetVertMap.size() << "\n";
		for (int i = 0; i < surfVertToTetVertMap.size(); i++) {
			outputFile << surfVertToTetVertMap[i] << "\n";
		}
		//write BC coords
		outputFile << "bcc " << barycentricCoords.size() << "\n";
		for (int i = 0; i < barycentricCoords.size(); i++) {
			outputFile << barycentricCoords[i].x << ' ' << barycentricCoords[i].y << ' ' << barycentricCoords[i].z << ' ' << barycentricCoords[i].w << "\n";
		}
		//write BC tet ids
		outputFile << "bct " << barycentricTetIds.size() << "\n";
		for (int i = 0; i < barycentricTetIds.size(); i++) {
			outputFile << barycentricTetIds[i] << "\n";
		}
		//write distance constraints
		outputFile << "dcs " << distanceConstraints.constraintCount << "\n";
		for (int i = 0; i < distanceConstraints.constraintCount; i++) {
			outputFile
				<< distanceConstraints.vertexIds[i].x << ' '
				<< distanceConstraints.vertexIds[i].y << ' '
				<< distanceConstraints.restValues[i] << "\n";
		}
		//write volume constraints
		outputFile << "vcs " << volumeConstraints.constraintCount << "\n";
		for (int i = 0; i < volumeConstraints.constraintCount; i++) {
			outputFile
				<< volumeConstraints.vertexIds[i].x << ' '
				<< volumeConstraints.vertexIds[i].y << ' '
				<< volumeConstraints.vertexIds[i].z << ' '
				<< volumeConstraints.vertexIds[i].w << ' '
				<< volumeConstraints.restValues[i] << "\n";
		}
		outputFile << "cpv " << surfVerts.size() << "\n";
		for (int i = 0; i < surfVerts.size(); i++) {
			outputFile
				<< distanceConstraints.constraintCountPerVertex[i] << ' '
				<< volumeConstraints.constraintCountPerVertex[i] << "\n";
		}
		outputFile.close();
	}

	void load(
		const string &filePath,
		vector<vec3> &tetMeshVerts,
		vector<ivec4> &tetMeshTets,
		vector<vec3> &surfVerts,
		vector<int> &surfMeshToTetMeshMap,
		vector<vec4> &barycentricCoords,
		vector<int> &barycentricTetrahedra,
		DistanceConstraintData distanceConstraints,
		VolumeConstraintData volumeConstraints) {

		enum FileReadMode {
			NONE,
			SURFMESH_VERT,
			TETMESH_VERT,
			TETMESH_TET,
			SURFMESH_TO_TETMESH_MAP,
			BARYCENTRIC_COORD,
			BARYCENTRIC_TET,
			DISTANCE_CONSTRAINT,
			VOLUME_CONSTRAINT,
			CONSTRAINTS_PER_VERTEX,
		};

		FileReadMode frm = NONE;
		string line;
		//int lineCounter = 0;
		int identifierLine = 0;
		ifstream inputFileStream(filePath);
		while (getline(inputFileStream, line)) {
			vector<string> splitLine = detail::splitString(line, ' ');
			// check for new line read mode
			if (identifierLine == 0) {
				string readModeIdentifier = splitLine[0];
				identifierLine = stoi(splitLine[1]);
				// determine line identifier
				if (readModeIdentifier == "tmv"){
					frm = TETMESH_VERT;
					tetMeshVerts.reserve(identifierLine);
				} else if (readModeIdentifier == "tmt") {
					frm = TETMESH_TET;
					tetMeshTets.reserve(identifierLine);
				} else if (readModeIdentifier == "smv") {
					frm = SURFMESH_VERT;
					surfVerts.reserve(identifierLine);
				} else if (readModeIdentifier == "s2t") {
					frm = SURFMESH_TO_TETMESH_MAP;
					surfMeshToTetMeshMap.reserve(identifierLine);
				} else if (readModeIdentifier == "bcc") {
					frm = BARYCENTRIC_COORD;
					barycentricCoords.reserve(identifierLine);
				} else if (readModeIdentifier == "bct") {
					frm = BARYCENTRIC_TET;
					barycentricTetrahedra.reserve(identifierLine);
				} else if (readModeIdentifier == "dcs") {
					frm = DISTANCE_CONSTRAINT;
					distanceConstraints.vertexIds.reserve(identifierLine);
					distanceConstraints.restValues.reserve(identifierLine);
				} else if (readModeIdentifier == "vcs") {
					frm = VOLUME_CONSTRAINT;
					volumeConstraints.vertexIds.reserve(identifierLine);
					volumeConstraints.restValues.reserve(identifierLine);
				} else if (readModeIdentifier == "cpv") {
					frm = CONSTRAINTS_PER_VERTEX;
					distanceConstraints.constraintCountPerVertex.reserve(identifierLine);
					volumeConstraints.constraintCountPerVertex.reserve(identifierLine);
				}
			} else {
				// interpret line
				identifierLine--;
				switch (frm) {
				case SURFMESH_VERT:
					surfVerts.push_back(vec3(stof(splitLine[0]), stof(splitLine[1]), stof(splitLine[2])));
					break;
				case TETMESH_VERT:
					tetMeshVerts.push_back(vec3(stof(splitLine[0]), stof(splitLine[1]), stof(splitLine[2])));
					break;
				case TETMESH_TET:
					tetMeshTets.push_back(ivec4(stoi(splitLine[0]), stoi(splitLine[1]), stoi(splitLine[2]), stoi(splitLine[3])));
					break;
				case SURFMESH_TO_TETMESH_MAP:
					surfMeshToTetMeshMap.push_back(stoi(splitLine[0]));
					break;
				case BARYCENTRIC_COORD:
					barycentricCoords.push_back(vec4(stof(splitLine[0]), stof(splitLine[1]), stof(splitLine[2]), stof(splitLine[3])));
				break;
				case BARYCENTRIC_TET:
					barycentricTetrahedra.push_back(stoi(splitLine[0]));
					break;
				case DISTANCE_CONSTRAINT:
					distanceConstraints.vertexIds.push_back(ivec2(stoi(splitLine[0]), stoi(splitLine[1])));
					distanceConstraints.restValues.push_back(stof(splitLine[2]));
					break;
				case VOLUME_CONSTRAINT:
					volumeConstraints.vertexIds.push_back(ivec4(stoi(splitLine[0]), stoi(splitLine[1]), stoi(splitLine[2]), stoi(splitLine[3])));
					volumeConstraints.restValues.push_back(stof(splitLine[4]));
					break;
				case CONSTRAINTS_PER_VERTEX:
					distanceConstraints.constraintCountPerVertex.push_back(stoi(splitLine[0]));
					volumeConstraints.constraintCountPerVertex.push_back(stoi(splitLine[1]));
					break;
				default:
					break;
				}
			}
		}
	}

}