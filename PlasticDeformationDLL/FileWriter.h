#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>

#include <glm/gtx/transform.hpp>

#include "Logger.h"

using namespace std;
using namespace glm;

namespace fileWriter {
	void writeTetMeshDataToFile(
		const string &filePath,
		const vector<vec3> &surfVerts,
		const vector<vec3> &tetMeshVerts,
		const vector<ivec4> &tetMeshTets,
		const vector<int> &surfVertToTetVertMap,
		const vector<glm::vec4> &barycentricCoords,
		const vector<int> &barycentricTetIds) {

		logger::log("--writeTetMeshDataToFile");
		ofstream outputFile;
		outputFile.open(filePath);
		//write sur verts
		outputFile << "suv " << surfVerts.size() << "\n";
		for (int i = 0; i < surfVerts.size(); i++) {
			outputFile << surfVerts[i].x << " " << surfVerts[i].y << " " << surfVerts[i].z << "\n";
		}
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
		outputFile.close();
	}

	void writeTetMeshDataToFile(
		const string &filePath,
		const vector<int> &surfVertToTetVertMap,
		const vector<glm::vec4> &barycentricCoords,
		const vector<int> &barycentricTetIds) {

		logger::log("--writeTetMeshDataToFile");
		ofstream outputFile;
		outputFile.open(filePath);
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
		outputFile.close();
	}
}