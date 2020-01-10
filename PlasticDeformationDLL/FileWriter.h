#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>

#include <glm/gtx/transform.hpp>

namespace fileWriter {
	void writeToFile(const string &filePath, const string &text, bool doOverwrite) {
		std::ofstream outputFile;
		if(doOverwrite)
			outputFile.open(filePath, std::ofstream::app);
		else
			outputFile.open(filePath, std::ofstream::trunc);
		outputFile << text;
		outputFile.close();
	}

	void writeTetMeshDataToFile(
		const string &filePath,
		//	const std::vector<glm::vec3> &surfVerts,
		//	const std::vector<glm::vec3> &tetMeshVerts,
		//	const std::vector<glm::ivec4> &tetMeshTets,
		const std::vector<int> &surfVertToTetVertMap,
		const std::vector<glm::vec4> &barycentricCoords,
		const std::vector<int> &barycentricTetIds) {

		std::ofstream outputFile;
		//std::string filePath = "F:\\Eigene Dateien\\Studium\\Master Schweden\\9 Master Thesis\\Practical\\MasterThesisPrototype\\Assets\\output.tetmesh";
		outputFile.open(filePath);
		//write sur verts
		/*outputFile << "suv " << surfVerts.size() << "\n";
		for (int i = 0; i < surfVerts.size(); i++) {
			outputFile << surfVerts[i].x << " " << surfVerts[i].y << " " << surfVerts[i].z << "\n";
		}//write sur verts
		outputFile << "tmv " << tetMeshVerts.size() << "\n";
		for (int i = 0; i < tetMeshVerts.size(); i++) {
			outputFile << tetMeshVerts[i].x << " " << tetMeshVerts[i].y << " " << tetMeshVerts[i].z << "\n";
		}*/
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