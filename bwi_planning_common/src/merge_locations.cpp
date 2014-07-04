/**
 *
 * \file merge_locations.cpp
 * \brief  Tool to merge location files into one.
 * 
 * execute merge_locations location1.yaml location2.yam ... locationN.yaml final.yaml
 * 
 * to merge the n files into final.yaml. Following files overwrite the preceeding ones in case of 
 * overlapping locations, so be careful because the order matters.
 *
 * \author  Matteo Leonetti <matteo@cs.utexas.edu>
 * 
 * Copyright (c) 2013, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 03/23/2013 09:07:41 PM piyushk $
 *
 **/

#include <bwi_planning_common/structures.h>

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <fstream>

using namespace std;

static void merge(	std::vector<std::string>& firstLoc,
			std::vector<int32_t>& firstMap,
			const std::vector<std::string>& secondLoc,
			const std::vector<int32_t>& secondMap) throw (logic_error);
			
static void clean (std::vector<std::string>& symbols, std::vector<int32_t>& pixels);

static void writeToFile(std::vector<std::string>& symbols, std::vector<int32_t>& pixels, const std::string& fileName);

int main(int argc, char **argv) {
	
	if(argc < 4) {
		cout << "usage: merge_locations location1.yaml [locationN.yaml]+ finalLocation.yaml" << endl;
		return -1;
	}
	
	std::vector<int32_t> location_map;
	std::vector<std::string> locations;
	bwi_planning_common::readLocationFile(argv[1], locations, location_map);
	
	//first string is the name of the executable
	//second string is the first source file
	//last string is the destination file
	for(int i=2; i<argc - 1; ++i) {
		std::vector<int32_t> location_map2;
		std::vector<std::string> locations2;
	
		bwi_planning_common::readLocationFile(argv[i], locations2, location_map2);
	
		try {
			merge(locations,location_map,locations2,location_map2);
		}catch (const logic_error& err) {
			cerr << "Error on " << argv[i] << ":" << err.what() << endl;
		}
	}
	
	//there may be some symbols that have been overwritten and are not in use any more
	clean(locations, location_map);
	
	writeToFile(locations,location_map,argv[argc-1]);
	
	return 0;
}
	
void merge(	std::vector<std::string>& firstLoc,
			std::vector<int32_t>& firstMap,
			const std::vector<std::string>& secondLoc,
			const std::vector<int32_t>& secondMap) throw (logic_error){
	
	if(firstMap.size() != secondMap.size())
		throw logic_error("the file seems to contain a different map. Skipping.");
	
	vector<int32_t> remapping(secondLoc.size());
	std::vector<std::string>::const_iterator secondLIt = secondLoc.begin();
	
	for(int pos=0; secondLIt != secondLoc.end(); ++secondLIt, ++pos) {
		
		//see if a symbol with the same name already exists in the first map
		const std::vector<std::string>::const_iterator symbol = find(firstLoc.begin(),firstLoc.end(),*secondLIt);

		std::vector<std::string>::const_iterator constBegin = firstLoc.begin();
		remapping[pos] = distance(constBegin,symbol); //if not found this equals size (the position of end())
		
		if(symbol == firstLoc.end())
			firstLoc.push_back(*secondLIt);
	}
	
	//overwrite the first map with the second
	
	vector<int32_t>::iterator firstMIt = firstMap.begin();
	vector<int32_t>::const_iterator secondMIt = secondMap.begin();
	
	for(; firstMIt != firstMap.end(); ++firstMIt, ++secondMIt) {
		if(*secondMIt >= 0)
			*firstMIt = remapping.at(*secondMIt); //checks bounds
	}
	
	return;
}

void clean (std::vector<std::string>& symbols, std::vector<int32_t>& pixels) {
	
	vector<bool> found(symbols.size());
	
	vector<int32_t>::iterator pixelIt = pixels.begin();
	
	for(; pixelIt != pixels.end(); ++pixelIt)
		if(*pixelIt >=0)
			found.at(*pixelIt) = true;
		
	vector<int32_t> remapping(found.size());
	int toAssign = 0;
	for(int i=0, size = remapping.size(); i < size; ++i) {
		if(found[i])
			remapping[i] = toAssign++;
		else {
			vector<string>::iterator toErase = symbols.begin();
			advance(toErase,toAssign);
			symbols.erase(toErase);
		}
	}
	
	//now remap the pixel map
	pixelIt = pixels.begin();
	for(; pixelIt != pixels.end(); ++pixelIt)
		if(*pixelIt >=0)
			*pixelIt = remapping.at(*pixelIt);

}

static void writeToFile(std::vector<std::string>& locations, std::vector<int32_t>& component_map, const std::string& location_file) {
	//code copied from mark_locations
	
	std::stringstream location_ss;
      location_ss << "locations:" << std::endl;
      for (size_t i = 0; i < locations.size(); ++i) {
        std::string &loc = locations[i];
        location_ss << "  - " << loc << std::endl;
      }
      location_ss << "data: [" << std::endl;
      for (size_t x = 0; x < component_map.size(); ++x) {
        location_ss << component_map[x];
        if (x != component_map.size() - 1)
          location_ss << ", ";
      }
      location_ss << "]" << std::endl;

      std::ofstream location_fout(location_file.c_str());
      location_fout << location_ss.str();
      location_fout.close();
      std::cout << "Wrote locations to file: " << location_file << std::endl;
	
}
	
	