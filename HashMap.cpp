//
// Created by yyh on 22-7-17.
//
#include "HashMap.h"

namespace Hash {

bool HashMap::addInfo(Hash::HashKey &key, Hash::HashData &data) {
  try {
    this->map.emplace(key, data);
    return true;
  } catch (std::bad_alloc) {
    PCL_INFO("bad alloc !! ");
    return false;
  }
}
bool HashMap::addInfo(std::pair<Hash::HashKey, Hash::HashData> &data) {
  try {
    this->map.insert(data);
    return true;
  } catch (std::bad_alloc) {
    PCL_INFO("bad alloc !! ");
    return false;
  }
}

HashData HashMap::getData(Hash::HashKey &key) {
  return (*this->map.find(key)).second;
}

bool HashMap::find(Hash::HashKey &key) {
  if (map.find(key) != map.end()) {
    return true;
  } else {
    return false;
  }
}

bool HashMap::empty() { return this->map.empty(); }

decltype(auto) HashMap::begin() { return this->map.begin(); }
}  // namespace Hash