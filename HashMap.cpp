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

decltype(auto) HashMap::find(Hash::HashKey &key) { return this->map.find(key); }

bool HashMap::empty() { return this->map.empty(); }

decltype(auto) HashMap::begin() { return this->map.begin(); }
}  // namespace Hash