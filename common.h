//
// Created by yyh on 22-10-21.
//

#ifndef CENTRAL_VOTING_COMMON_H
#define CENTRAL_VOTING_COMMON_H
#include <vector>
#include "HashMap.h"
namespace PPF{
typedef std::vector<std::vector<std::vector<std::vector<std::vector<Hash::HashData>>>>> searchMapType;
typedef boost::shared_ptr<std::vector<
    std::vector<std::vector<std::vector<std::vector<Hash::HashData>>>>>> searchMapTypePtr;
}

#endif  // CENTRAL_VOTING_COMMON_H
