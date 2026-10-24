#pragma once

#include "StepArgument.h"
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace CADacomb::CAD
{
    class StepDocument
    {
      public:
        StepDocument();
        virtual ~StepDocument();

        void addSimpleEntity(int entityId, const std::string& typeName, const std::vector<StepArgument>& arguments);
        void addComplexEntity(int entityId, const std::string& summaryType, const std::map<std::string, std::vector<StepArgument>>& parts);

        const std::string&                                      getEntityType(int entityId) const;
        const std::vector<StepArgument>&                        getEntityArguments(int entityId) const;
        bool                                                    hasComplexEntity(int entityId) const;
        const std::map<std::string, std::vector<StepArgument>>& getComplexEntity(int entityId) const;

        const std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>>& getEntities() const;
        const std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>&  getComplexEntities() const;

      private:
        std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>> entities;
        std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>  complexEntities;
    };
}
