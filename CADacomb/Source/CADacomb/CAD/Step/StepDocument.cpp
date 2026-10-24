#include "StepDocument.h"

#include <stdexcept>

namespace CADacomb::CAD
{
    StepDocument::StepDocument() {}

    StepDocument::~StepDocument() {}

    void StepDocument::addSimpleEntity(int entityId, const std::string& typeName, const std::vector<StepArgument>& arguments)
    {
        entities[entityId] = std::make_pair(typeName, arguments);
    }

    void StepDocument::addComplexEntity(int entityId, const std::string& summaryType, const std::map<std::string, std::vector<StepArgument>>& parts)
    {
        complexEntities[entityId] = parts;
        entities[entityId]        = std::make_pair(summaryType, std::vector<StepArgument>());
    }

    const std::string& StepDocument::getEntityType(int entityId) const
    {
        std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>>::const_iterator iterator = entities.find(entityId);
        if (iterator == entities.end())
        {
            throw std::runtime_error("STEP entity #" + std::to_string(entityId) + " not found");
        }
        return iterator->second.first;
    }

    const std::vector<StepArgument>& StepDocument::getEntityArguments(int entityId) const
    {
        std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>>::const_iterator iterator = entities.find(entityId);
        if (iterator == entities.end())
        {
            throw std::runtime_error("STEP entity #" + std::to_string(entityId) + " not found");
        }
        return iterator->second.second;
    }

    bool StepDocument::hasComplexEntity(int entityId) const
    {
        return complexEntities.count(entityId) > 0;
    }

    const std::map<std::string, std::vector<StepArgument>>& StepDocument::getComplexEntity(int entityId) const
    {
        std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>::const_iterator iterator = complexEntities.find(entityId);
        if (iterator == complexEntities.end())
        {
            throw std::runtime_error("STEP complex entity #" + std::to_string(entityId) + " not found");
        }
        return iterator->second;
    }

    const std::unordered_map<int, std::pair<std::string, std::vector<StepArgument>>>& StepDocument::getEntities() const
    {
        return entities;
    }

    const std::unordered_map<int, std::map<std::string, std::vector<StepArgument>>>& StepDocument::getComplexEntities() const
    {
        return complexEntities;
    }
}
