#include "StepParser.h"

#include <cctype>
#include <map>
#include <stdexcept>

namespace CADacomb::CAD
{
    StepParser::StepParser() {}

    StepParser::~StepParser() {}

    StepDocument StepParser::parse(const std::string& rawText) const
    {
        std::string cleanedText = stripComments(rawText);
        size_t      dataStart   = cleanedText.find("DATA;");
        if (dataStart == std::string::npos)
        {
            throw std::runtime_error("Not a valid STEP file (missing DATA; section)");
        }

        StepDocument document;
        size_t       position = dataStart + 5;
        size_t       length   = cleanedText.size();

        while (position < length)
        {
            size_t hashIndex = cleanedText.find('#', position);
            if (hashIndex == std::string::npos)
            {
                break;
            }
            size_t index = hashIndex + 1;
            while (index < length && std::isdigit(static_cast<unsigned char>(cleanedText[index])))
            {
                index++;
            }
            if (index == hashIndex + 1)
            {
                position = index;
                continue;
            }
            int entityId = std::stoi(cleanedText.substr(hashIndex + 1, index - (hashIndex + 1)));
            skipWhitespace(cleanedText, index);
            if (index >= length || cleanedText[index] != '=')
            {
                position = index;
                continue;
            }
            index++;
            skipWhitespace(cleanedText, index);
            if (index >= length)
            {
                break;
            }

            if (cleanedText[index] == '(')
            {
                index++;
                std::map<std::string, std::vector<StepArgument>> parts;
                while (index < length)
                {
                    skipWhitespace(cleanedText, index);
                    if (index >= length || cleanedText[index] == ')')
                    {
                        index++;
                        break;
                    }
                    size_t keywordStart = index;
                    while (index < length && (std::isalnum(static_cast<unsigned char>(cleanedText[index])) || cleanedText[index] == '_'))
                    {
                        index++;
                    }
                    std::string partName = toUpperAscii(cleanedText.substr(keywordStart, index - keywordStart));
                    skipWhitespace(cleanedText, index);
                    std::pair<std::vector<StepArgument>, size_t> parsed = parseArguments(cleanedText, index);
                    index                                               = parsed.second;
                    parts[partName]                                     = parsed.first;
                }

                std::string              summaryKind = "COMPLEX:";
                std::vector<std::string> candidateNames;
                candidateNames.push_back("B_SPLINE_SURFACE_WITH_KNOTS");
                candidateNames.push_back("B_SPLINE_CURVE_WITH_KNOTS");
                candidateNames.push_back("GEOMETRIC_REPRESENTATION_CONTEXT");
                candidateNames.push_back("LENGTH_UNIT");
                candidateNames.push_back("PLANE_ANGLE_UNIT");

                for (size_t candidateIndex = 0; candidateIndex < candidateNames.size(); candidateIndex++)
                {
                    if (parts.count(candidateNames[candidateIndex]) > 0)
                    {
                        summaryKind += candidateNames[candidateIndex];
                        break;
                    }
                }
                document.addComplexEntity(entityId, summaryKind, parts);
            }
            else
            {
                size_t keywordStart = index;
                while (index < length && (std::isalnum(static_cast<unsigned char>(cleanedText[index])) || cleanedText[index] == '_'))
                {
                    index++;
                }
                std::string typeName = toUpperAscii(cleanedText.substr(keywordStart, index - keywordStart));
                skipWhitespace(cleanedText, index);
                if (index < length && cleanedText[index] == '(')
                {
                    std::pair<std::vector<StepArgument>, size_t> parsed = parseArguments(cleanedText, index);
                    index                                               = parsed.second;
                    document.addSimpleEntity(entityId, typeName, parsed.first);
                }
            }

            size_t semicolonIndex = cleanedText.find(';', index);
            position              = (semicolonIndex != std::string::npos) ? semicolonIndex + 1 : length;
        }

        return document;
    }

    std::string StepParser::stripComments(const std::string& rawText) const
    {
        std::string cleanedText;
        cleanedText.reserve(rawText.size());
        size_t index = 0;
        while (index < rawText.size())
        {
            if (index + 1 < rawText.size() && rawText[index] == '/' && rawText[index + 1] == '*')
            {
                index += 2;
                while (index + 1 < rawText.size() && !(rawText[index] == '*' && rawText[index + 1] == '/'))
                {
                    index++;
                }
                index += 2;
            }
            else
            {
                cleanedText += rawText[index];
                index++;
            }
        }
        return cleanedText;
    }

    void StepParser::skipWhitespace(const std::string& text, size_t& index) const
    {
        while (index < text.size() && (text[index] == ' ' || text[index] == '\t' || text[index] == '\r' || text[index] == '\n'))
        {
            index++;
        }
    }

    std::string StepParser::toUpperAscii(const std::string& inputText) const
    {
        std::string result = inputText;
        for (size_t index = 0; index < result.size(); index++)
        {
            result[index] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[index])));
        }
        return result;
    }

    std::pair<std::vector<StepArgument>, size_t> StepParser::parseArguments(const std::string& text, size_t startPosition) const
    {
        std::vector<std::vector<StepArgument>> argumentStack(1);
        std::vector<std::string>               pendingKeywords(1, "");
        size_t                                 index  = startPosition + 1;
        size_t                                 length = text.size();

        while (index < length)
        {
            skipWhitespace(text, index);
            if (index >= length)
            {
                break;
            }
            char                       currentChar = text[index];
            std::vector<StepArgument>& currentList = argumentStack.back();

            if (currentChar == '#')
            {
                index++;
                size_t numberStart = index;
                while (index < length && std::isdigit(static_cast<unsigned char>(text[index])))
                {
                    index++;
                }
                int referenceId = std::stoi(text.substr(numberStart, index - numberStart));
                currentList.push_back(StepArgument::createReference(referenceId));
            }
            else if (currentChar == '\'')
            {
                index++;
                std::string parsedString;
                while (index < length)
                {
                    if (text[index] == '\'')
                    {
                        if (index + 1 < length && text[index + 1] == '\'')
                        {
                            parsedString += '\'';
                            index += 2;
                        }
                        else
                        {
                            index++;
                            break;
                        }
                    }
                    else
                    {
                        parsedString += text[index];
                        index++;
                    }
                }
                currentList.push_back(StepArgument::createString(parsedString));
            }
            else if (currentChar == '.' && index + 1 < length && std::isupper(static_cast<unsigned char>(text[index + 1])))
            {
                index++;
                size_t enumStart = index;
                while (index < length && text[index] != '.')
                {
                    index++;
                }
                std::string enumValue = text.substr(enumStart, index - enumStart);
                if (index < length)
                {
                    index++;
                }
                currentList.push_back(StepArgument::createEnumeration(enumValue));
            }
            else if (std::isdigit(static_cast<unsigned char>(currentChar)) || currentChar == '+' || currentChar == '-' || currentChar == '.')
            {
                size_t numberStart = index;
                if (text[index] == '+' || text[index] == '-')
                {
                    index++;
                }
                while (index < length && (std::isdigit(static_cast<unsigned char>(text[index])) || text[index] == '.'))
                {
                    index++;
                }
                if (index < length && (text[index] == 'e' || text[index] == 'E'))
                {
                    index++;
                    if (index < length && (text[index] == '+' || text[index] == '-'))
                    {
                        index++;
                    }
                    while (index < length && std::isdigit(static_cast<unsigned char>(text[index])))
                    {
                        index++;
                    }
                }
                double numericValue = std::stod(text.substr(numberStart, index - numberStart));
                currentList.push_back(StepArgument::createNumber(numericValue));
            }
            else if (std::isalpha(static_cast<unsigned char>(currentChar)) || currentChar == '_')
            {
                size_t keywordStart = index;
                while (index < length && (std::isalnum(static_cast<unsigned char>(text[index])) || text[index] == '_'))
                {
                    index++;
                }
                std::string keyword = text.substr(keywordStart, index - keywordStart);
                skipWhitespace(text, index);
                if (index < length && text[index] == '(')
                {
                    index++;
                }
                pendingKeywords.push_back(keyword);
                argumentStack.push_back(std::vector<StepArgument>());
            }
            else if (currentChar == '(')
            {
                index++;
                pendingKeywords.push_back("");
                argumentStack.push_back(std::vector<StepArgument>());
            }
            else if (currentChar == ')')
            {
                index++;
                std::vector<StepArgument> completedList = argumentStack.back();
                argumentStack.pop_back();
                std::string keyword = pendingKeywords.back();
                pendingKeywords.pop_back();
                if (argumentStack.empty())
                {
                    return std::make_pair(completedList, index);
                }
                if (!keyword.empty())
                {
                    argumentStack.back().push_back(StepArgument::createTyped(keyword, completedList));
                }
                else
                {
                    argumentStack.back().push_back(StepArgument::createList(completedList));
                }
            }
            else if (currentChar == '$')
            {
                index++;
                currentList.push_back(StepArgument::createNone());
            }
            else if (currentChar == '*')
            {
                index++;
                currentList.push_back(StepArgument::createStar());
            }
            else
            {
                index++;
            }
        }

        throw std::runtime_error("Unterminated STEP argument list");
    }
}
