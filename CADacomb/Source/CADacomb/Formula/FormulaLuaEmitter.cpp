#include "CADacomb/Formula/FormulaLuaEmitter.h"

#include "CADacomb/Formula/SlangToLuaTranspiler.h"

#include <sstream>

namespace CADacomb
{
    FormulaLuaEmitter::FormulaLuaEmitter() {}

    FormulaLuaEmitter::~FormulaLuaEmitter() {}

    std::string FormulaLuaEmitter::getMinimalPrelude()
    {
        return R"LUA(local _vecMT = {}
local function _isVec(v) return type(v) == "table" and getmetatable(v) == _vecMT end
local function _makeVec(x, y, z, w, dim) return setmetatable({ x = x or 0.0, y = y or 0.0, z = z or 0.0, w = w or 0.0, _dim = dim }, _vecMT) end
local function float2(x, y) if _isVec(x) then return _makeVec(x.x, x.y, 0.0, 0.0, 2) end return _makeVec(x, y == nil and x or y, 0.0, 0.0, 2) end
local function float3(x, y, z) if _isVec(x) then return x._dim == 2 and _makeVec(x.x, x.y, y or 0.0, 0.0, 3) or _makeVec(x.x, x.y, x.z, 0.0, 3) end if y == nil then return _makeVec(x, x, x, 0.0, 3) end return _makeVec(x, y, z, 0.0, 3) end
local function float4(x, y, z, w) if _isVec(x) then return _makeVec(x.x, x.y, x.z, x._dim == 3 and (y or 0.0) or x.w, 4) end if y == nil then return _makeVec(x, x, x, x, 4) end return _makeVec(x, y, z, w, 4) end
local function _binOp(a, b, fn)
    if _isVec(a) and _isVec(b) then local d = a._dim return _makeVec(fn(a.x, b.x), fn(a.y, b.y), d >= 3 and fn(a.z, b.z) or 0.0, d >= 4 and fn(a.w, b.w) or 0.0, d)
    elseif _isVec(a) then local d = a._dim return _makeVec(fn(a.x, b), fn(a.y, b), d >= 3 and fn(a.z, b) or 0.0, d >= 4 and fn(a.w, b) or 0.0, d)
    else local d = b._dim return _makeVec(fn(a, b.x), fn(a, b.y), d >= 3 and fn(a, b.z) or 0.0, d >= 4 and fn(a, b.w) or 0.0, d) end
end
local function _unOp(a, fn) if _isVec(a) then local d = a._dim return _makeVec(fn(a.x), fn(a.y), d >= 3 and fn(a.z) or 0.0, d >= 4 and fn(a.w) or 0.0, d) end return fn(a) end
_vecMT.__add = function(a, b) return _binOp(a, b, function(x, y) return x + y end) end
_vecMT.__sub = function(a, b) return _binOp(a, b, function(x, y) return x - y end) end
_vecMT.__mul = function(a, b) return _binOp(a, b, function(x, y) return x * y end) end
_vecMT.__div = function(a, b) return _binOp(a, b, function(x, y) return x / y end) end
_vecMT.__unm = function(a) return _makeVec(-a.x, -a.y, -a.z, -a.w, a._dim) end
local function abs(a) return _unOp(a, math.abs) end
local function sqrt(a) return _unOp(a, math.sqrt) end
local function sin(a) return _unOp(a, math.sin) end
local function cos(a) return _unOp(a, math.cos) end
local function tan(a) return _unOp(a, math.tan) end
local function asin(a) return _unOp(a, math.asin) end
local function acos(a) return _unOp(a, math.acos) end
local function atan(a) return _unOp(a, math.atan) end
local function atan2(y, x) return math.atan(y, x) end
local function floor(a) return _unOp(a, math.floor) end
local function ceil(a) return _unOp(a, math.ceil) end
local function fract(a) return _unOp(a, function(x) return x - math.floor(x) end) end
local function frac(a) return fract(a) end
local function exp(a) return _unOp(a, math.exp) end
local function log(a) return _unOp(a, math.log) end
local function pow(a, b) return (_isVec(a) or _isVec(b)) and _binOp(a, b, function(x, y) return x ^ y end) or (a ^ b) end
local function min(a, b) return (_isVec(a) or _isVec(b)) and _binOp(a, b, math.min) or math.min(a, b) end
local function max(a, b) return (_isVec(a) or _isVec(b)) and _binOp(a, b, math.max) or math.max(a, b) end
local function clamp(x, lo, hi) return min(max(x, lo), hi) end
local function lerp(a, b, t) return a + (b - a) * t end
local function dot(a, b) if _isVec(a) and _isVec(b) then local d = a.x * b.x + a.y * b.y if a._dim >= 3 then d = d + a.z * b.z end if a._dim >= 4 then d = d + a.w * b.w end return d end return a * b end
local function length(v) return _isVec(v) and math.sqrt(dot(v, v)) or math.abs(v) end
local function normalize(v) local l = length(v) return l == 0.0 and v or (v / l) end
local function cross(a, b) return float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x) end)LUA";
    }

    std::string FormulaLuaEmitter::emit(const Formula& formula)
    {
        std::ostringstream out;
        out << "do\n";

        std::istringstream preludeStream(getMinimalPrelude());
        std::string        line;
        while (std::getline(preludeStream, line))
        {
            out << "    " << line << "\n";
        }

        std::string helperCode = trim(formula.getHelper());
        if (!helperCode.empty())
        {
            out << "\n";
            std::string        transpiledHelper = SlangToLuaTranspiler::transpile(helperCode);
            std::istringstream helperStream(transpiledHelper);
            while (std::getline(helperStream, line))
            {
                out << "    " << line << "\n";
            }
        }

        out << "\n";
        out << "    function " << formula.getName() << "(_input)\n";
        out << "        local input = {}\n";
        out << "        for _i = 1, " << formula.getInputCount() << " do input[_i - 1] = _input[_i] or 0.0 end\n";
        out << "        local output = {}\n";
        out << "        for _i = 0, " << formula.getOutputCount() << " - 1 do output[_i] = 0.0 end\n";

        std::string exprCode = trim(formula.getExpression());
        if (!exprCode.empty())
        {
            std::string        transpiledExpr = SlangToLuaTranspiler::transpile(exprCode);
            std::istringstream exprStream(transpiledExpr);
            while (std::getline(exprStream, line))
            {
                out << "        " << line << "\n";
            }
        }

        out << "        local _result = {}\n";
        out << "        for _i = 0, " << formula.getOutputCount() << " - 1 do _result[_i + 1] = output[_i] end\n";
        out << "        return _result\n";
        out << "    end\n";
        out << "end\n";

        return out.str();
    }

    std::string FormulaLuaEmitter::trim(const std::string& text)
    {
        size_t start = 0;
        while (start < text.size() && (text[start] == ' ' || text[start] == '\t' || text[start] == '\r' || text[start] == '\n'))
        {
            start++;
        }
        size_t end = text.size();
        while (end > start && (text[end - 1] == ' ' || text[end - 1] == '\t' || text[end - 1] == '\r' || text[end - 1] == '\n'))
        {
            end--;
        }
        return text.substr(start, end - start);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("FormulaLuaEmitter/EmitsCallableFunctionWithTranspiledHelper")
{
    CADacomb::Formula formula;
    formula.setName("sphereSdf");
    formula.setInputCount(4);
    formula.setOutputCount(1);
    formula.setHelper("float sdSphere(float3 p, float r)\n{\n    return length(p) - r;\n}");
    formula.setExpression("float3 p = float3(input[0], input[1], input[2]);\noutput[0] = sdSphere(p, input[3]);");

    std::string lua = CADacomb::FormulaLuaEmitter::emit(formula);

    CHECK(lua.find("local function sdSphere(p, r)") != std::string::npos);
    CHECK(lua.find("function sphereSdf(_input)") != std::string::npos);
    CHECK(lua.find("local p = float3(input[0], input[1], input[2])") != std::string::npos);
    CHECK(lua.find("output[0] = sdSphere(p, input[3])") != std::string::npos);
}
#endif
