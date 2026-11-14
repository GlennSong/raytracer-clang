#include "apps/citysim/control_policy_file.h"

#include "engine/script_assets.h"   // loadScriptCode: files under assets/scripts
#ifdef RT_ENABLE_SCRIPTING
#include "apps/driving/scripting/rule_set_lua.h"
#include "engine/scripting/lua_state.h"
#include "engine/scripting/script_vm.h"
#endif

namespace citysim {

bool loadControlPolicy(const std::string& ruleSetFile, engine::roads::lanes::ControlPolicy& out, std::string* err) {
#ifdef RT_ENABLE_SCRIPTING
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return false;
    };
    const std::string code = engine::loadScriptCode(ruleSetFile, "");
    if (code.empty()) return fail(ruleSetFile + ": not found under assets/scripts (or it is empty)");
    engine::ScriptVM vm;
    driving::RuleSet rules;
    if (!vm.doString(code, err) || !driving::loadRuleSet(vm, ruleSetFile, rules, err)) return false;
    // the shared policy's file, when the rule set names one (rule_set.placement.policy)
    std::string policyFile;
    {
        lua_State* L = engine::luaState(vm);
        const int top = lua_gettop(L);
        lua_getglobal(L, "rule_set");
        if (lua_istable(L, -1)) {
            lua_getfield(L, -1, "placement");
            if (lua_istable(L, -1)) {
                lua_getfield(L, -1, "policy");
                if (lua_type(L, -1) == LUA_TSTRING) policyFile = lua_tostring(L, -1);
            }
        }
        lua_settop(L, top);
    }
    if (!policyFile.empty()) {
        const std::string policyCode = engine::loadScriptCode(policyFile, "");
        if (policyCode.empty()) return fail(ruleSetFile + ": its placement policy " + policyFile + " is not found under assets/scripts");
        if (!vm.doString(policyCode, err)) return false;
    }
    return driving::loadPlacementPolicy(vm, ruleSetFile, rules, out, err);
#else
    (void)ruleSetFile;
    (void)out;
    if (err) *err = "this build has no scripting, so no rule set can be read";
    return false;
#endif
}

}  // namespace citysim
