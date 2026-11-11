#include "apps/driving/road_rules.h"

#include <cstddef>

namespace driving {

namespace {

// One need of one fact checked against what the rule set gives: `what` names the table ("number", "personal
// setting"), `unit` is the unit the rule set wrote it in, `found` whether it is there at all.
void checkNeed(const std::string& ruleSetName, const std::string& factName, const char* what, const FactNeed& need,
               bool found, const std::string& unit, std::vector<std::string>& problems) {
    if (!found) {
        problems.push_back("fact \"" + factName + "\" needs the " + what + " \"" + need.name + "\", which rule set \"" +
                           ruleSetName + "\" does not have");
        return;
    }
    UnitFamily family;
    if (!unitFamily(unit, family)) {
        problems.push_back("fact \"" + factName + "\" needs the " + what + " \"" + need.name + "\" as " +
                           unitFamilyWords(need.family) + ", but rule set \"" + ruleSetName + "\" writes it in \"" +
                           unit + "\", which is not a unit (units: " + unitWords() + ")");
        return;
    }
    if (family != need.family) {
        problems.push_back("fact \"" + factName + "\" needs the " + what + " \"" + need.name + "\" as " +
                           unitFamilyWords(need.family) + " (" + unitFamilyStoredUnit(need.family) +
                           "), but rule set \"" + ruleSetName + "\" gives it in \"" + unit + "\", which is " +
                           unitFamilyWords(family));
    }
}

}  // namespace

const Device* RoadRules::device(const RuleSet& rules, engine::NameIndex kind) const {
    if (kind >= deviceOfKind.size()) return nullptr;
    const uint32_t d = deviceOfKind[kind];
    if (d >= rules.devices.size()) return nullptr;
    return &rules.devices[d];
}

bool buildRoadRules(const engine::LaneGraph& roads, const RuleSet& rules, const FactRegistry& facts, RoadRules& out,
                    std::vector<std::string>* problems) {
    std::vector<std::string> found;
    out.deviceOfKind.assign(roads.regulationKindNames.size(), kNoDevice);

    for (std::size_t k = 0; k < roads.regulationKindNames.size(); ++k) {
        const std::string& kindName = roads.regulationKindNames[k];
        const Device* d = rules.findDevice(kindName);
        if (!d) {
            found.push_back("the lane graph has regulations of kind \"" + kindName + "\", but rule set \"" +
                            rules.name + "\" has no device \"" + kindName + "\"");
            continue;
        }
        out.deviceOfKind[k] = static_cast<uint32_t>(d - rules.devices.data());
    }

    for (const FactSpec& fact : facts.all()) {
        for (const FactNeed& need : fact.numbers) {
            const Number* n = rules.findNumber(need.name);
            checkNeed(rules.name, fact.name, "number", need, n != nullptr, n ? n->unit : std::string(), found);
        }
        for (const FactNeed& need : fact.settings) {
            const PersonalSetting* s = rules.findPersonal(need.name);
            checkNeed(rules.name, fact.name, "personal setting", need, s != nullptr, s ? s->unit : std::string(),
                      found);
        }
    }

    if (problems) problems->insert(problems->end(), found.begin(), found.end());
    return found.empty();
}

}  // namespace driving
