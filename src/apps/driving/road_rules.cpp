#include "apps/driving/road_rules.h"

#include <cstddef>

namespace driving {

namespace {

// One need of one fact (or other reader) checked against what the rule set gives: `who` names the one that needs it
// (`fact "corner_speed"`), `what` names the table ("number", "personal setting"), `unit` is the unit the rule set
// wrote it in, `found` whether it is there at all.
void checkNeed(const std::string& ruleSetName, const std::string& who, const char* what, const FactNeed& need,
               bool found, const std::string& unit, std::vector<std::string>& problems) {
    if (!found) {
        problems.push_back(who + " needs the " + what + " \"" + need.name + "\", which rule set \"" + ruleSetName +
                           "\" does not have");
        return;
    }
    UnitFamily family;
    if (!unitFamily(unit, family)) {
        problems.push_back(who + " needs the " + what + " \"" + need.name + "\" as " + unitFamilyWords(need.family) +
                           ", but rule set \"" + ruleSetName + "\" writes it in \"" + unit +
                           "\", which is not a unit (units: " + unitWords() + ")");
        return;
    }
    if (family != need.family) {
        problems.push_back(who + " needs the " + what + " \"" + need.name + "\" as " + unitFamilyWords(need.family) +
                           " (" + unitFamilyStoredUnit(need.family) + "), but rule set \"" + ruleSetName +
                           "\" gives it in \"" + unit + "\", which is " + unitFamilyWords(family));
    }
}

// Every number and setting one fact or reader declares, checked in the order it declares them.
void checkNeeds(const RuleSet& rules, const std::string& who, const std::vector<FactNeed>& numbers,
                const std::vector<FactNeed>& settings, std::vector<std::string>& problems) {
    for (const FactNeed& need : numbers) {
        const Number* n = rules.findNumber(need.name);
        checkNeed(rules.name, who, "number", need, n != nullptr, n ? n->unit : std::string(), problems);
    }
    for (const FactNeed& need : settings) {
        const PersonalSetting* s = rules.findPersonal(need.name);
        checkNeed(rules.name, who, "personal setting", need, s != nullptr, s ? s->unit : std::string(), problems);
    }
}

// The road class a path belongs to: a lane's section's; a connector's, that of the lane it joins. kNoName for none.
engine::NameIndex roadClassOf(const engine::LaneGraph& roads, engine::PathId path) {
    engine::PathId lane = path;
    const engine::Path& p = roads.paths[path.index];
    if (p.kind == engine::PathKind::Connector) {
        if (p.next.count == 0 || p.next.first >= roads.nextPaths.size()) return engine::kNoName;
        lane = roads.nextPaths[p.next.first];
    }
    if (!lane.valid() || lane.index >= roads.paths.size()) return engine::kNoName;
    const engine::Path& l = roads.paths[lane.index];
    if (l.kind != engine::PathKind::Lane || l.section.index >= roads.sections.size()) return engine::kNoName;
    return roads.sections[l.section.index].roadClass;
}

}  // namespace

const Device* RoadRules::device(const RuleSet& rules, engine::NameIndex kind) const {
    if (kind >= deviceOfKind.size()) return nullptr;
    const uint32_t d = deviceOfKind[kind];
    if (d >= rules.devices.size()) return nullptr;
    return &rules.devices[d];
}

Real RoadRules::baseLimit(const engine::LaneGraph& roads, engine::PathId path) const {
    if (!path.valid() || path.index >= roads.paths.size()) return 0;
    const Real posted = roads.paths[path.index].speedLimit;
    if (posted > 0) return posted;
    return path.index < unpostedLimit.size() ? unpostedLimit[path.index] : Real(0);
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

    // Paths with no posted limit take their road class's default, which must be a speed.
    out.unpostedLimit.assign(roads.paths.size(), 0);
    enum class Lack : uint8_t { Nothing, NoDefault, NotASpeed };   // what a road class with unposted paths lacks
    std::vector<Lack> lacking(roads.roadClassNames.size(), Lack::Nothing);
    for (std::size_t p = 0; p < roads.paths.size(); ++p) {
        if (roads.paths[p].speedLimit > 0) continue;
        const engine::NameIndex cls = roadClassOf(roads, engine::PathId(static_cast<uint32_t>(p)));
        if (cls >= roads.roadClassNames.size()) continue;
        const Number* limit = rules.findDefaultLimit(roads.roadClassNames[cls]);
        UnitFamily family = UnitFamily::Ratio;
        if (!limit) lacking[cls] = Lack::NoDefault;
        else if (!unitFamily(limit->unit, family) || family != UnitFamily::Speed) lacking[cls] = Lack::NotASpeed;
        else out.unpostedLimit[p] = static_cast<Real>(limit->value);
    }
    for (std::size_t c = 0; c < lacking.size(); ++c) {
        const std::string& cls = roads.roadClassNames[c];
        if (lacking[c] == Lack::NoDefault)
            found.push_back("the lane graph has paths of road class \"" + cls + "\" with no posted limit, but rule set \"" +
                            rules.name + "\" has no " + kKeyDefaultLimits + "." + cls);
        else if (lacking[c] == Lack::NotASpeed)
            found.push_back("rule set \"" + rules.name + "\" gives " + kKeyDefaultLimits + "." + cls + " in \"" +
                            rules.findDefaultLimit(cls)->unit + "\", which is not a speed");
    }

    // Every control a junction carries must be a row of the rule set's control table.
    out.controlOfName.assign(roads.controlNames.size(), kNoDevice);
    std::vector<uint32_t> junctionsOfControl(roads.controlNames.size(), 0);
    for (const engine::Junction& jn : roads.junctions)
        if (jn.control < junctionsOfControl.size()) ++junctionsOfControl[jn.control];
    out.darkOfName.assign(roads.controlNames.size(), 0);
    for (std::size_t c = 0; c < roads.controlNames.size(); ++c) {
        const ControlKind* k = rules.findControl(roads.controlNames[c]);
        // lights with no programme running are treated as the row their `when_dark` names (checked to exist at load)
        const ControlKind* dark = k && k->hasLights() && !kLightsRunAProgramme && !k->whenDark.empty()
                                      ? rules.findControl(k->whenDark)
                                      : nullptr;
        if (dark) {
            out.controlOfName[c] = static_cast<uint32_t>(dark - rules.controls.data());
            out.darkOfName[c] = 1;
        } else if (k) out.controlOfName[c] = static_cast<uint32_t>(k - rules.controls.data());
        else if (junctionsOfControl[c] > 0)
            found.push_back("the lane graph has " + std::to_string(junctionsOfControl[c]) + " junctions controlled by \"" +
                            roads.controlNames[c] + "\", but rule set \"" + rules.name + "\" has no control." +
                            roads.controlNames[c]);
    }

    for (const FactSpec& fact : facts.all())
        checkNeeds(rules, "fact \"" + fact.name + "\"", fact.numbers, fact.settings, found);
    for (const ReaderSpec& reader : facts.readers())
        checkNeeds(rules, reader.name, reader.numbers, reader.settings, found);

    if (problems) problems->insert(problems->end(), found.begin(), found.end());
    return found.empty();
}

}  // namespace driving
