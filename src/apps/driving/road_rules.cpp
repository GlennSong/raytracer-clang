#include "apps/driving/road_rules.h"

#include "apps/driving/signal_programme.h"   // hasLightClock: whether the lights run a programme

#include <cstddef>
#include <map>

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

}  // namespace

const Device* RoadRules::device(const RuleSet& rules, engine::NameIndex kind) const {
    if (kind >= deviceOfKind.size()) return nullptr;
    const uint32_t d = deviceOfKind[kind];
    if (d >= rules.devices.size()) return nullptr;
    return &rules.devices[d];
}

bool buildRoadRules(const engine::RoadModel& model, const RuleSet& rules, const FactRegistry& facts, RoadRules& out,
                    std::vector<std::string>* problems) {
    const engine::LaneGraph& roads = model.laneLevel;
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

    // Each path's limit: the one the road model posts on it, or else its road class's default, which must be a speed.
    out.limitOfPath.assign(roads.paths.size(), 0);
    enum class Lack : uint8_t { NoDefault, NotASpeed };   // what a road class with unposted paths lacks
    std::map<std::string, Lack> lacking;                  // by class name, so the problems come in name order
    for (std::size_t p = 0; p < roads.paths.size(); ++p) {
        const engine::PathId path(static_cast<uint32_t>(p));
        const Real posted = model.postedLimit(path);
        if (posted > 0) {
            out.limitOfPath[p] = posted;
            continue;
        }
        const std::string& cls = model.roadClassOf(path);
        if (cls.empty()) continue;
        const Number* limit = rules.findDefaultLimit(cls);
        UnitFamily family = UnitFamily::Ratio;
        if (!limit) lacking[cls] = Lack::NoDefault;
        else if (!unitFamily(limit->unit, family) || family != UnitFamily::Speed) lacking[cls] = Lack::NotASpeed;
        else out.limitOfPath[p] = static_cast<Real>(limit->value);
    }
    for (const std::pair<const std::string, Lack>& c : lacking) {
        const std::string& cls = c.first;
        if (c.second == Lack::NoDefault)
            found.push_back("the lane graph has paths of road class \"" + cls + "\" with no posted limit, but rule set \"" +
                            rules.name + "\" has no " + kKeyDefaultLimits + "." + cls);
        else
            found.push_back("rule set \"" + rules.name + "\" gives " + kKeyDefaultLimits + "." + cls + " in \"" +
                            rules.findDefaultLimit(cls)->unit + "\", which is not a speed");
    }

    // Every control a junction states must be a row of the rule set's control table (a junction stating none is bound
    // to none).
    out.controlOfJunction.assign(roads.junctions.size(), kNoDevice);
    out.darkJunction.assign(roads.junctions.size(), 0);
    out.lightsRun = hasLightClock(facts);
    std::map<std::string, uint32_t> junctionsLacking;   // by control name, so the problems come in name order
    for (std::size_t j = 0; j < roads.junctions.size(); ++j) {
        const std::string& control = model.controlAt(engine::JunctionId(static_cast<uint32_t>(j)));
        if (control.empty()) continue;
        const ControlKind* k = rules.findControl(control);
        // lights with no programme running are treated as the row their `when_dark` names (checked to exist at load)
        const ControlKind* dark = k && k->hasLights() && !out.lightsRun && !k->whenDark.empty()
                                      ? rules.findControl(k->whenDark)
                                      : nullptr;
        if (dark) {
            out.controlOfJunction[j] = static_cast<uint32_t>(dark - rules.controls.data());
            out.darkJunction[j] = 1;
        } else if (k) out.controlOfJunction[j] = static_cast<uint32_t>(k - rules.controls.data());
        else ++junctionsLacking[control];
    }
    for (const std::pair<const std::string, uint32_t>& c : junctionsLacking)
        found.push_back("the lane graph has " + std::to_string(c.second) + " junctions controlled by \"" + c.first +
                        "\", but rule set \"" + rules.name + "\" has no control." + c.first);

    for (const FactSpec& fact : facts.all())
        checkNeeds(rules, "fact \"" + fact.name + "\"", fact.numbers, fact.settings, found);
    for (const ReaderSpec& reader : facts.readers())
        checkNeeds(rules, reader.name, reader.numbers, reader.settings, found);

    if (problems) problems->insert(problems->end(), found.begin(), found.end());
    return found.empty();
}

}  // namespace driving
